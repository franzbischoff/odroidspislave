
#include <Arduino.h>
#include <avr/wdt.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <FanController.h>

// DS18B20 Temperature Sensor
#define ONE_WIRE_BUS 4 // Digital pin 4
// PSU Control
#define ONLINE_PIN 5     // Digital pin 5
#define PSU_START_PIN 16 // Digital pin 16 / A2
#define PSU_OK_PIN 17    // Digital pin 17 / A3
// Sensor wire is plugged into port 2 on the Arduino.
#define FAN_TACH_PIN 2 // Digital pin 2, INT0, yellow wire on FAN
#define FAN_PWM_PIN 9  // Digital pin 9, PWM, Blue wire on FAN

// Pins 10, 11, 12 and 13 are reserved for SPI.

// Choose a threshold in milliseconds between readings.
// A smaller value will give more updated results,
// while a higher value will give more accurate and smooth readings
#define SENSOR_THRESHOLD 1000

/**** SPI Slave mode section */

typedef void (*voidFuncPtr)(void);
static volatile voidFuncPtr intSPIFunc;
volatile uint8_t spiRdy;
volatile uint8_t spiData;

void attachSPIInterrupt(void (*userFunc)(void))
{
  intSPIFunc = userFunc;
  SPCR |= _BV(SPIE);
}

void detachSPIInterrupt()
{
  SPCR &= ~_BV(SPIE);
}

ISR(SPI_STC_vect)
{
  if (intSPIFunc)
    intSPIFunc();
}

void spi_slave_begin()
{
  spiRdy = 0;
  spiData = 0;
  pinMode(MOSI, INPUT);
  pinMode(SCK, INPUT);
  pinMode(MISO, OUTPUT);
  pinMode(SS, INPUT);

  SPCR |= _BV(SPE);
}

void spi_slave_end()
{
  SPCR &= ~_BV(SPE);
}

uint8_t spi_slave_receive()
{
  uint8_t receive;

  asm volatile("nop");
  while (!(SPSR & _BV(SPIF)))
    ;
  receive = SPDR;

  return receive;
}

uint8_t spi_slave_transfer(uint8_t data)
{
  uint8_t receive;

  asm volatile("nop");
  while (!(SPSR & _BV(SPIF)))
    ;
  receive = SPDR;
  SPDR = data;

  return receive;
}

/**** SPI Slave mode section end */

// Arduino regular code

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature tempSensor(&oneWire);

// Create a new FanController instance.
FanController fan(FAN_TACH_PIN, SENSOR_THRESHOLD, FAN_PWM_PIN);

void get_instructions()
{
  spiData = spi_slave_receive();
  spiRdy = 1;
}

uint8_t system_online = 0;
uint8_t psu_is_on = 0;
uint8_t builtin_temp_check = 0;
uint8_t remote_temp_check = 0;
uint8_t use_builtin_temp = 0;
uint8_t remote_temp = 0;
unsigned long atimer = 0;
unsigned long spiTimer = 0;

void checkRemoteTemperature()
{
  Serial.println("DEBUG: Checking remote temperature");
  if (spiRdy == 1)
  {
    remote_temp = spiData;
    spiRdy = 0;
    Serial.print("Temp[C]=");
    Serial.println(remote_temp, DEC);
    remote_temp_check = 1;
    use_builtin_temp = 0;
    spiTimer = millis();
  }
}

void checkSystemOnline()
{
  if (digitalRead(ONLINE_PIN) == HIGH)
  {
    Serial.println("DEBUG: System is online");
    system_online = 1;
  }
  else
  {
    Serial.println("DEBUG: System is offline");
    system_online = 0;
  }
}

// Needs to check all possibilities, this is a sensitive part
void dealWithPSU()
{
  if (system_online == 1)
  {
    if (psu_is_on == 0)
    {
      Serial.println("DEBUG: System needs the PSU on");

      if (digitalRead(PSU_START_PIN) > 0)
      {
        Serial.println("DEBUG: But seems we already tried");

        if (digitalRead(PSU_OK_PIN) > 0)
        {
          Serial.println("DEBUG: Indeed PSU tells us it's on");
          psu_is_on = 1;
        }
        else
        {
          Serial.println("DEBUG: But PSU is not on, let's wait a bit more");
          // panic?
        }
      }
      else
      {
        if (digitalRead(PSU_OK_PIN) > 0)
        {
          Serial.println("DEBUG: PSU is already on, but the switch is off?");
          // panic?
        }

        Serial.println("DEBUG: Turning on the Switch");
        digitalWrite(PSU_START_PIN, HIGH);
      }
    }
    else
    {
      Serial.println("DEBUG: PSU is already on, nice job!");
    }
  }
  else
  {
    if (psu_is_on == 1)
    {
      Serial.println("DEBUG: System needs the PSU off");

      if (digitalRead(PSU_START_PIN) == 0)
      {
        Serial.println("DEBUG: But seems we already tried to stop");

        if (digitalRead(PSU_OK_PIN) == 0)
        {
          Serial.println("DEBUG: Indeed PSU is silent");
          psu_is_on = 0;
        }
        else
        {
          Serial.println("DEBUG: But PSU still on, let's wait a bit more");
          // panic?
        }
      }
      else
      {
        if (digitalRead(PSU_OK_PIN) == 0)
        {
          Serial.println("DEBUG: PSU is already off, but the switch is on?");
          // panic?
        }

        Serial.println("DEBUG: Turning off the Switch");
        digitalWrite(PSU_START_PIN, LOW);
      }
    }
    else
    {
      Serial.println("DEBUG: PSU is already off, nice job!");
    }
  }
}

// Is the builtin temp sensor working?
void checkBuiltinTemp()
{
  if (builtin_temp_check == 0)
  {
    Serial.println("DEBUG: Checking builtin temp");

    // Grab a count of devices on the wire
    uint8_t numberOfDevices = tempSensor.getDeviceCount();

    /*
     * Constructs DallasTemperature with strong pull-up turned on. Strong pull-up is mandated in DS18B20 datasheet for parasitic
     * power (2 wires) setup. (https://datasheets.maximintegrated.com/en/ds/DS18B20.pdf, p. 7, section 'Powering the DS18B20').
     */

    if (numberOfDevices == 0)
    {
      Serial.println("DEBUG: No devices found, check wiring");
      return;
    }
    else
    {
      Serial.print("DEBUG: Found ");
      Serial.print(numberOfDevices, DEC);
      Serial.println(" devices.");

      // // report parasite power requirements
      // Serial.print("DEBUG: Parasite power is: ");
      // if (tempSensor.isParasitePowerMode())
      //   Serial.println("ON");
      // else
      //   Serial.println("OFF");
    }

    tempSensor.requestTemperatures(); // Send the command to get temperatures
    float tempC = tempSensor.getTempCByIndex(0);

    if (tempC < 0)
    {
      Serial.println("DEBUG: reading error, check wiring");
      return;
    }

    Serial.print("InTemp[C]=");
    Serial.println(tempC);

    use_builtin_temp = 1;
    builtin_temp_check = 1;
  }
  else
  {
    Serial.println("DEBUG: Builtin temp already checked");
  }
}

float getBuiltinTemp()
{
  if (builtin_temp_check == 1)
  {
    tempSensor.requestTemperatures(); // Send the command to get temperatures
    float tempC = tempSensor.getTempCByIndex(0);

    if (tempC < 0)
    {
      Serial.println("DEBUG: reading error, check wiring");
      builtin_temp_check = 0;
      return -1.0f;
    }

    return tempC;
  } else {
    return -1.0f;
  }
}

float getRemoteTemp()
{
  float temp = -1.0f;

  if (remote_temp_check == 1)
  {
    temp = (float)remote_temp;
    temp /= 10.0f;
  }

  return temp;
}

void adjustFanSpeed()
{
  float temp = 0.0f;

  if (use_builtin_temp == 1)
  {
    Serial.println("DEBUG: Using builtin temp");
    temp = getBuiltinTemp();
  }
  else
  {
    Serial.println("DEBUG: Using remote temp");
    temp = getRemoteTemp();
  }

  if (temp < 0.0f)
  {
    Serial.println("DEBUG: Temp reading error, check wiring");
    return;
  }

  if (temp < 30.0f)
  {
    Serial.println("DEBUG: Temp is low, let's turn off the fan");
    fan.setDutyCycle(0); // Set fan duty cycle to 0%
  }
  else if (temp > 40.0f)
  {
    Serial.println("DEBUG: Temp is getting high, let's turn on the fan");
    fan.setDutyCycle(10); // Set fan duty cycle to 10%
  }
  else if (temp > 50.0f)
  {
    Serial.println("DEBUG: Temp is hot, let's increase the fan speed");
    fan.setDutyCycle(30); // Set fan duty cycle to 30%
  }
  else if (temp > 60.0f)
  {
    Serial.println("DEBUG: Temp is getting crazy, let's pump it up");
    fan.setDutyCycle(60); // Set fan duty cycle to 60%
  }
}

void setup()
{
  Serial.begin(9600);
  // FAN
  fan.begin();
  fan.setDutyCycle(15); // Set fan duty cycle to 10%

  // Start up Dallas OneWire
  tempSensor.begin();

  // SPI
  spi_slave_begin();
  attachSPIInterrupt(get_instructions);
  // call the toggle_led function every 1000 millis (1 second)

  // Online Check
  pinMode(ONLINE_PIN, INPUT);
  pinMode(PSU_OK_PIN, INPUT);
  pinMode(PSU_START_PIN, OUTPUT);
  digitalWrite(PSU_START_PIN, LOW); // don't start PSU yet

  wdt_disable();       /* Disable the watchdog and wait for more than 2 seconds */
  delay(3000);         /* Done so that the Arduino doesn't keep resetting infinitely in case of wrong configuration */
  wdt_enable(WDTO_2S); /* Enable the watchdog with a timeout of 2 seconds */
  // give some time to settle things
  delay(100);
  atimer = millis(); // starting timer
  spiTimer = millis();
}

void loop()
{
  unsigned long currentMillis = millis();
  if (atimer > currentMillis)
  {
    // millis() overflow
    atimer = currentMillis;
    spiTimer = currentMillis;
  }
  // Task 1: Check if system is online
  checkSystemOnline();

  // Task 2: Turn on/off PSU and check the results on next iteration
  dealWithPSU();

  // If the system is not running, don't need to do anything else
  // Wait a bit, because the CPU may be halted and PSU is refusing to stop
  // if(system_online == 0 || psu_is_on == 0) {
  //   delay(1000);
  //   return;
  // }

  // Task 3: Check Built-in temperature sensor
  checkBuiltinTemp();

  // Task 4: Wait for SPI temperatures
  checkRemoteTemperature();

  // Task 5: If SPI is silent for more than 20 seconds, use built-in temperature sensor to adjust fan speed
  if ((currentMillis - spiTimer) > 20000)
  {
    Serial.println("DEBUG: No remote temp for 20 sec");
    use_builtin_temp = 1;
    remote_temp_check = 0;
  }

  // Task 6: If SPI is silent for more than 1 minute, increase fan speed to a noisy level
  if ((currentMillis - spiTimer) > 60000)
  {
    Serial.println("DEBUG: No remote temp for 60 sec");
    fan.setDutyCycle(60); // Set fan duty cycle to 60%
  }
  else
  {
    adjustFanSpeed();
  }

  delay(1000);
  wdt_reset(); /* Reset the watchdog */
  atimer = currentMillis;
}
