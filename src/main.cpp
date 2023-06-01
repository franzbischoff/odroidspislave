
#include <Arduino.h>
#include <avr/wdt.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <FanController.h>

// DS18B20 Temperature Sensor
#define ONE_WIRE_BUS 4 // Digital pin 4
// PSU Control
#define PSU_START_PIN 16 // Digital pin 16 / A2
#define PSU_OK_PIN 17    // Digital pin 17 / A3
#define ONLINE_PIN 19    // Digital pin 19 / A5
// Sensor wire is plugged into port 2 on the Arduino.
#define FAN_TACH_PIN 2 // Digital pin 2, INT0, yellow wire on FAN
#define FAN_PWM_PIN 9  // Digital pin 9, PWM, Blue wire on FAN

#define STATUS_LED 7 // Digital pin 7
#define BUZZER 3     // Digital pin 3 (PWM at 490hz)

// Pins 10, 11, 12 and 13 are reserved for SPI.

// Choose a threshold in milliseconds between readings.
// A smaller value will give more updated results,
// while a higher value will give more accurate and smooth readings
#define SENSOR_THRESHOLD 1000

/**** SPI Slave mode section */

typedef void (*voidFuncPtr)(void);
static volatile voidFuncPtr intSPIFunc;
volatile uint8_t spi_rdy;
volatile uint8_t spi_data;

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
  spi_rdy = 0;
  spi_data = 0;
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
OneWire one_wire(ONE_WIRE_BUS);

// Pass our one_wire reference to Dallas Temperature sensor
DallasTemperature temp_sensor(&one_wire);

// Create a new FanController instance.
FanController fan(FAN_TACH_PIN, SENSOR_THRESHOLD, FAN_PWM_PIN);

void get_instructions()
{
  spi_data = spi_slave_receive();
  spi_rdy = 1;
}

uint8_t system_online = 0;
uint8_t psu_is_on = 0;
uint8_t builtin_temp_check = 0;
uint8_t remote_temp_check = 0;
uint8_t use_builtin_temp = 0;
uint8_t remote_temp = 0;
uint8_t curr_status = 1;
unsigned long atimer = 0;
unsigned long spi_timer = 0;
unsigned long sys_timer = 0;
unsigned long loop_timer = 0;

void check_remote_temperature()
{
  Serial.println("DEBUG: Checking remote temperature");
  if (spi_rdy == 1)
  {
    remote_temp = spi_data;
    spi_rdy = 0;
    Serial.print("Temp[C]=");
    Serial.println(remote_temp, DEC);
    remote_temp_check = 1;
    use_builtin_temp = 0;
    spi_timer = millis();
  }
}

void check_system_online()
{
  if (digitalRead(ONLINE_PIN) == HIGH)
  {
    if (system_online == 0)
    {
      Serial.println("DEBUG: System became online");
      sys_timer = millis();
      system_online = 1;
    }
    else
    {
      Serial.println("DEBUG: System is online");
    }
    curr_status = 2;
  }
  else
  {
    if (system_online == 1)
    {
      Serial.println("DEBUG: System became offline");
      sys_timer = millis();
      system_online = 0;
    }
    else
    {
      Serial.println("DEBUG: System is offline");
    }
    curr_status = 1;
  }
}

// Needs to check all possibilities, this is a sensitive part
void deal_with_psu()
{
  int value = 0;

  if (system_online == 1)
  {
    if (psu_is_on == 0)
    {
      curr_status = 3;
      Serial.println("DEBUG: System needs the PSU on");

      if ((value = digitalRead(PSU_START_PIN)) > 0)
      {
        Serial.print("DEBUG: But seems we already tried: ");
        Serial.println(value);

        if ((value = analogRead(PSU_OK_PIN)) > 100)
        {
          Serial.print("DEBUG: Indeed PSU tells us it's on: ");
          Serial.println(value);
          psu_is_on = 1;
        }
        else
        {
          curr_status = 5;
          Serial.print("DEBUG: But PSU is not on, let's wait a bit more: ");
          Serial.println(value);
          // panic?
        }
      }
      else
      {
        if ((value = analogRead(PSU_OK_PIN)) > 100)
        {
          curr_status = 5;
          Serial.print("DEBUG: PSU is already on, but the switch is off? ");
          Serial.println(value);
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
      curr_status = 3;
      // Wait 10 seconds before turning off the PSU. The system may be just rebooting
      if ((sys_timer < loop_timer) && ((loop_timer - sys_timer) > 10000))
      {

        Serial.println("DEBUG: System needs the PSU off");

        if (digitalRead(PSU_START_PIN) == 0)
        {
          Serial.println("DEBUG: But seems we already tried to stop");

          if ((value = analogRead(PSU_OK_PIN)) < 100)
          {
            Serial.print("DEBUG: Indeed PSU is silent: ");
            Serial.println(value);
            psu_is_on = 0;
          }
          else
          {
            curr_status = 5;
            Serial.print("DEBUG: But PSU still on, let's wait a bit more: ");
            Serial.println(value);
            // panic?
          }
        }
        else
        {
          if ((value = analogRead(PSU_OK_PIN)) < 100)
          {
            curr_status = 5;
            Serial.print("DEBUG: PSU is already off, but the switch is on? ");
            Serial.println(value);
            // panic?
          }

          Serial.println("DEBUG: Turning off the Switch");
          digitalWrite(PSU_START_PIN, LOW);
        }
      }
      else
      {
        Serial.println("DEBUG: Waiting 10s before turning off the PSU");
      }
    }
    else
    {
      Serial.println("DEBUG: PSU is already off, nice job!");
    }
  }
}

// Is the builtin temp sensor working?
void check_builtin_temp()
{
  if (builtin_temp_check == 0)
  {
    Serial.println("DEBUG: Checking builtin temp");

    // Grab a count of devices on the wire
    uint8_t numberOfDevices = temp_sensor.getDeviceCount();

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
      // if (temp_sensor.isParasitePowerMode())
      //   Serial.println("ON");
      // else
      //   Serial.println("OFF");
    }

    temp_sensor.requestTemperatures(); // Send the command to get temperatures
    float tempC = temp_sensor.getTempCByIndex(0);

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
}

float get_builtin_temp()
{
  if (builtin_temp_check == 1) // && use_builtin_temp == 1)
  {
    temp_sensor.requestTemperatures(); // Send the command to get temperatures
    float tempC = temp_sensor.getTempCByIndex(0);

    if (tempC < 0)
    {
      Serial.println("DEBUG: reading error, check wiring");
      builtin_temp_check = 0;
      return -1.0f;
    }
    else
    {
      Serial.print("InTemp[C]=");
      Serial.println(tempC);
    }

    return tempC;
  }
  else
  {
    return -1.0f;
  }
}

float get_remote_temp()
{
  float temp = -1.0f;

  if (remote_temp_check == 1)
  {
    temp = (float)remote_temp;
  }

  return temp;
}

void adjust_fan_speed()
{
  float temp = 0.0f;

  if (use_builtin_temp == 1)
  {
    Serial.println("DEBUG: Using builtin temp");
    temp = get_builtin_temp();
  }
  else
  {
    Serial.println("DEBUG: Using remote temp");
    temp = get_remote_temp();
    Serial.println(temp);
  }

  if (temp < 0.0f)
  {
    Serial.println("DEBUG: Temp reading error, check wiring");
    return;
  }

  if (temp < 35.0f)
  {
    Serial.println("DEBUG: Temp is low, let's turn off the fan");
    fan.setDutyCycle(0); // Set fan duty cycle to 0%
  }
  else if (temp > 60.0f)
  {
    Serial.println("DEBUG: Temp is getting crazy, let's pump it up");
    fan.setDutyCycle(60); // Set fan duty cycle to 60%
  }
  else if (temp > 50.0f)
  {
    Serial.println("DEBUG: Temp is hot, let's increase the fan speed");
    fan.setDutyCycle(30); // Set fan duty cycle to 30%
  }
  else if (temp > 45.0f)
  {
    Serial.println("DEBUG: Temp is getting high, let's turn on the fan");
    fan.setDutyCycle(20); // Set fan duty cycle to 20%
  }
  else if (temp > 40.0f)
  {
    Serial.println("DEBUG: Temp is getting high, let's turn on the fan");
    fan.setDutyCycle(15); // Set fan duty cycle to 15%
  }
}

void status_code(uint8_t code)
{

  // total time must not exceed 2 second
  int16_t time_left = 1650;

  uint16_t on_time = (time_left / code) - 150;

  for (uint8_t i = 0; i < code; i++)
  {
    digitalWrite(STATUS_LED, HIGH);
    delay(on_time);
    digitalWrite(STATUS_LED, LOW);
    delay(150);

    time_left -= (on_time + 150);
  }

  if (time_left < 0 || time_left > 2000)
  {
    delay(0);
  }
  else
  {
    delay(350);
  }
}

void buzz_it()
{
  tone(BUZZER, 512, 300);
  delay(300);
  tone(BUZZER, 1024, 500);
}

void post_beep()
{
  tone(BUZZER, 1024, 200);
}

void wdt_reset_fixed()
{
  wdt_reset(); /* Reset the watchdog */
  // set up WDT interrupt
  WDTCSR = (1 << WDCE) | (1 << WDE);
  // Start watchdog timer with 4s prescaller
  WDTCSR = (1 << WDIE) | (1 << WDE) | (1 << WDP3) | (1 << WDP0);
}

void setup()
{
  wdt_disable(); // Disable the watchdog to avoid infinite reset loop
  Serial.begin(9600);
  // FAN
  fan.begin();
  fan.setDutyCycle(15); // Set fan duty cycle to 10%

  // Start up Dallas OneWire
  temp_sensor.begin();

  // SPI
  spi_slave_begin();
  attachSPIInterrupt(get_instructions);
  // call the toggle_led function every 1000 millis (1 second)

  // Online Check
  pinMode(ONLINE_PIN, INPUT);
  pinMode(PSU_OK_PIN, INPUT);
  pinMode(PSU_START_PIN, OUTPUT);
  digitalWrite(PSU_START_PIN, LOW); // don't start PSU yet

  pinMode(BUZZER, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  atimer = millis(); // starting timer
  spi_timer = millis();

  wdt_enable(WDTO_4S); /* Enable the watchdog with a timeout of 4 seconds */
  // give some time to settle things
  delay(100);
  post_beep();
}

void loop()
{
  loop_timer = millis();
  if (atimer > loop_timer)
  {
    // millis() overflow
    atimer = loop_timer;
    spi_timer = loop_timer;
  }

  // Task 1: Check if system is online
  check_system_online();

  // Task 2: Turn on/off PSU and check the results on next iteration
  deal_with_psu();

  // If the system is not running, don't need to do anything else
  // Wait a bit, because the CPU may be halted and PSU is refusing to stop
  if (system_online == 1 && psu_is_on == 1)
  {
    // Task 3: Check Built-in temperature sensor
    check_builtin_temp();

    // Task 4: Wait for SPI temperatures
    check_remote_temperature();

    // Task 5: If SPI is silent for more than 30 seconds, use built-in temperature sensor to adjust fan speed
    if (spi_timer < loop_timer)
    {
      if ((loop_timer - spi_timer) > 30000)
      {
        curr_status = 4;
        Serial.println("DEBUG: No remote temp for 30 sec");
        use_builtin_temp = 1;
        remote_temp_check = 0;
      }

      // Task 6: If SPI is silent for more than 3 minute, increase fan speed to a noisy level
      if ((loop_timer - spi_timer) > 180000)
      {
        curr_status = 6;
        Serial.println("DEBUG: No remote temp for 180 sec");
        buzz_it(); // fan.setDutyCycle(60); // Set fan duty cycle to 60%
      }
    }

    adjust_fan_speed();
  }

  status_code(curr_status);
  atimer = loop_timer;
  Serial.println("DEBUG: End of cicle");
  wdt_reset_fixed();
}
