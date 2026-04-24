
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
#define FAN_MAX_DUTY_CYCLE 50

// Pins 10, 11, 12 and 13 are reserved for SPI.

// Choose a threshold in milliseconds between readings.
// A smaller value will give more updated results,
// while a higher value will give more accurate and smooth readings
#define SENSOR_THRESHOLD 1000
#define SPI_TEMP_HEADER 0xAA

enum SpiDebugEvent : uint8_t
{
  SPI_DEBUG_NONE = 0,
  SPI_DEBUG_HEADER_OK,
  SPI_DEBUG_FRAME_VALID,
  SPI_DEBUG_BAD_CHECKSUM,
  SPI_DEBUG_RESYNC_HEADER,
  SPI_DEBUG_SS_RESYNC
};

/**** SPI Slave mode section */

typedef void (*voidFuncPtr)(void);
static volatile voidFuncPtr intSPIFunc;
volatile uint8_t spi_rdy;
volatile uint8_t spi_data;
volatile uint8_t spi_state;
volatile uint8_t spi_temp_candidate;
volatile uint8_t spi_frame_header;
volatile uint8_t spi_frame_temp;
volatile uint8_t spi_frame_checksum;
volatile uint8_t spi_frame_expected_checksum;
volatile uint8_t spi_debug_event;
volatile uint8_t spi_debug_pending;
volatile uint16_t spi_valid_frames;
volatile uint16_t spi_invalid_frames;

enum SpiFrameState : uint8_t
{
  SPI_WAIT_HEADER = 0,
  SPI_WAIT_TEMP = 1,
  SPI_WAIT_CHECKSUM = 2
};

static uint8_t make_temp_checksum(uint8_t temp)
{
  return (uint8_t)(SPI_TEMP_HEADER ^ temp);
}

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

static void spi_handle_received_byte(uint8_t received)
{
  if (spi_state == SPI_WAIT_HEADER)
  {
    if (received == SPI_TEMP_HEADER)
    {
      spi_frame_header = received;
      spi_debug_event = SPI_DEBUG_HEADER_OK;
      spi_debug_pending = 1;
      spi_state = SPI_WAIT_TEMP;
    }
    return;
  }

  if (spi_state == SPI_WAIT_TEMP)
  {
    spi_temp_candidate = received;
    spi_frame_temp = received;
    spi_state = SPI_WAIT_CHECKSUM;
    return;
  }

  spi_frame_checksum = received;
  spi_frame_expected_checksum = make_temp_checksum(spi_temp_candidate);

  if (received == spi_frame_expected_checksum)
  {
    spi_data = spi_temp_candidate;
    spi_rdy = 1;
    spi_valid_frames++;
    spi_debug_event = SPI_DEBUG_FRAME_VALID;
    spi_debug_pending = 1;
    spi_state = SPI_WAIT_HEADER;
    return;
  }

  spi_invalid_frames++;
  spi_debug_event = SPI_DEBUG_BAD_CHECKSUM;
  spi_debug_pending = 1;

  // Try to resync quickly if a new frame starts immediately after a bad checksum.
  if (received == SPI_TEMP_HEADER)
  {
    spi_frame_header = received;
    spi_debug_event = SPI_DEBUG_RESYNC_HEADER;
    spi_debug_pending = 1;
    spi_state = SPI_WAIT_TEMP;
  }
  else
  {
    spi_state = SPI_WAIT_HEADER;
  }
}

void spi_slave_begin()
{
  spi_rdy = 0;
  spi_data = 0;
  spi_state = SPI_WAIT_HEADER;
  spi_temp_candidate = 0;
  spi_frame_header = 0;
  spi_frame_temp = 0;
  spi_frame_checksum = 0;
  spi_frame_expected_checksum = 0;
  spi_debug_event = SPI_DEBUG_NONE;
  spi_debug_pending = 0;
  spi_valid_frames = 0;
  spi_invalid_frames = 0;
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
  spi_handle_received_byte(SPDR);
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
uint8_t last_ss_state = HIGH;
uint8_t fan_boot_test_done = 0;

void debug_dump_spi_packet()
{
  uint8_t frame_header = 0;
  uint8_t frame_temp = 0;
  uint8_t frame_checksum = 0;
  uint8_t frame_expected_checksum = 0;
  uint8_t debug_event = SPI_DEBUG_NONE;
  uint8_t have_debug = 0;
  uint16_t valid_frames = 0;
  uint16_t invalid_frames = 0;

  noInterrupts();
  if (spi_debug_pending == 1)
  {
    frame_header = spi_frame_header;
    frame_temp = spi_frame_temp;
    frame_checksum = spi_frame_checksum;
    frame_expected_checksum = spi_frame_expected_checksum;
    debug_event = spi_debug_event;
    valid_frames = spi_valid_frames;
    invalid_frames = spi_invalid_frames;
    spi_debug_pending = 0;
    have_debug = 1;
  }
  interrupts();

  if (have_debug == 0)
  {
    return;
  }

  if (debug_event == SPI_DEBUG_HEADER_OK)
  {
    Serial.print("DEBUG: SPI sync header detected: 0x");
    Serial.println(frame_header, HEX);
    return;
  }

  if (debug_event == SPI_DEBUG_FRAME_VALID)
  {
    Serial.print("DEBUG: SPI packet raw header=0x");
    Serial.print(frame_header, HEX);
    Serial.print(" temp=0x");
    Serial.print(frame_temp, HEX);
    Serial.print(" checksum=0x");
    Serial.print(frame_checksum, HEX);
    Serial.print(" valid_frames=");
    Serial.print(valid_frames);
    Serial.print(" invalid_frames=");
    Serial.println(invalid_frames);
    Serial.print("DEBUG: SPI packet temperature[C]=");
    Serial.println(frame_temp, DEC);
    return;
  }

  if (debug_event == SPI_DEBUG_BAD_CHECKSUM)
  {
    Serial.print("DEBUG: SPI packet invalid header=0x");
    Serial.print(frame_header, HEX);
    Serial.print(" temp=0x");
    Serial.print(frame_temp, HEX);
    Serial.print(" checksum=0x");
    Serial.print(frame_checksum, HEX);
    Serial.print(" expected=0x");
    Serial.print(frame_expected_checksum, HEX);
    Serial.print(" valid_frames=");
    Serial.print(valid_frames);
    Serial.print(" invalid_frames=");
    Serial.println(invalid_frames);
    return;
  }

  if (debug_event == SPI_DEBUG_RESYNC_HEADER)
  {
    Serial.println("DEBUG: SPI checksum failed but next byte looks like a new header, re-syncing");
    return;
  }

  if (debug_event == SPI_DEBUG_SS_RESYNC)
  {
    Serial.println("DEBUG: SPI CS released mid-frame, resetting parser to wait for next header");
  }
}

void check_spi_sync()
{
  uint8_t ss_state = digitalRead(SS);

  if (last_ss_state == LOW && ss_state == HIGH && spi_state != SPI_WAIT_HEADER)
  {
    noInterrupts();
    spi_state = SPI_WAIT_HEADER;
    spi_temp_candidate = 0;
    spi_debug_event = SPI_DEBUG_SS_RESYNC;
    spi_debug_pending = 1;
    interrupts();
  }

  last_ss_state = ss_state;
}

void run_fan_boot_test()
{
  fan.setDutyCycle(15);
  delay(1000);

  fan.setDutyCycle(50);
  delay(1000);

  fan.setDutyCycle(30);
  delay(1000);

  fan.setDutyCycle(15);
}

uint8_t get_builtin_sensor_count()
{
  uint8_t numberOfDevices = 0;

  detachSPIInterrupt();
  numberOfDevices = temp_sensor.getDeviceCount();
  attachSPIInterrupt(get_instructions);

  return numberOfDevices;
}

float read_builtin_temperature()
{
  float tempC = DEVICE_DISCONNECTED_C;

  detachSPIInterrupt();
  temp_sensor.requestTemperatures();
  tempC = temp_sensor.getTempCByIndex(0);
  attachSPIInterrupt(get_instructions);

  return tempC;
}

void check_remote_temperature()
{
  check_spi_sync();
  debug_dump_spi_packet();
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

void update_temperature_source()
{
  if (spi_timer < loop_timer && (loop_timer - spi_timer) > 30000)
  {
    if (use_builtin_temp == 0 || remote_temp_check == 1)
    {
      curr_status = 4;
      Serial.println("DEBUG: No remote temp for 30 sec");
    }
    use_builtin_temp = 1;
    remote_temp_check = 0;
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
      fan_boot_test_done = 0;
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
            fan_boot_test_done = 0;
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
    Serial.println("DEBUG: Pausing SPI IRQ while probing builtin temp sensor");
    uint8_t numberOfDevices = get_builtin_sensor_count();

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

    float tempC = read_builtin_temperature();

    if (tempC == DEVICE_DISCONNECTED_C)
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
    float tempC = read_builtin_temperature();

    if (tempC == DEVICE_DISCONNECTED_C)
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
  byte fan_speed = 0;

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
    fan.setDutyCycle(fan_speed); // Set fan duty cycle to 0%
  }
  else if (temp >= 40.0f)
  {
    Serial.println("DEBUG: Temp is getting high, let's turn on the fan");
    fan_speed = (byte)(2.5f * temp - 90.0f);
    if (fan_speed > FAN_MAX_DUTY_CYCLE)
    {
      fan_speed = FAN_MAX_DUTY_CYCLE;
    }
    fan.setDutyCycle(fan_speed); // Set fan duty cycle to 15%
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
  fan.setDutyCycle(15);

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
  last_ss_state = digitalRead(SS);

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

  // Always process SPI frames so remote temperature reception is visible
  // even when the system state machine is not yet fully online.
  check_remote_temperature();
  update_temperature_source();

  // Probe the builtin sensor so it is available as a fallback source.
  check_builtin_temp();

  // The boot test is only relevant while the system is online and the PSU is on.
  if (system_online == 1 && psu_is_on == 1)
  {
    if (fan_boot_test_done == 0)
    {
      run_fan_boot_test();
      fan_boot_test_done = 1;
    }

    // Task 5: If SPI is silent for more than 30 seconds, use built-in temperature sensor to adjust fan speed
    if (spi_timer < loop_timer)
    {
      // Task 6: If SPI is silent for more than 3 minute, increase fan speed to a noisy level
      if ((loop_timer - spi_timer) > 180000)
      {
        curr_status = 6;
        Serial.println("DEBUG: No remote temp for 180 sec");
        buzz_it(); // fan.setDutyCycle(60); // Set fan duty cycle to 60%
      }
    }
  }

  // Keep the fan under temperature control even while the PSU state machine
  // is waiting to turn off or already considers the system offline.
  adjust_fan_speed();

  status_code(curr_status);
  atimer = loop_timer;
  wdt_reset_fixed();
}
