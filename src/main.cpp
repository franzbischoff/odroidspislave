
#include <Arduino.h>
#include "FanController.h"

// Sensor wire is plugged into port 2 on the Arduino.
#define SENSOR_PIN 2
// Choose a threshold in milliseconds between readings.
// A smaller value will give more updated results,
// while a higher value will give more accurate and smooth readings
#define SENSOR_THRESHOLD 1000
// PWM pin (4th on 4 pin fans)
#define PWM_PIN 9

typedef void (*voidFuncPtr)(void);
static volatile voidFuncPtr intSPIFunc;
// pins used for the connection with the sensor
// the other you need are controlled by the SPI library):
uint8_t tempData = 0;

// Initialize library
FanController fan(SENSOR_PIN, SENSOR_THRESHOLD, PWM_PIN);

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

void slave_begin()
{
  pinMode(MOSI, INPUT);
  pinMode(SCK, INPUT);
  pinMode(MISO, OUTPUT);
  pinMode(SS, INPUT);

  SPCR |= _BV(SPE);
}

void slave_end()
{
  SPCR &= ~_BV(SPE);
}

byte slave_transfer(byte _data)
{
  byte receive;

  asm volatile("nop");
  while (!(SPSR & _BV(SPIF)))
    ;
  receive = SPDR;
  SPDR = _data;

  return receive;
}

void get_instructions()
{
  tempData = slave_transfer(0x00);
  // SPI.endTransaction();
  Serial.print("Temp[C]=");
  Serial.println(tempData, DEC);
}

void setup()
{
  Serial.begin(9600);
  fan.begin();
  slave_begin();
  attachSPIInterrupt(get_instructions);
  fan.setDutyCycle(10);
  // give the sensor time to set up:
  delay(100);
}

void loop()
{
  // Call fan.getSpeed() to get fan RPM.
  Serial.print("Current speed: ");
  uint16_t rpms = fan.getSpeed(); // Send the command to get RPM
  Serial.print(rpms);
  Serial.println(" RPM");

    // Get new speed from Serial (0-100%)
    if (Serial.available() > 0)
    {
            // Parse speed
      uint16_t input = (uint16_t)Serial.parseInt();

      // Constrain a 0-100 range
      uint8_t target = max(min(input, 50), 0);

      if (target > 0)
      {

        // Print obtained value
        Serial.print("Setting duty cycle: ");
        Serial.println(target, DEC);

        // Set fan duty cycle
        fan.setDutyCycle(target);
      }
    }

  // Get duty cycle
  uint8_t dutyCycle = fan.getDutyCycle();
  Serial.print("Duty cycle: ");
  Serial.println(dutyCycle, DEC);
  delay(1000);
}
