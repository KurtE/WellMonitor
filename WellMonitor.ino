#include <EEPROM.h>
#include <Arduino.h>
#include "globals.h"
#include "Display.h"
#include <SPIN.h>
#include <ILI9341_t3n.h>
#include <ili9341_t3n_font_Arial.h>
#include <ili9341_t3n_font_ArialBold.h>
#include <Adafruit_NeoPixel.h>
#include <XPT2046_Touchscreen.h>
#include <Wire.h>
#include <Adafruit_SHT31.h>

#include <SPI.h>
#include <RH_RF95.h>
#include <RHHardwareSPI1.h>

#include <elapsedMillis.h>
#include <IntervalTimer.h>
#include <TimeLib.h>

#include "CurrentSensor.h"

#define MIN_DELTA_TO_REPORT 2
int last_val = -332767;    // setup to some value that we wont ever see



// MISO 5, MOSI 21 SCK 20
#define RF95_FREQ 915.0
//====================================================================================
// Globals
//====================================================================================
// SPI1 Miso=D5, Mosi=21, sck=20, CS=31
RH_RF95 rf95(RFM95_CS, RFM95_INT, hardware_spi1);
uint8_t master_node;
elapsedMillis time_to_update_time_temp;

uint8_t cur_sensor_index = 0;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, NEOPIXEL_PIN, NEO_RGB + NEO_KHZ800);
Adafruit_SHT31 sht31 = Adafruit_SHT31();

bool sht31_detected = false;   // Does this unit have an sht31? 


//====================================================================================
// Setup
//====================================================================================
void setup() {
  setSyncProvider(getTeensy3Time);

  // Use an IO pin to signal if we are the master or slave.
  pinMode(MASTER_SLAVE, INPUT_PULLDOWN);  // Now using PD as 3.3v next pin over


  while (!Serial && (millis() < 5000)) ;
  Serial.begin(115200);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  strip.setPixelColor(0, strip.Color(255, 0, 0));
  strip.show();
  delay(125);
  strip.setPixelColor(0, strip.Color(0, 255, 0));
  strip.show();
  delay(125);
  strip.setPixelColor(0, strip.Color(0, 0, 255));
  strip.show();
  delay(125);
  strip.setPixelColor(0, strip.Color(0, 0, 0));
  strip.show();

  master_node = digitalRead(MASTER_SLAVE) ? 0 : 1;

  if (master_node) {
    Serial.println("Master Node");
  } else {
    Serial.println("Slave Node");
  }

  InitTFTDisplay();

  // Lets init the time
  if (timeStatus() != timeSet) {
    Serial.println("Unable to sync with the RTC");
  } else {
    Serial.println("RTC has set the system time");
  }


  // Init the radio on the board - probably move to own function later
  SPI1.setMISO(RFM95_MISO);
  SPI1.setMOSI(RFM95_MOSI);
  SPI1.setSCK(RFM95_SCK);
  if (!rf95.init()) {
    Serial.println("RF95: init failed");
  } else {
    if (!rf95.setFrequency(RF95_FREQ)) {
      Serial.println("RF95: setFrequency failed");
    } else
      rf95.setTxPower(23, false);
  }

  // Setup your Analog settings
  analogReadResolution(11); // 11-bit resolution 0 to 2047
  analogReadAveraging(4); // 4,8,16, or 32 samples.

  // Hack lets see if we can detect if we have external PU on the SCL/SDA pins
  // If not then sht32 may not be installed ans Wire library may hang...
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  if (digitalRead(A4) && digitalRead(A5)) {
    if (sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
      sht31_detected = true;   // Does this unit have an sht31? 
    } else {
      Serial.println("Couldn't find SHT31");
    }

  } else {
    Serial.println("SHT31 - Not detected");
  }

  Serial.println("Before Init Sensors");
  CurrentSensor::initSensors();

}


//====================================================================================
// Main Loop
//====================================================================================
void loop() {

  // We now have the Interval timer doing most of the sensor work.
  // We simple look for it to signal us.
  uint32_t sensors_changed = CurrentSensor::any_sensor_changed;
  if (sensors_changed) {
    CurrentSensor::any_sensor_changed = 0;  // clear it out

    // update the displayed data.
    for (uint8_t sensor_index = 0; sensor_index < g_sensors_cnt; sensor_index++) {
      uint8_t sensor_updated = (sensors_changed >> 24) & 0xff;
      UpdateDisplaySensorData(sensor_index, sensor_updated);
      sensors_changed <<= 8;  // Should rework the order
    }

  }

  if (time_to_update_time_temp >= UPDATE_TIME_TEMP_MILLIS) {
    UpdateDisplayTempHumidy();
    UpdateDisplayDateTime();

    time_to_update_time_temp = 0;
  }
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

void printDigits(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

void digitalClockDisplay() {
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year());
  Serial.println();
}

