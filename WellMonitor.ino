#include <SD.h>
#include <SD_t3.h>

//#include <WireKinetis.h>

#include <EEPROM.h>
#include <Arduino.h>
#include "globals.h"
#include "Display.h"
#include "RemoteData.h"
#include <SPIN.h>
#include <ILI9341_t3n.h>
#include <ili9341_t3n_font_Arial.h>
#include <ili9341_t3n_font_ArialBold.h>
#include <Adafruit_NeoPixel.h>
#include <XPT2046_Touchscreen.h>
#include <i2c_t3.h>
#include "SHT31.h"
#include <ADC.h>

#include <SPI.h>
//#include <RHDatagram.h>
#include <RH_RF95.h>
#include <RHHardwareSPI1.h>

#include <elapsedMillis.h>
#include <IntervalTimer.h>
#include <TimeLib.h>

#include "CurrentSensor.h"

#define MIN_DELTA_TO_REPORT 2
int last_val = -332767;    // setup to some value that we wont ever see
bool g_sd_detected = false;      // did we detect an sd card?



//====================================================================================
// Globals
//====================================================================================
// SPI1 Miso=D5, Mosi=21, sck=20, CS=31
uint8_t g_master_node;

elapsedMillis time_to_update_time_temp;


uint8_t cur_sensor_index = 0;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, NEOPIXEL_PIN, NEO_RGB + NEO_KHZ800);

#define UPDATE_LED_TIME_MS  5000
elapsedMillis time_to_update_led;
static uint32_t led_colors[4] = {strip.Color(16, 0, 0), strip.Color(0, 16, 0), strip.Color(0, 0, 16), strip.Color(0, 0, 0)};
uint8_t led_color_index = 0;

#ifdef ENABLE_SHT31
Adafruit_SHT31 sht31 = Adafruit_SHT31();
bool g_sht31_detected = false;   // Does this unit have an sht31?
#endif

//====================================================================================
// Setup
//====================================================================================
void setup() {
  pinMode(13, OUTPUT);
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
  time_to_update_led = 0;

  g_master_node = digitalRead(MASTER_SLAVE) ? 0 : 1;

  if (g_master_node) {
    Serial.println("Master Node");
  } else {
    Serial.println("Slave Node");
  }

  if (!(g_sd_detected = SD.begin(BUILTIN_SDCARD))) {
    Serial.println("SDCard failed to init");
  }

  InitTFTDisplay();

  // Lets init the time
  if (timeStatus() != timeSet) {
    Serial.println("Unable to sync with the RTC");
  } else {
    Serial.println("RTC has set the system time");
  }
  //
  if (g_master_node && g_sd_detected) {
    File dataFile = SD.open("well_log.csv", FILE_WRITE);
    if (dataFile) {
      time_t t = now();
      dataFile.printf("-1,RESET,%d/%d/%d %d:%02d:%02d\r\n", month(t), day(t), year(t) % 100,
                      hour(t), minute(t), second(t));
      dataFile.close();
    }
  }

  InitRemoteRadio();

  // Setup your Analog settings
  //  analogReadResolution(11); // 11-bit resolution 0 to 2047
  //  analogReadAveraging(4); // 4,8,16, or 32 samples.

  // Hack lets see if we can detect if we have external PU on the SCL/SDA pins
  // If not then sht31 may not be installed and Wire library may hang...
#ifdef ENABLE_SHT31
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  if (digitalRead(A4) && digitalRead(A5)) {
    if (sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
      g_sht31_detected = true;   // Does this unit have an sht31?
    } else {
      Serial.println("Couldn't find SHT31");
    }

  } else {
    Serial.println("SHT31 - Not detected");
  }
#endif
  if (g_master_node) {
    Serial.println("Master: Before Init Sensors");
    CurrentSensor::initSensors();
  }

  // Let the other side know we are here
#ifdef USE_RADIOHEAD_DATAGRAM
  SendRemotePing(true);
#endif
  // Debug
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(9, OUTPUT);

}


//====================================================================================
// Main Loop
//====================================================================================
void loop() {
  uint32_t start_time = millis();
  digitalWriteFast(4, HIGH);
  // We now have the Interval timer doing most of the sensor work.
  // We simple look for it to signal us.
  bool sensors_changed = CurrentSensor::checkSensors();
  sensors_changed |= ProcessRemoteMessages();

  if (sensors_changed) {
    Serial.println("==>Loop Sensor changed");
    digitalWriteFast(0, HIGH);
    CurrentSensor::any_sensor_changed = 0;  // clear it out

    // update the displayed data.
    for (uint8_t sensor_index = 0; sensor_index < g_sensors_cnt; sensor_index++) {
      if (UpdateDisplaySensorData(sensor_index)) {
#ifdef USE_RADIOHEAD_DATAGRAM
        if (g_master_node) {
          SendRemoteSensorData(sensor_index);  // Tell other side we have updated information.
        }
#endif
      }
    }


    digitalWriteFast(0, LOW);
    CurrentSensor::sensor_scan_state = SENSOR_SCAN_START; // tell the scan to sart up again.
  }
  // If we are master maybe send status update to remtoe
#ifndef USE_RADIOHEAD_DATAGRAM
  sendRemoteState();
#endif
  if (time_to_update_time_temp >= UPDATE_TIME_TEMP_MILLIS) {
    ReadTempHumiditySensor();
    UpdateDisplayDateTime();
    time_to_update_time_temp = 0;

  }
  // Process touch screen
  ProcessTouchScreen();
  digitalWriteFast(4, LOW);

  if (time_to_update_led > UPDATE_LED_TIME_MS) {
    strip.setPixelColor(0, led_colors[led_color_index++ & 0x3]);
    strip.show();
    time_to_update_led = 0;
  }

  if (Serial.available()) {
    while (Serial.read() != -1) ; // get rid of everything.
    CurrentSensor::show_sensor_data = !CurrentSensor::show_sensor_data;
  }
  uint32_t loop_time = millis() - start_time;
  if (loop_time < 5) {
    delay(5 - loop_time);
  }
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

// Update the system time...
void UpdateSystemTimeWithRemoteTime(time_t t) {  // in Main INO file
  time_t tnow = now();
  if ((timeStatus() == timeNotSet) || (tnow < t)) {
    uint32_t dt = (uint32_t)(t - tnow);
    if (dt > 10) {
      if (Serial)
        Serial.printf("Update time %x %x\n", tnow, t);
      Teensy3Clock.set(t);
      setTime(t);
      uint32_t dt = (uint32_t)(t - tnow);
  
      if (g_sd_detected) {
        File dataFile = SD.open("well_log.csv", FILE_WRITE);
        if (dataFile) {
          dataFile.printf("-1,CLOCK,%d/%d/%d %d:%02d:%02d,%d/%d/%d %d:%02d:%02d,%d\r\n",
                          month(tnow), day(tnow), year(tnow) % 100, hour(tnow), minute(tnow), second(tnow),
                          month(t), day(t), year(t) % 100, hour(t), minute(t), second(t),
                          dt);
          dataFile.close();
        }
      }
      CurrentSensor::updateStartTimes(dt);
    }
  }
}

