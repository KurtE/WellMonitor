//====================================================================================
// Functions to control and output to TFT display
//====================================================================================
//====================================================================================
// Include files
//====================================================================================
#include <Arduino.h>
#include "globals.h"
#include "display.h"
#include <TimeLib.h>
#include <ILI9341_t3n.h>
#include <ili9341_t3n_font_Arial.h>
#include <ili9341_t3n_font_ArialBold.h>
#include <Adafruit_NeoPixel.h>
#include <XPT2046_Touchscreen.h>


//====================================================================================
// globals
//====================================================================================
XPT2046_Touchscreen ts(TOUCH_CS);
ILI9341_t3n tft = ILI9341_t3n(TFT_CS, TFT_DC, TFT_RST, TFT_MOSI, TFT_SCK, TFT_MISO);
uint16_t g_display_t = 0;
uint16_t g_display_h = 0;
int     g_display_min = 99;   // some bogus value

static const uint16_t SENSOR_Y_STARTS[] = {TFT_WELL1_Y, TFT_WELL2_Y, TFT_PRESURE_Y, TFT_HEATER_Y};

//====================================================================================
// Init the display
//====================================================================================
void InitTFTDisplay(void)
{
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(ILI9341_BLACK);

  if (!ts.begin()) {
    Serial.println("Couldn't start touchscreen controller");
  }


  tft.setFont(Arial_14);
  tft.setTextColor(ILI9341_YELLOW);
  // Date/time - Will display

  tft.setCursor(TFT_TEMP_HEADER_X, TFT_TIMETEMP_Y);
  tft.print("Temp:");

  tft.setCursor(TFT_HUMIDITY_HEADER_X, TFT_TIMETEMP_Y);
  tft.print("Hum:");

  // Display Headers for the wells and the like
  tft.setFont(Arial_24);
  tft.setTextColor(ILI9341_YELLOW);

  tft.setCursor(TFT_TITLES_X, TFT_WELL1_Y);
  tft.print("Well 1:");
  tft.setCursor(TFT_TITLES_X, TFT_WELL2_Y);
  tft.print("Well 2:");
  tft.setCursor(TFT_TITLES_X, TFT_PRESURE_Y);
  tft.print("Pump:");
  tft.setCursor(TFT_TITLES_X, TFT_HEATER_Y);
  tft.print("Heat:");

  // lets experiment seeing where things fit:
  tft.setFont(Arial_14);
  tft.setTextColor(ILI9341_RED, ILI9341_BLACK);

  tft.setCursor(TFT_TEMP_X, TFT_TIMETEMP_Y);
  tft.print(g_display_t, DEC);

}


//====================================================================================
// UpdateDisplayTempHumidy - Update the displays Data/Time
//====================================================================================
void UpdateDisplayTempHumidy()
{
  if (!sht31_detected) {
    return; // no SHT31 detected.
  }
  static boolean sht31_read_started = false;  // sort of piece of ... 

  // If we have not started a read yet, do so now... 
  if (!sht31_read_started) {
    if (sht31.beginReadTempHum())
      sht31_read_started = true;
      return;
  }
  
  if (sht31.completeReadTempHum()) {

    tft.setFont(Arial_14);
    tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
    // Date/time - Will display

    float t = sht31.temperature();
    float h = sht31.humidity();

    // Make sure they are valid
    uint16_t tint = (uint16_t)(t * 1.8 + 32.0);
    if (tint != g_display_t) {
      g_display_t = tint;
      tft.setCursor(TFT_TEMP_X, TFT_TIMETEMP_Y);
      tft.print(g_display_t, DEC);
    }

    uint16_t hint = (uint16_t)h;
    if (hint != g_display_h) {
      g_display_h = hint;
      tft.setCursor(TFT_HUMIDITY_X, TFT_TIMETEMP_Y);
      tft.print(g_display_h, DEC);
    }
  }
  // Tell system that next call should start new read...
  sht31_read_started = false;
}

//====================================================================================
// UpdateDisplayTime - Update the displays Data/Time
//====================================================================================

void UpdateDisplayDateTime() {
  time_t tnow = now();
  if (minute(tnow) != g_display_min) {
    g_display_min = minute(tnow);

    tft.setFont(Arial_14);
    tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
    tft.setCursor(TFT_TIME_X, TFT_TIMETEMP_Y);
    tft.printf("%s %d %d %d:%02d", monthShortStr(month(tnow)), day(tnow), year(tnow) % 100,
               hour(tnow), minute(tnow));

  }

}


//====================================================================================
// Helper to blank to end of text line.
//====================================================================================
void EraseRestOfTextLine(const ILI9341_t3_font_t &f) {
  int16_t x;
  int16_t y;
  tft.getCursor(&x, &y);
  tft.fillRect(x, y, tft.width(), f.line_space, ILI9341_BLACK); // width will be truncated.
}

//====================================================================================
// UpdateDisplaySensorData - Update Sensor data
//====================================================================================
#define SENSOR_ON (SENSOR_UPDATE_ON_BOOT_DETECTED | SENSOR_UPDATE_ON_DETECTED)
void UpdateDisplaySensorData(uint8_t iSensor, uint8_t update_state) {
  uint16_t y_start = SENSOR_Y_STARTS[iSensor];
  if (update_state & (SENSOR_ON | SENSOR_UPDATE_OFF_DETECTED)) {
    Serial.printf("UDSD: %d %d %d\n", iSensor, update_state, g_Sensors[iSensor]->state());
    if ((update_state & SENSOR_ON) && 
        (!(update_state & SENSOR_UPDATE_OFF_DETECTED) || (g_Sensors[iSensor]->onTime() > g_Sensors[iSensor]->offTime())) ) {

      // We have a logical On condition and if there was on off it was before the on, so show on.
      tft.setFont(Arial_14);
      tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
      tft.setCursor(TFT_STATE_X, y_start + TFT_STATE_OFFSET_Y);
      time_t t = g_Sensors[iSensor]->onTime();
      tft.printf("ON %d/%d/%d %d:%02d", month(t), day(t), year(t)%100, hour(t), minute(t));
      EraseRestOfTextLine(Arial_14);

      tft.setFont(Arial_14);
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
      tft.setCursor(TFT_STATE_X, y_start + TFT_STATE_ROW2_OFFSET_Y);
      tft.printf("C:%d", g_Sensors[iSensor]->curValue());
      EraseRestOfTextLine(Arial_14);
    } else {
      // It turned off and no on since.
      tft.setFont(Arial_14);
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
      tft.setCursor(TFT_STATE_X, y_start + TFT_STATE_OFFSET_Y);
      time_t t = g_Sensors[iSensor]->offTime();
      time_t ton = g_Sensors[iSensor]->onTime();
      tft.printf("OFF %d/%d/%d %d:%02d", month(t), day(t), year(t)%100, hour(t), minute(t));
      EraseRestOfTextLine(Arial_14);

      // lets update the display information maybe delta time
      tft.setFont(Arial_14);
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
      tft.setCursor(TFT_STATE_X, y_start + TFT_STATE_ROW2_OFFSET_Y);
      t -= ton;   // We have the number of seconds that the sensor was on. 
      if (elapsedDays(t)) {
        tft.printf("OT: %dD %d:%02d A:%d", elapsedDays(t), hour(t), minute(t), g_Sensors[iSensor]->avgValue());
      } else {
        tft.printf("OT: %d:%02d:%02d A:%d", hour(t), minute(t), second(t), g_Sensors[iSensor]->avgValue());
      }
      EraseRestOfTextLine(Arial_14);
    }
  } else if (update_state & SENSOR_UPDATE_DONE_NEW_VALUE) {
    // We have a changed value if state is on update the display
    if (g_Sensors[iSensor]->state()) {
      tft.setFont(Arial_14);
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
      tft.setCursor(TFT_STATE_X, y_start + TFT_STATE_ROW2_OFFSET_Y);
      tft.printf("C:%d M:%d X:%d A:%d", g_Sensors[iSensor]->curValue(),
                 g_Sensors[iSensor]->minValue(), g_Sensors[iSensor]->maxValue(), g_Sensors[iSensor]->avgValue());
      EraseRestOfTextLine(Arial_14);
    }

  }
}


