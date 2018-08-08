//====================================================================================
// Functions to control and output to TFT display
//====================================================================================
//====================================================================================
// Include files
//====================================================================================
#include <Arduino.h>
#include "globals.h"
#include "display.h"
#include "RemoteData.h"
#include <TimeLib.h>
#include <ILI9341_t3n.h>
#include <ili9341_t3n_font_Arial.h>
#include <ili9341_t3n_font_ArialBold.h>
#include <Adafruit_NeoPixel.h>
#include <XPT2046_Touchscreen.h>
#include <SD.h>
//====================================================================================
// globals
//====================================================================================
XPT2046_Touchscreen ts(TFT_TCS, TFT_TIRQ);
ILI9341_t3n tft = ILI9341_t3n(TFT_CS, TFT_DC, TFT_RST, TFT_MOSI, TFT_SCK, TFT_MISO);
uint16_t  g_current_temp = 0;
uint16_t  g_current_humidity = 0;

int       g_display_min = 99;   // some bogus value
int16_t   g_display_date_last_x = 0;   // remember date end of output from last write
uint32_t  g_tft_backlight_timer; // When was the last time the touch screen was touched.
uint16_t  g_backlight_value = 0; // what is the current backlight 0-255

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
  tft.setFont(Arial_16);
  tft.setTextColor(ILI9341_YELLOW);

  tft.setCursor(TFT_TITLES_X, TFT_WELL1_Y);
  tft.print("Well1:");
  tft.setCursor(TFT_TITLES_X, TFT_WELL2_Y);
  tft.print("Well2:");
  tft.setCursor(TFT_TITLES_X, TFT_PRESURE_Y);
  tft.print("Pump:");
  tft.setCursor(TFT_TITLES_X, TFT_HEATER_Y);
  tft.print("Heat:");

  // lets experiment seeing where things fit:
  tft.setFont(Arial_14);
  tft.setTextColor(ILI9341_RED, ILI9341_BLACK);

  tft.setCursor(TFT_TEMP_X, TFT_TIMETEMP_Y);
  tft.print(g_current_temp, DEC);

  // Lets try playing with backlight to see how it works
  analogWrite(TFT_BACKLIGHT, 255);  // Turn it off...
  delay(1000);
  analogWrite(TFT_BACKLIGHT, 128);
  delay(1000);
  analogWrite(TFT_BACKLIGHT, 64);
  delay(1000);
  SetFullTFTBacklight();
}


//====================================================================================
// ReadTempHumiditySensor - Update the displays Data/Time
//====================================================================================
uint32_t ReadTempHumiditySensor()
{
#ifdef ENABLE_SHT31
  if (!g_sht31_detected) {
    return 0xffffffff; // no SHT31 detected. probably won't call again...
  }
  pinMode(9, OUTPUT);
  static boolean sht31_read_started = false;  // sort of piece of ...

  // If we have not started a read yet, do so now...
  if (!sht31_read_started) {
    if (sht31.beginReadTempHum())
      sht31_read_started = true;
    return TEMP_READ_TIME_MILLIS;
  }

  digitalWriteFast(9, HIGH);
  if (sht31.completeReadTempHum()) {
    float t = sht31.temperature();
    float h = sht31.humidity();
    uint16_t tint = (uint16_t)t;
    uint16_t hint = (uint16_t)h;
    if (UpdateTempHumidity(tint, hint, true)) {
      // We updated temp and/or humidity. so send message
      //SendRemoteTempHumidityMsg();
    }
  }
  // Tell system that next call should start new read...
  digitalWriteFast(9, LOW);
  sht31_read_started = false;
  return UPDATE_TIME_TEMP_MILLIS;
#else
  return 0xffffffff; // no SHT31 detected. probably won't call again...
#endif
}

//====================================================================================
// UpdateTempHumidity - Update the displays Data/Time
//====================================================================================
bool UpdateTempHumidity(uint16_t temp, uint16_t humidity, bool local_data)
{
#ifdef ENABLE_SHT31
  bool fChanged = false;
  // Pass 1 if data is local display or if remote and no loacl display
  if (!local_data && g_sht31_detected)
    return false; //

  tft.setFont(Arial_14);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  // Date/time - Will display

  if (temp != g_current_temp) {
    g_current_temp = temp;
    tft.setCursor(TFT_TEMP_X, TFT_TIMETEMP_Y);
    tft.print(g_current_temp, DEC);
    fChanged = true;
  }

  if (humidity != g_current_humidity) {
    g_current_humidity = humidity;
    tft.setCursor(TFT_HUMIDITY_X, TFT_TIMETEMP_Y);
    tft.print(g_current_humidity, DEC);
    fChanged = true;
  }
  return fChanged && local_data;
#else
  return false;
#endif  
}

//====================================================================================
// UpdateDisplayTime - Update the displays Data/Time
//====================================================================================

bool UpdateDisplayDateTime() {
  time_t tnow = now();
  bool date_changed = false;
  if (minute(tnow) != g_display_min) {
    int16_t x, y;
    g_display_min = minute(tnow);

    tft.setFont(Arial_14);
    tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
    tft.setCursor(TFT_TIME_X, TFT_TIMETEMP_Y);
    tft.printf("%s %d %d %d:%02d", monthShortStr(month(tnow)), day(tnow), year(tnow) % 100,
               hour(tnow), minute(tnow));
    tft.getCursor(&x, &y);
    if (x < g_display_date_last_x) {
      tft.fillRect(x, y, g_display_date_last_x - x, Arial_14.line_space, ILI9341_BLACK);
    }
    g_display_date_last_x = x;

    // See if the date has changed
    if (previousMidnight(tnow) != CurrentSensor::todaysStartTime()) {
      CurrentSensor::todaysStartTime(previousMidnight(tnow)); // update our start timer time
      date_changed = true;
    }
  }
  return date_changed;
}


//====================================================================================
// Helper to blank to end of text line.
//====================================================================================
void EraseRestOfTextLine(int16_t y_text, const ILI9341_t3_font_t &f) {
  int16_t x;
  int16_t y;
  tft.getCursor(&x, &y);
  // Make sure we did not wrap around to next line... 
  if (y == y_text) {
    tft.fillRect(x, y, tft.width(), f.line_space, ILI9341_BLACK); // width will be truncated.
  }
}

//====================================================================================
// DisplayCenterPoints - Debug information for sensors
//====================================================================================
void DisplayCenterPoints() {
  if (!g_master_node) return; // only do on master

  for (uint8_t iSensor = 0; iSensor < g_sensors_cnt; iSensor++) {
    CurrentSensor *psensor = g_Sensors[iSensor];
    if (psensor->calibrating() == CurrentSensor::CALIBRATE_DONE_DISPLAY) {
      uint16_t cp = psensor->centerPoint();
      uint16_t y_start = SENSOR_Y_STARTS[iSensor];
      psensor->calibrating(CurrentSensor::CALIBRATE_DONE);
      tft.setFont(Arial_10);
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
      tft.setCursor(4, y_start + TFT_STATE_ROW2_OFFSET_Y);
      tft.printf("%d", cp);
    }
  }
}

//====================================================================================
// ShowLoopStatus - Updates every so often to let know program is not hung
//====================================================================================
void ShowLoopStatus(uint8_t loop_status_count) 
{
  tft.setTextColor((loop_status_count & 1)?ILI9341_WHITE : ILI9341_BLACK, ILI9341_BLACK);
  tft.setFont(Arial_10);
  tft.setCursor(2, 225);  //
  tft.print("*");
}

//====================================================================================
// ShowStatusMessage - Display a status message 
//====================================================================================
void ShowStatusMessage(uint8_t error_level, const char *psz, uint32_t val, bool send_remote) {
  switch (error_level) {
    case ERROR_LEVEL_NONE:
    default:
    case ERROR_LEVEL_INFO: tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK); break;
    case ERROR_LEVEL_WARN: tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK); break;
    case ERROR_LEVEL_ERROR: tft.setTextColor(ILI9341_RED, ILI9341_BLACK); break;
  }
  tft.setFont(Arial_10);
  tft.setCursor(TFT_ERROR_X, TFT_ERROR_Y);  
  if (psz) tft.printf(psz, val);

  EraseRestOfTextLine(TFT_ERROR_Y, Arial_10);

  if (send_remote) {
    sendRemoteStatusMsg(error_level, psz, val);
  }
}

//====================================================================================
// GenerateDisplayTime
//====================================================================================
void GenerateDisplayTime(time_t t, char *pstr) {
  time_t midnight = CurrentSensor::todaysStartTime();

  if (t > midnight) {
    sprintf(pstr, "Today %d:%02d", hour(t), minute(t));
  } else if (t > (midnight - SECS_PER_DAY)) {
    sprintf(pstr, "Yesterday %d:%02d", hour(t), minute(t));
  } else {
    sprintf(pstr, "%d/%d/%d %d:%02d", month(t), day(t), year(t) % 100, hour(t), minute(t));
  }
}
//====================================================================================
// UpdateDisplaySensorData - Update Sensor data
//====================================================================================
// Maybe should move this somewhere else???
bool UpdateDisplaySensorData(uint8_t iSensor, bool date_changed) {
  CurrentSensor *psensor = g_Sensors[iSensor];

  uint16_t y_start = SENSOR_Y_STARTS[iSensor];
  bool send_remote_update = false;
  time_t t = psensor->offTime();
  time_t ton = psensor->onTime();
  char date_time_str[32];
  //Serial.printf("*** UDSD *** %d %d %d %d %d\n", iSensor, psensor->state(), psensor->displayState(),
  //              psensor->curValue(), psensor->displayVal());
  // If the state changed or date changed, than redraw most everything. 
  // currently this is slightly off as 2nd line in on condition has other data... 
  // But this is in process of being reorganized. 
  if ((psensor->state() != psensor->displayState()) || date_changed) {
    psensor->displayState(psensor->state());
    if (psensor->state()) {
      // We have a logical On condition and if there was on off it was before the on, so show on.
      tft.setFont(Arial_14);
      tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
      tft.setCursor(TFT_STATE_X, y_start + TFT_STATE_OFFSET_Y);
      time_t t = psensor->onTime();
      GenerateDisplayTime(t, date_time_str);
      tft.printf("ON %s", date_time_str);
      EraseRestOfTextLine(y_start + TFT_STATE_OFFSET_Y, Arial_14);

      tft.setFont(Arial_14);
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
      tft.setCursor(TFT_STATE_X, y_start + TFT_STATE_ROW2_OFFSET_Y);
      tft.printf("C:%d", psensor->curValue());
      EraseRestOfTextLine(y_start + TFT_STATE_ROW2_OFFSET_Y, Arial_14);
      send_remote_update = true;   // Lets send all on messages
      psensor->displayVal(0xffff);  // clear the remembered value...

      EventList::LogEvent(EventList::SENSOR_ON, iSensor, t, 0, 0);
    
    } else {
      tft.setFont(Arial_12);
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
      tft.setCursor(TFT_STATE_X, y_start + TFT_STATE_OFFSET_Y);
      GenerateDisplayTime(t, date_time_str);
      tft.printf("OFF %s", date_time_str);

      time_t delta_t = t - ton;
      tft.printf(" DT: %d:%02d:%02d", hour(delta_t), minute(delta_t), second(delta_t));

      EraseRestOfTextLine(y_start + TFT_STATE_OFFSET_Y, Arial_14);

      // lets update the display information maybe delta time
      tft.setFont(Arial_12);
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
      tft.setCursor(TFT_STATE_X, y_start + TFT_STATE_ROW2_OFFSET_Y);
      if (elapsedDays(delta_t)) {
        tft.printf("OT: %dD %d:%02d A:%d", elapsedDays(delta_t), hour(delta_t), minute(delta_t), psensor->avgValue());
      } else {
        tft.printf("OT: %d:%02d:%02d A:%d", hour(delta_t), minute(delta_t), second(delta_t), psensor->avgValue());
      }
      EraseRestOfTextLine(y_start + TFT_STATE_ROW2_OFFSET_Y, Arial_14);
      send_remote_update = true;   // Lets send all on messages
      EventList::LogEvent(EventList::SENSOR_OFF, iSensor, t, ton, delta_t);
    }
  } else if (psensor->state() && (psensor->curValue() != psensor->displayVal())) {
    psensor->displayVal(psensor->curValue());
    tft.setFont(Arial_14);
    tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
    tft.setCursor(TFT_STATE_X, y_start + TFT_STATE_ROW2_OFFSET_Y);
    tft.setClipRect(TFT_STATE_X, y_start + TFT_STATE_ROW2_OFFSET_Y, tft.width(), Arial_14.line_space);
 
    tft.printf("C:%d R:%d-%d A:%d", psensor->curValue(),
               psensor->minValue(), psensor->maxValue(), psensor->avgValue());
    EraseRestOfTextLine(y_start + TFT_STATE_ROW2_OFFSET_Y, Arial_14);
    tft.setClipRect();
    send_remote_update = true;   // Lets send all on messages

  }
  return send_remote_update;
}

//====================================================================================
// GetTouchPoint - Return true if touched and convert touch point to X and Y values
//====================================================================================
TS_Point g_pt;  // currently global for debug stuff

bool GetTouchPoint(int16_t *px, int16_t *py)
{
  if (!ts.touched()) // only process when touched.  empty is just a timer...
    return false;

  g_pt = ts.getPoint();

  // Scale from ~0->4000 to tft.width using the calibration #'s
#ifdef SCREEN_ORIENTATION_1
  *px = map(g_pt.x, TS_MINX, TS_MAXX, 0, tft.width());
  *py = map(g_pt.y, TS_MINY, TS_MAXY, 0, tft.height());
#else
  *px = map(g_pt.x, TS_MINX, TS_MAXX, tft.width(), 0);
  *py = map(g_pt.y, TS_MAXY, TS_MINY, 0, tft.height());
#endif
  return true;

}

//====================================================================================
// SetFullTFTBacklight - Turn to max brightness... May change later...
//====================================================================================
void SetFullTFTBacklight() 
{
  if (g_backlight_value != TFT_DIM_MIN) {
    g_backlight_value = TFT_DIM_MIN;
    analogWrite(TFT_BACKLIGHT, g_backlight_value);
  }
  g_tft_backlight_timer = millis(); // remember last time touched.
}


//====================================================================================
// ProcessTouchScreen - Process input from Touch Screen.
//====================================================================================
bool ProcessTouchScreen()
{
  if (!ts.touched()) { // only process when touched.  empty is just a timer...
    if ((g_backlight_value < TFT_DIM_MAX) && (millis() - g_tft_backlight_timer) >= TFT_DIM_INTERVAL_MILLIS) {
      g_backlight_value += TFT_DIM_AMOUNT;
      if ( g_backlight_value > TFT_DIM_MAX) g_backlight_value = TFT_DIM_MAX;
      analogWrite(TFT_BACKLIGHT, g_backlight_value);
      g_tft_backlight_timer = millis();
    }
    return false;
  }

  SetFullTFTBacklight();  // Make sure it is on...
  
  int16_t x, y, x_prev = 0xffff, y_prev = 0xffff;
  while (GetTouchPoint(&x, &y)) {
    if ((x != x_prev) || (y != y_prev)) {
      x_prev = x;
      y_prev = y;
      if (g_debug_output)Serial.printf("PTS Raw: %d, %d Out: %d, %d\n", g_pt.x, g_pt.y, x, y);
    }
  }
  g_tft_backlight_timer = millis(); // remember last time touched.
  return true;
}

