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


//====================================================================================
// globals
//====================================================================================
//#############################################################################################################
bool g_fDebugOutput = false;
//=============================================================================================================
//  Display Object Implementation.
//=============================================================================================================
inline void  DISPOBJ::Enable(boolean fEnabled) {
  _fEnabled = fEnabled;
}


//=============================================================================================================
//  Display Object: Button
//=============================================================================================================
void BUTTON::Init(int16_t x, int16_t y, int16_t dx, int16_t dy, int16_t wBtnClr, int16_t wHiClr, int16_t wTextClr, const char *psz, int16_t wVal)  {
  // Save away all of the data...
  // first the base objects stuff...
  _x = x;
  _w = dx;
  _y = y;
  _h = dy;
  _fEnabled = true;        // default to enabled...

  _wBtnClr = wBtnClr;
  _wHiClr = wHiClr;
  _wTextClr = _wTextClr;
  _psz = psz;
  _wVal = wVal;
  _fPressed = false;
}


void BUTTON::Draw() {

  // BUGBUG:: Hard coded font size... 10x16
  int16_t wXText = _x + _w / 2 - strlen(_psz) * 10 / 2; // Could precalc and save also verify that text fits...
  int16_t wYText = _y + _h / 2 - 16 / 2;
#if 0
  Serial.print("Btn: ");
  Serial.write(_psz);
  Serial.write(" ");
  Serial.print(_x, DEC);
  Serial.write(" ");
  Serial.print(_y, DEC);
  Serial.write(" ");
  Serial.print(_w, DEC);
  Serial.write(" ");
  Serial.print(_h, DEC);
  Serial.write(" ");
  Serial.print(wXText, DEC);
  Serial.write(" ");
  Serial.println(wYText, DEC);
#endif
  tft.setCursor(wXText, wYText);
  tft.setTextSize(2);
  tft.setTextColor(_wTextClr);

  if (_fPressed) {
    tft.fillRect(_x, _y, _w, _h, _wHiClr);
    tft.drawRect(_x + 2, _y + 2, _w - 4, _h - 4, _wBtnClr);

    // Now output graphic text...
    tft.print(_psz);
  }
  else {
    tft.fillRect(_x, _y, _w, _h, _wBtnClr);
    tft.drawRect(_x + 2, _y + 2, _w - 4, _h - 4, _wHiClr);

    // Now output graphic text...
    tft.print(_psz);
  }
}

// Assumes that we got a button down event, we pass in the X, Y from this event and it will check to see if the coordinates
// are within the object and process it. If the up happens in the same object then we will return the object ID, else we will
//
int16_t BUTTON::ProcessTouch(int16_t x, int16_t y) {

  //int16_t wTouchState;

  if (!((x >= _x) && (x <= (_x + _w)) && (y >= _y) && (y <= (_y + _h))))
    return 0x0;        // Special value that says Not within the object...

  SetPressed(true);
  Draw();    // Show it as depressed.

  // Now lets wait for the touch to release...
  while (ts.touched()) {
    GetTouchPoint(&x, &y);
    if (g_fDebugOutput) {
//      Serial.printf("Touch(%d,%d: (%d, %d)\n", g_last_touch_raw_x, g_last_touch_raw_y, x, y);
    }
  }

  SetPressed(false);
  Draw();

  // And see if we were still inside the button
  if ((x >= _x) && (x <= (_x + _w)) && (y >= _y) && (y <= (_y + _h)))
    return _wVal;

  return 0xffff;    // not on the button any more...
}
//=============================================================================================================
//  Display Object: slider
//=============================================================================================================
void SLIDER::Init (int16_t x, int16_t y, int16_t dx, int16_t dy, int16_t wClr, int16_t wHiClr, int16_t wBackClr, int16_t wMin, int16_t wMax, int16_t wVal) {
  // first the base objects stuff...
  _x = x;
  _w = dx;
  _y = y;
  _h = dy;
  _fEnabled = true;        // default to enabled...
  _fVert = (dy > dx);

  // Now the slider specific fields
  _wClr = wClr;
  _wHiClr = wHiClr;
  _wBackClr = _wBackClr;
  _fPressed = false;

  _wMin = wMin;
  _wMax = wMax;
  _wVal = wVal;
  MapValToDispVal();
}

void SLIDER::MapValToDispVal(void) {
  if (_fVert)
    // In vertical we start to fill from bottom to top...
    _wDispVal = map(_wVal, _wMin, _wMax, _y + _h, _y + 1);
  else
    _wDispVal = map(_wVal, _wMin, _wMax, _x + 1, _x + _w);
}

int16_t SLIDER::MapDispValToVal(int16_t wXorY) {
  if (_fVert)
    return map(wXorY, _y + 1, _y + _h, _wMax, _wMin);

  return map(wXorY, _x + 1, _x + _w, _wMin, _wMax);
}


void SLIDER::Draw(void) {
  // Start off real simple simple filled rect to where the current value is... May also have simple text value...
  //First lets draw the outline frame around our object...
  tft.drawRect(_x, _y, _w, _h, _wClr);
  if (_fVert) {
    // In vertical we start to fill from bottom to top...
    if ( _wDispVal > (_y + 1))
      tft.fillRect(_x + 1, _y + 1, _w - 2,  _wDispVal - _y, _wBackClr);
    if ( _wDispVal < (_y + _h - 1))
      tft.fillRect(_x + 1,  _wDispVal, _w - 2, _h - (_wDispVal - _y), _wHiClr);
  }
  else {
    if ( _wDispVal > (_x + 1))
      tft.fillRect(_x + 1, _y + 1,  _wDispVal - _x, _h - 2, _wHiClr);
    if ( _wDispVal < (_x + _w - 1))
      tft.fillRect( _wDispVal + 1, _y + 1, _w - (_wDispVal - _x), _h - 2, _wBackClr);
  }
}

int16_t SLIDER::ProcessTouch(int16_t x, int16_t y) {
  //int16_t wTouchState;

  if (!((x >= _x) && (x <= (_x + _w)) && (y >= _y) && (y <= (_y + _h))))
    return 0x0;        // Special value that says Not within the object...

  SetPressed(true);
  Draw();    // Show it as depressed.

  // Now lets wait for the touch to release...
  while (ts.touched()) {
    GetTouchPoint(&x, &y);
    if (_fVert) {
      if ((y >= _y) && (y <= (_y + _h))) {
        _wDispVal = y;
        Draw();    // try to redraw....
      }
    }
    else {
      if ((x >= _x) && (x <= (_x + _w))) {
        _wDispVal = x;
        Draw();    // try to redraw....
      }
    }
  }
//  while (tft.FTouchEventsAvail()) {
//    tft.GetTouchPoint(&x, &y);
//  }

  SetPressed(false);

  // And see if we were still inside the button
  if ((x >= _x) && (x < (_x + _w)) && (y >= _y) && (y < (_y + _h))) {
    if (_fVert)
      SetValue(MapDispValToVal(y));
    else
      SetValue(MapDispValToVal(x));
    return 1;
  }

  // Otherwise make sure we are mapped back to the correct value...
  MapValToDispVal();
  Draw();
  return 0xffff;    // not on the button any more...
}


void SLIDER::SetValue(int16_t wVal) {
  if (wVal != _wVal) {
    _wVal = wVal;

    MapValToDispVal();

    Draw();    // redraw the slider
  }
}

