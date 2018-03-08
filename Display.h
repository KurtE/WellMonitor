//====================================================================================
// Display header file.  
//====================================================================================
#ifndef __Display_h__
#define __Display_h__
#include <ILI9341_t3n.h>
#include <XPT2046_Touchscreen.h>

//====================================================================================
// Includes
//====================================================================================
#include "CurrentSensor.h"

//====================================================================================
// Defines
//====================================================================================
#define TFT_TIMETEMP_Y 2
#define TFT_TIME_X 2
#define TFT_TEMP_HEADER_X 150
#define TFT_TEMP_X 210
#define TFT_HUMIDITY_HEADER_X 240
#define TFT_HUMIDITY_X 290

#define TFT_TITLES_X 2
#define TFT_WELL1_Y 30
#define TFT_WELL2_Y 80
#define TFT_PRESURE_Y 130
#define TFT_HEATER_Y 180

#define TFT_STATE_X 95
#define TFT_STATE_OFFSET_Y 4
#define TFT_STATE_ROW2_OFFSET_Y 24

#define UPDATE_TIME_TEMP_MILLIS 500

// Bounds for touch . 


//====================================================================================
// Global data.  Probably clean up
//====================================================================================
extern uint16_t g_current_temp;
extern uint16_t g_current_humidity;
extern ILI9341_t3n tft;
extern XPT2046_Touchscreen ts;


//====================================================================================
// Screen layout and functions  
//====================================================================================
extern void InitTFTDisplay(void);
extern void ReadTempHumiditySensor(void);
extern void UpdateDisplayDateTime();
extern bool UpdateTempHumidity(uint16_t temp, uint16_t humidity, bool local_data);
extern bool UpdateDisplaySensorData(uint8_t iSensor);
extern bool ProcessTouchScreen();
extern bool GetTouchPoint(int16_t *px, int16_t *py);

//====================================================================================
// Define our display objects. 
//====================================================================================
class DISPOBJ {
public:
  inline void    Enable(boolean fEnabled);

  virtual void Draw(void)=0;
  virtual int16_t ProcessTouch(int16_t x, int16_t y)=0;
  friend class OLEDSGC;
protected:
  DISPOBJ *pdoNext;        // Next display object
  int16_t    _x;             //TL
  int16_t    _y;    
  int16_t    _w;            
  int16_t    _h;
boolean _fEnabled : 
  1;
boolean _fPressed : 
  1;
boolean _fTouchActive : 
  1;
};

class BUTTON  : 
public DISPOBJ {
public:
  void Init (int16_t x, int16_t y, int16_t dx, int16_t dy, int16_t wBtnClr, int16_t wHiClr, int16_t wTextClr, const char *psz, int16_t wVal);
  virtual void Draw(void);
  virtual int16_t ProcessTouch(int16_t x, int16_t y);

  // some button specific functions
  inline void SetPressed(boolean fPressed)  {
    _fPressed = fPressed;
  };        
private:    
  int16_t _wBtnClr;
  int16_t _wHiClr;
  int16_t _wTextClr;
  const char *_psz;
  int16_t  _wVal;

  //        boolean _fPressed;
};

class SLIDER  : 
public DISPOBJ {
public:
  void Init (int16_t x, int16_t y, int16_t dx, int16_t dy, int16_t wClr, int16_t wHiClr, int16_t wBackClr, int16_t wMin, int16_t wMax, int16_t wVal);
  virtual void Draw(void);
  virtual int16_t ProcessTouch(int16_t x, int16_t y);

  // some button specific functions
  inline int16_t GetValue(void) {
    return _wVal;
  };
  void SetValue(int16_t wVal);

  inline void SetPressed(boolean fPressed)  {
    _fPressed = fPressed;
  };        
private:    
  inline void MapValToDispVal(void);
  inline int16_t MapDispValToVal(int16_t wXorY);

  int16_t _wClr;
  int16_t _wHiClr;
  int16_t _wBackClr;
  boolean _fPressed;
  boolean _fVert;        // is the slider vertical or horizontal?

  int16_t _wMin;
  int16_t _wMax;
  int16_t _wVal;

  int16_t _wDispVal;        // This is the split X or Y value of what the current value maps to..
};




#endif


