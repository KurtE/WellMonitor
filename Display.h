//====================================================================================
// Display header file.  
//====================================================================================
#ifndef __Display_h__
#define __Display_h__

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

#define TFT_STATE_X 115
#define TFT_STATE_OFFSET_Y 4
#define TFT_STATE_ROW2_OFFSET_Y 24
//====================================================================================
// Screen layout and functions  
//====================================================================================
extern void InitTFTDisplay(void);
extern void UpdateDisplayTempHumidy(void);
extern void UpdateDisplayDateTime();
extern void UpdateSensorData(uint8_t iSensor, SensorUpdateState update_state);


#endif


