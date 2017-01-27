//====================================================================================
// Put all of the global defines and configiration settings here.  
//====================================================================================
#ifndef __globals_h__
#define __globals_h__
#include <Wire.h>
#include <Adafruit_SHT31.h>

//====================================================================================
// Pin Numbers. 
//====================================================================================
#define TFT_RST -1
#define RFM95_INT     2     // RFM 95 Interrupt pin
#define RFM95_MISO    5     // RFM 95 MISO
#define TFT_MOSI      7     // SPI MOSI
#define TOUCH_CS      8     // SPI Touch CS
#define TFT_MISO      12    // SPI MISO
#define TFT_SCK       14    // SPI SCK
#define TFT_CS        15    // TFT CS
#define NEOPIXEL_PIN  17    // NeoPixel pin
#define RFM95_SCK     20    // SPI 1 SCK
#define RFM95_MOSI    21    // SPI 1 MOSI used for RFM95
#define TFT_DC        22    // TFT DC
#define MASTER_SLAVE  24    // If low slave if high master...
#define RFM95_CS      31    // RFM95 CS
#define RFM95_RST     37    // RFM95 Reset pin

#define  SENSOR1_PIN  A13   // Current sensor 1 pin 
#define  SENSOR2_PIN  A14   // Current sensor 2 pin 
#define  SENSOR3_PIN  A15   // Current sensor 3 pin 
#define  SENSOR4_PIN  A16   // Current sensor 4 pin 

//====================================================================================
// EEPRM - offsets  
//====================================================================================
#define EEPROM_SENSOR_VERSION 1
#define EEPROM_SENSOR_INFO 0


//====================================================================================
// Touch screen calibrations.  
//====================================================================================
// This is calibration data for the raw touch data to the screen coordinates
// Warning, These are WIP
#define TS_MINX 337
#define TS_MINY 529
#define TS_MAXX 3729
#define TS_MAXY 3711

//====================================================================================
// Current g_Sensors.   
//====================================================================================
#define MIN_CURRENT_ON      5   // Min value to report. 

//====================================================================================
// Global variables
//====================================================================================
extern Adafruit_SHT31 sht31;
extern bool sht31_detected; 	// Does this unit have an sht31? 
extern uint8_t master_node;

#endif
