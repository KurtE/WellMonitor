/*************************************************** 
  This is a library for the SHT31 Digital Humidity & Temp Sensor

  Designed specifically to work with the SHT31 Digital sensor from Adafruit
  ----> https://www.adafruit.com/products/2857

  These displays use I2C to communicate, 2 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/
#ifndef _SHT31_H
#define _SHT31_H

#include "Arduino.h"
#define USE_I2C_T3
#ifdef USE_I2C_T3
#include "i2c_t3.h"
#else
#include "Wire.h"
#define I2C_STOP 1
#endif

#define SHT31_DEFAULT_ADDR    0x44
#define SHT31_MEAS_HIGHREP_STRETCH 0x2C06
#define SHT31_MEAS_MEDREP_STRETCH  0x2C0D
#define SHT31_MEAS_LOWREP_STRETCH  0x2C10
#define SHT31_MEAS_HIGHREP         0x2400
#define SHT31_MEAS_MEDREP          0x240B
#define SHT31_MEAS_LOWREP          0x2416
#define SHT31_READSTATUS           0xF32D
#define SHT31_CLEARSTATUS          0x3041
#define SHT31_SOFTRESET            0x30A2
#define SHT31_HEATEREN             0x306D
#define SHT31_HEATERDIS            0x3066

class Adafruit_SHT31 {
 public:
  Adafruit_SHT31();
  boolean begin(uint8_t i2caddr = SHT31_DEFAULT_ADDR);
  float readTemperature(void);
  float readHumidity(void);
  float temperature(void) { return _temp; } 
  float humidity(void) { return _humidity; }
  uint16_t readStatus(void);
  void reset(void);
  void heater(boolean);
  uint8_t crc8(const uint8_t *data, int len);

  // Setup to be able to read in two parts 
  // as to not have to delay processor for half second...
  boolean beginReadTempHum(void);
  boolean completeReadTempHum(void);

 private:
  boolean readTempHum(void);
  boolean writeCommand(uint16_t cmd);

  uint8_t _i2caddr;
  boolean readData(void);
  float _humidity, _temp;
};

#endif
