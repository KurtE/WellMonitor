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

#define I2C_T3_TIMEOUT 5000

#include "globals.h"
#include "SHT31.h"
#ifdef ENABLE_SHT31

Adafruit_SHT31::Adafruit_SHT31() {
}


boolean Adafruit_SHT31::begin(uint8_t i2caddr) {
  #ifdef SHT31_ALR_PIN
  pinMode(SHT31_ALR_PIN, INPUT);
  #endif
  Wire.begin();
#ifdef USE_I2C_T3
  Wire.setDefaultTimeout(I2C_T3_TIMEOUT);
#endif  
  _i2caddr = i2caddr;
  reset();
  //return (readStatus() == 0x40);
  return true;
}

uint16_t Adafruit_SHT31::readStatus(void) {
  writeCommand(SHT31_READSTATUS);
  Wire.requestFrom(_i2caddr, (uint8_t)3);
  uint16_t stat = Wire.read();
  stat <<= 8;
  stat |= Wire.read();
  //Serial.println(stat, HEX);
  return stat;
}

void Adafruit_SHT31::reset(void) {
  writeCommand(SHT31_SOFTRESET);
  delay(10);
}

void Adafruit_SHT31::heater(boolean h) {
  if (h)
    writeCommand(SHT31_HEATEREN);
  else
    writeCommand(SHT31_HEATERDIS);
}


float Adafruit_SHT31::readTemperature(void) {
  if (! readTempHum()) return NAN;

  return _temp;
}
  

float Adafruit_SHT31::readHumidity(void) {
  if (! readTempHum()) return NAN;

  return _humidity;
}


boolean Adafruit_SHT31::beginReadTempHum(void) {
  return writeCommand(SHT31_MEAS_HIGHREP);
}

boolean Adafruit_SHT31::completeReadTempHum(void) {
  uint8_t readbuffer[6];

#ifdef USE_I2C_T3
  Wire.requestFrom(_i2caddr, (uint8_t)6, I2C_STOP, I2C_T3_TIMEOUT);
#else
  Wire.requestFrom(_i2caddr, (uint8_t)6);
#endif  
  if (Wire.available() != 6) 
    return false;
  for (uint8_t i=0; i<6; i++) {
    readbuffer[i] = Wire.read();
    //Serial.print("0x"); Serial.println(readbuffer[i], HEX);
  }
  uint16_t ST, SRH;
  ST = readbuffer[0];
  ST <<= 8;
  ST |= readbuffer[1];

  if (readbuffer[2] != crc8(readbuffer, 2)) return false;

  SRH = readbuffer[3];
  SRH <<= 8;
  SRH |= readbuffer[4];

  if (readbuffer[5] != crc8(readbuffer+3, 2)) return false;

  //Serial.print("ST = "); Serial.println(ST);
  double stemp = ST;
  stemp *= 315;
  stemp /= 0xffff;
  stemp = -49 + stemp;
  _temp = stemp;
  
  //Serial.print("SRH = "); Serial.println(SRH);
  double shum = SRH;
  shum *= 100;
  shum /= 0xFFFF;
  
  _humidity = shum;
  
  return true;
}


boolean Adafruit_SHT31::readTempHum(void) {
  if (beginReadTempHum()) {  // simply write out the command. 
    delay(500);
    return completeReadTempHum();
  }
  return false;
}

boolean Adafruit_SHT31::writeCommand(uint16_t cmd) {
  Wire.beginTransmission(_i2caddr);
  Wire.write(cmd >> 8);
  Wire.write(cmd & 0xFF);
#ifdef USE_I2C_T3
  uint8_t retval = Wire.endTransmission(I2C_STOP, I2C_T3_TIMEOUT);  
#else
  uint8_t retval = Wire.endTransmission();  
#endif  
  if (retval) {
    //Serial.printf("AWC End Fail try again: %d %d\n", cmd, retval );
    // Try again... 
    retval = Wire.endTransmission();  
    if (retval) {
      //Serial.printf("Failed again: %d %d\n", cmd, retval );
      return false;
    }
  }
  return true;
}

uint8_t Adafruit_SHT31::crc8(const uint8_t *data, int len)
{
/*
*
 * CRC-8 formula from page 14 of SHT spec pdf
 *
 * Test data 0xBE, 0xEF should yield 0x92
 *
 * Initialization data 0xFF
 * Polynomial 0x31 (x8 + x5 +x4 +1)
 * Final XOR 0x00
 */

  const uint8_t POLYNOMIAL(0x31);
  uint8_t crc(0xFF);
  
  for ( int j = len; j; --j ) {
      crc ^= *data++;

      for ( int i = 8; i; --i ) {
  crc = ( crc & 0x80 )
    ? (crc << 1) ^ POLYNOMIAL
    : (crc << 1);
      }
  }
  return crc;
}

/*********************************************************************/
#endif //ENABLE_SHT31

