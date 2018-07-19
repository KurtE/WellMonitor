//====================================================================================
// Functions to do the remote communications between main display with sensors
// and a remote slve device.
//====================================================================================
//====================================================================================
// Include files
//====================================================================================
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
#include <elapsedMillis.h>
#include <TimeLib.h>

#include <SPI.h>

#include <RHDatagram.h>
#include <RH_RF95.h>
#include <RHHardwareSPI1.h>


//====================================================================================
// Structures
//====================================================================================
#pragma pack(push, 1)
typedef struct  _wm_ping_msg {
  uint8_t message_type;
  time_t time;
} WM_PING_MSG;

typedef struct _wm_temp_hum_msg {
  uint8_t 	message_type;
  time_t 		time;
  uint16_t 	temp;
  uint16_t 	humidity;
} WM_TEMP_HUM_MSG;


typedef struct _wm_sensor_msg {
  uint8_t 	message_type;
  time_t 		time;
  uint8_t 	sensor_index;
  uint8_t   	state;
  uint8_t 	curValue;
  time_t    	onTime;
  time_t    	offTime;
  uint16_t  	minValue;
  uint16_t  	maxValue;
  uint16_t  	avgValue;
} WM_SENSOR_MSG;

typedef struct  _wm_state_msg {
  time_t     time;          // current time
  uint8_t    states[4];     // current state of each sensor
  time_t     onTimes[4];    // last on time for sensor
  time_t     offTimes[4];   // last off time for sensor
  uint16_t   curValues[4];  // current values
  uint16_t   minValues[4];  // minimum values
  uint16_t   maxValues[4];  // Max values
  uint16_t   avgValues[4];  // Avg values
} WM_STATE_MSG; // state message

#pragma pack(pop)

//====================================================================================
// globals
//====================================================================================
RH_RF95 rf95(RFM95_CS, RFM95_INT, hardware_spi1);

// Class to manage message delivery and receipt, using the driver declared above
//RHDatagram manager(rf95, WM_MASTER_NODE);

// data kept about messages and the like:
uint8_t g_other_radio_id = 0xff;		// assumming only other radio fro now
elapsedMillis time_since_last_msg_received;		// how much time since last message was received
elapsedMillis time_since_last_msg_sent;       // how much time since last message was sent

//====================================================================================
// InitRemoteRadio
//====================================================================================
void InitRemoteRadio()
{
  // Init the radio/Manager on the board - probably move to own function later
  // First init the SPI to use the alternate pin numbers
  Serial.println("Init Remote Radio");
  SPI1.setMISO(RFM95_MISO);
  SPI1.setMOSI(RFM95_MOSI);
  SPI1.setSCK(RFM95_SCK);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // Make sure it is reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);

#ifdef USE_RADIOHEAD_DATAGRAM
  // Our address depends on if Master or not
  if (!g_master_node) {
    Serial.printf(" setThisAddress %x\n", WM_SLAVE_NODE);
    manager.setThisAddress(WM_SLAVE_NODE );
  }
  if (!manager.init()) {
    Serial.println("RF95: init failed");
  } else {
    if (!rf95.setFrequency(RF95_FREQ)) {
      Serial.println("RF95: setFrequency failed");
    } else
      rf95.setTxPower(23, false);
    Serial.println("Completed InitRadio\n");

  }
#else
  // Try keeping it simple
  if (!rf95.init()) {
    Serial.println("LoRa radio init failed");
  } else {
    Serial.println("LoRa radio init OK!");

    // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
    if (!rf95.setFrequency(RF95_FREQ)) {
      Serial.println("setFrequency failed");
    } else {
      //rf95.setTxPower(13, false);
    }
    
    //rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096); // slow and reliable?

    Serial.println("Completed InitRadio\n");
  }
#endif
  rf95.printRegisters();
  //  Serial.printf("Our Address: %x\n", manager.thisAddress());

}
//----------------------------------------------
#ifdef USE_RADIOHEAD_DATAGRAM
//====================================================================================
// ProcessPingMessage
//====================================================================================
void ProcessPingMsg(WM_PING_MSG *ppmsg) {
  Serial.println("ProcessPingMsg");
  // Lets see if the time sent to us is > than our time... If so maybe update our time
  time_t tnow = now();
  if ((timeStatus() == timeNotSet) || (tnow < ppmsg->time)) {
    Teensy3Clock.set(ppmsg->time);
  }
  // If this is the first message from one side then may want to send message back
  // to other side with data or asking for data...
  if (ppmsg->message_type == WM_MSG_ID_STARTUP) {
    if (g_master_node) {
      // Let the other side know our status.
#ifdef ENABLE_SHT31
      if (g_sht31_detected) {  // If we have temp and like might as well send it as well.
        SendRemoteTempHumidityMsg();
      } else {
        SendRemotePing(false);
      }
#else
      SendRemotePing(false);
#endif
    } else {
      SendRemotePing(true);	// tell master
    }
  }
}
//====================================================================================
// ProcessPinMessage
//====================================================================================
void ProcessTempHumMsg(WM_TEMP_HUM_MSG *ppmsg) {
  // If we receive these messages and our sensor does not have the temp sensor
  // Than update the display.
  time_t tnow = now();
  if ((timeStatus() == timeNotSet) || (tnow < ppmsg->time)) {
    Teensy3Clock.set(ppmsg->time);
  }

  UpdateTempHumidity(ppmsg->temp, ppmsg->humidity, false);
}

//====================================================================================
// ProcessSensorMsg
//====================================================================================
uint32_t ProcessSensorMsg(WM_SENSOR_MSG *pmsg)
{
  uint8_t sensor_index = pmsg->sensor_index;
  uint32_t retval = 0;
  Serial.printf("PRSM %d %d %d\n", sensor_index, pmsg->state, pmsg->curValue);
  CurrentSensor *psensor = g_Sensors[sensor_index];
  if (pmsg->state != psensor->state()) {
    psensor->state(pmsg->state);
    if (pmsg->state == SENSOR_STATE_ON) {
      retval = SENSOR_UPDATE_ON_DETECTED;
      psensor->curValue(pmsg->curValue);
    }
    else if (pmsg->state == SENSOR_STATE_OFF)
      psensor->curValue(pmsg->curValue);
    retval = SENSOR_UPDATE_ON_DETECTED;
    Serial.printf("PRSM NS: %d %d\n", psensor->state(), psensor->curValue());
  } else if (pmsg->curValue != psensor->curValue()) {
    psensor->state(pmsg->state);
    retval = SENSOR_UPDATE_DONE_NEW_VALUE;
  }
  psensor->curValue(pmsg->curValue);
  psensor->onTime(pmsg->onTime);
  psensor->offTime(pmsg->offTime);
  psensor->minValue(pmsg->minValue);
  psensor->maxValue(pmsg->maxValue);
  psensor->avgValue(pmsg->avgValue);
  return retval;
}

//====================================================================================
// ProcessRemoteMessages
//====================================================================================
uint32_t ProcessRemoteMessages()
{
  uint8_t buff[RF95_BUFFER_SIZE];
  uint8_t len;
  uint8_t to;
  uint8_t id;
  uint32_t retval = 0;

  if (manager.available()) {
    len = sizeof(buff);
    if (manager.recvfrom (buff, &len, &g_other_radio_id, &to, &id)) {

      // Change the LED state to show we received a message
      digitalWrite(13, !digitalRead(13));
      time_since_last_msg_received = 0;	// zero out timer of how long since last message.
      Serial.printf("CRM: (%x) l:%u f:%u t:%u i:%u\n", buff[0], len, g_other_radio_id, to, id);

      switch (buff[0]) {
        case WM_MSG_ID_STARTUP:
        case WM_MSG_ID_PING:		// Ping message - with our RTC time
          ProcessPingMsg((WM_PING_MSG*)buff);
          break;

        case WM_MSG_ID_TEMP_HUM:	// Temp/Humidity messages
          ProcessTempHumMsg((WM_TEMP_HUM_MSG*)buff);
          break;

        case WM_MSG_ID_SENSOR_DATA:
          retval |= ProcessSensorMsg((WM_SENSOR_MSG*)buff);
          break;
      }
    }
  }
  if (time_since_last_msg_sent > SEND_MAX_TIME_BETWEEN_SENDS) {
#ifdef ENABLE_SHT31
    if (g_sht31_detected) {  // If we have temp and like might as well send it as well.
      SendRemoteTempHumidityMsg();
    } else {
      SendRemotePing(false);
    }
#else
    SendRemotePing(false);
#endif
    time_since_last_msg_sent = 0;
  }
  return retval;
}

//====================================================================================
// SendRemoteTempHumidityMsg
//====================================================================================
void  SendRemoteTempHumidityMsg() {
  WM_TEMP_HUM_MSG msg;
  if (g_other_radio_id != 0xff) {
    Serial.printf("SRTHM: %d\n", g_other_radio_id);
    msg.message_type = WM_MSG_ID_TEMP_HUM;
    msg.time = now();
    msg.temp = g_current_temp;
    msg.humidity = g_current_humidity;
    manager.sendto((uint8_t *)&msg, sizeof(msg), g_other_radio_id);
    time_since_last_msg_sent = 0;
  }
}


//====================================================================================
// SendRemotePing
//====================================================================================
extern void	SendRemotePing(boolean startup) {
  WM_PING_MSG msg;
  if (!g_master_node) {
    if (g_other_radio_id == 0xff) g_other_radio_id = WM_MASTER_NODE; 	// set client to master mode
  }
  Serial.printf("SRPING: %d %d\n", g_other_radio_id, startup);
  msg.message_type = startup ? WM_MSG_ID_STARTUP : WM_MSG_ID_PING;
  msg.time = now();
  manager.sendto((uint8_t *)&msg, sizeof(msg), g_other_radio_id);
  time_since_last_msg_sent = 0;

}


//====================================================================================
// SendRemoteSensorData
//====================================================================================
extern void SendRemoteSensorData(uint8_t sensor_index) {
  if (g_other_radio_id != 0xff) {
    CurrentSensor *psensor = g_Sensors[sensor_index];
    WM_SENSOR_MSG msg;
    msg.message_type = WM_MSG_ID_SENSOR_DATA;
    msg.time = now();
    msg.sensor_index = sensor_index;
    msg.state = psensor->state();
    msg.curValue =  psensor->curValue();
    msg.onTime = psensor->onTime();
    msg.offTime = psensor->offTime();
    msg.minValue = psensor->minValue();
    msg.maxValue = psensor->maxValue();
    msg.avgValue = psensor->avgValue();

    digitalWriteFast(9, HIGH);
    manager.sendto((uint8_t *)&msg, sizeof(msg), g_other_radio_id);
    digitalWriteFast(9, LOW);
    Serial.printf("SRSD: %d %d %d\n", g_other_radio_id, sensor_index, msg.state);
    time_since_last_msg_sent = 0;
  }
}
#else
///////////////////////////////////////////////////////////////////////////////////
// Simple handling...
///////////////////////////////////////////////////////////////////////////////////
elapsedMillis time_since_last_report;
#define TIME_BETWEEN_REPORTS_MS 5000    // start of every 5 seconds...
void sendRemoteState() {
  if (!g_master_node) return;   // only send if we are master
  if (time_since_last_msg_sent < TIME_BETWEEN_REPORTS_MS) return;
  time_since_last_msg_sent = 0;

  WM_STATE_MSG msg;
  msg.time = now();
  for (int sensor_index = 0; sensor_index < 4; sensor_index++) {
    msg.states[sensor_index] = g_Sensors[sensor_index]->state();
    msg.curValues[sensor_index] =  g_Sensors[sensor_index]->curValue();
    msg.onTimes[sensor_index] = g_Sensors[sensor_index]->onTime();
    msg.offTimes[sensor_index] = g_Sensors[sensor_index]->offTime();
    msg.minValues[sensor_index] = g_Sensors[sensor_index]->minValue();
    msg.maxValues[sensor_index] = g_Sensors[sensor_index]->maxValue();
    msg.avgValues[sensor_index] = g_Sensors[sensor_index]->avgValue();
  }
  digitalWriteFast(9, HIGH);
  uint32_t stime = millis();
  bool avail = rf95.available();
  bool sent = rf95.send((uint8_t *)&msg, sizeof(msg));
  bool wait_completed = rf95.waitPacketSent(1000);   // wait up to 1/4 second? 
  digitalWriteFast(9, LOW);
  if (Serial) {
    Serial.printf("SRSD(%d %d %d %d): %d %d %d %d\n", avail, sent, wait_completed, millis()-stime, msg.states[0], msg.states[1], msg.states[2],
                msg.states[3]);
  }
}
//====================================================================================
// ProcessRemoteMessages
//====================================================================================
union {
  WM_STATE_MSG msg;
  uint8_t     buff[RH_RF95_MAX_MESSAGE_LEN];
} msg_buf;

uint32_t ProcessRemoteMessages()
{
  uint8_t len;
  uint32_t retval = 0;

  if (g_master_node) return 0;   // Right now only do if not master

  if (rf95.available()) {
    digitalWrite(13, !digitalRead(13));
    len = sizeof(msg_buf);
    if (rf95.recv ((uint8_t*)&msg_buf, &len)) {
      time_since_last_msg_received = 0;  // zero out timer of how long since last message.
      if (Serial)
        Serial.printf("CRM: l: % u\n", len);

      // Lets see if we have any new updated data...
      for (int sensor_index = 0; sensor_index < 4; sensor_index++) {
        if (Serial)
          Serial.printf("PRSM % d % d % d\n", sensor_index, msg_buf.msg.states[sensor_index], msg_buf.msg.curValues[sensor_index]);
        if (msg_buf.msg.states[sensor_index] != g_Sensors[sensor_index]->state()) {
          g_Sensors[sensor_index]->state(msg_buf.msg.states[sensor_index]);
          retval = 1;
        }
        if (msg_buf.msg.curValues[sensor_index] != g_Sensors[sensor_index]->curValue()) {
          g_Sensors[sensor_index]->curValue(msg_buf.msg.curValues[sensor_index]);
          retval = 1;
        }
        if (msg_buf.msg.onTimes[sensor_index] != g_Sensors[sensor_index]->onTime()) {
          g_Sensors[sensor_index]->onTime(msg_buf.msg.onTimes[sensor_index]);
          retval = 1;
        }
        if (msg_buf.msg.offTimes[sensor_index] != g_Sensors[sensor_index]->offTime()) {
          g_Sensors[sensor_index]->offTime(msg_buf.msg.offTimes[sensor_index]);
          retval = 1;
        }
        // Assume these won't change without one of the other items changing...
        g_Sensors[sensor_index]->minValue(msg_buf.msg.minValues[sensor_index]);
        g_Sensors[sensor_index]->maxValue(msg_buf.msg.maxValues[sensor_index]);
        g_Sensors[sensor_index]->avgValue(msg_buf.msg.avgValues[sensor_index]);
      }
    } 
  }
  return retval;
}
#endif
