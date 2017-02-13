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
#include <Adafruit_SHT31.h>
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

#pragma pack(pop)

//====================================================================================
// globals
//====================================================================================
RH_RF95 rf95(RFM95_CS, RFM95_INT, hardware_spi1);

// Class to manage message delivery and receipt, using the driver declared above
RHDatagram manager(rf95, WM_MASTER_NODE);

// data kept about messages and the like:
uint8_t g_other_radio_id = 0xff;		// assumming only other radio fro now
elapsedMillis time_since_last_msg_received;		// how much time since last message was received
elapsedMillis time_since_last_msg_sent;		// how much time since last message was received

//====================================================================================
// InitRemoteRadio
//====================================================================================
void InitRemoteRadio()
{
  // Init the radio/Manager on the board - probably move to own function later
  // First init the SPI to use the alternate pin numbers
  SPI1.setMISO(RFM95_MISO);
  SPI1.setMOSI(RFM95_MOSI);
  SPI1.setSCK(RFM95_SCK);

  // Our address depends on if Master or not
  if (!g_master_node) {
  	manager.setThisAddress(WM_SLAVE_NODE );
  }
  if (!manager.init()) {
    Serial.println("RF95: init failed");
  } else {
    if (!rf95.setFrequency(RF95_FREQ)) {
      Serial.println("RF95: setFrequency failed");
    } else
      rf95.setTxPower(23, false);
  }

}

//====================================================================================
// ProcessPinMessage
//====================================================================================
void ProcessPingMsg(WM_PING_MSG *ppmsg) {
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
			if (g_sht31_detected) {  // If we have temp and like might as well send it as well. 
				SendRemoteTempHumidityMsg();
			} else {
				SendRemotePing(false);
			}
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
		if (g_sht31_detected) {  // If we have temp and like might as well send it as well. 
			SendRemoteTempHumidityMsg();
		} else {
			SendRemotePing(false);
		}
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
	msg.message_type = startup? WM_MSG_ID_STARTUP : WM_MSG_ID_PING;
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

		manager.sendto((uint8_t *)&msg, sizeof(msg), g_other_radio_id); 	
		Serial.printf("SRSD: %d %d %d\n", g_other_radio_id, sensor_index, msg.state);
		time_since_last_msg_sent = 0;
	}
}
