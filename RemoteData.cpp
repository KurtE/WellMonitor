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

typedef struct  _wm_state_msg {
  uint16_t   message_type;  // Message type
  uint16_t   packet_num;    // current packet number
  time_t     time;          // current time
  uint8_t    states[4];     // current state of each sensor
  time_t     onTimes[4];    // last on time for sensor
  time_t     offTimes[4];   // last off time for sensor
  uint16_t   curValues[4];  // current values
  uint16_t   minValues[4];  // minimum values
  uint16_t   maxValues[4];  // Max values
  uint16_t   avgValues[4];  // Avg values
} WM_STATE_MSG; // state message

// Note actually simple header part of state msg above
typedef struct  _wm_ack_msg {
  uint16_t message_type;
  uint16_t   packet_num;    // current packet number
  time_t time;
} WM_ACK_MSG;

#pragma pack(pop)

//====================================================================================
// globals
//====================================================================================
RH_RF95 rf95(RFM95_CS, RFM95_INT, hardware_spi1);

// Class to manage message delivery and receipt, using the driver declared above
RHDatagram manager(rf95, WM_MASTER_NODE);

// data kept about messages and the like:
uint8_t g_other_radio_id = WM_SLAVE_NODE;		  // assume our other radio for now
elapsedMillis time_since_last_msg_received;		// how much time since last message was received
elapsedMillis time_since_last_msg_sent;       // how much time since last message was sent

//====================================================================================
// InitRemoteRadio
//====================================================================================
void InitRemoteRadio()
{
  // Init the radio/Manager on the board - probably move to own function later
  // First init the SPI to use the alternate pin numbers
  if (g_debug_output) Serial.println("Init Remote Radio");
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

  // Our address depends on if Master or not
  if (!g_master_node) {
    if (g_debug_output) Serial.printf(" setThisAddress %x\n", WM_SLAVE_NODE);
    manager.setThisAddress(WM_SLAVE_NODE );
  }
  if (!manager.init()) {
    Serial.println("RF95: init failed");
  } else {
    if (!rf95.setFrequency(RF95_FREQ)) {
      Serial.println("RF95: setFrequency failed");
    } else
      rf95.setTxPower(23, false);
    if (g_debug_output) Serial.println("Completed InitRadio\n");
  }
  if (g_debug_output) {
    rf95.printRegisters();
    Serial.printf("Our Address: %x\n", manager.thisAddress());
  }

}
elapsedMillis time_since_last_report;
uint16_t state_packet_number = 0;
#define TIME_BETWEEN_REPORTS_MS 5000    // start of every 5 seconds...

//====================================================================================
// SendRemoteState
//====================================================================================
void sendRemoteState() {
  if (!g_master_node) return;   // only send if we are master
  if (time_since_last_msg_sent < TIME_BETWEEN_REPORTS_MS) return;
  time_since_last_msg_sent = 0;

  WM_STATE_MSG msg;
  msg.message_type = WM_MSG_ID_SENSOR_DATA;
  msg.packet_num =  state_packet_number++;

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
  bool avail = manager.available();
  bool sent = manager.sendto((uint8_t *)&msg, sizeof(msg), g_other_radio_id);
  bool wait_completed = manager.waitPacketSent(1000);   // wait up to 1/4 second?
  digitalWriteFast(9, LOW);
  if (g_debug_output) {
    Serial.printf("SRSD(%d %d %d %d): %d %d %d %d\n", avail, sent, wait_completed, millis() - stime, msg.states[0], msg.states[1], msg.states[2],
                  msg.states[3]);
  }
}

//====================================================================================
// SendRemoteACK - Send ack after receiving state information...
//====================================================================================
void  SendRemoteAck(uint16_t packet_num) {
  WM_ACK_MSG msg;
  if (!g_master_node) {
    if (g_other_radio_id == 0xff) g_other_radio_id = WM_MASTER_NODE;  // set client to master mode
  }
  msg.message_type = WM_MSG_ID_ACK;
  msg.packet_num =  packet_num;
  msg.time = now();

  bool avail = manager.available();
  bool sent = manager.sendto((uint8_t *)&msg, sizeof(msg), g_other_radio_id);
  bool wait_completed = manager.waitPacketSent(1000);   // wait up to 1/4 second?
  if (g_debug_output) {
    Serial.printf("SRACK(%d %d %d): %d %d\n", avail, sent, wait_completed, g_other_radio_id, packet_num);
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

  //if (g_master_node) return 0;   // Right now only do if not master

  if (manager.available()) {
    digitalWrite(13, !digitalRead(13));
    len = sizeof(msg_buf);
    uint8_t to;
    uint8_t id;
    if (manager.recvfrom ((uint8_t*)&msg_buf, &len, &g_other_radio_id, &to, &id)) {
      time_since_last_msg_received = 0;  // zero out timer of how long since last message.
      if (g_debug_output)
        Serial.printf("PRM: f:%u id:%d l: %u\n", g_other_radio_id, id, len);

      // See if we maybe should update clock?
      UpdateSystemTimeWithRemoteTime(msg_buf.msg.time);

      // Lets see what the message is:
      if (msg_buf.msg.message_type == WM_MSG_ID_SENSOR_DATA) {
        // Lets see if we have any new updated data...
        for (int sensor_index = 0; sensor_index < 4; sensor_index++) {
          if (g_debug_output) {
            Serial.printf("PRSM % d % d % d\n", sensor_index, msg_buf.msg.states[sensor_index], msg_buf.msg.curValues[sensor_index]);
          }
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
            g_Sensors[sensor_index]->displayState(SENSOR_STATE_CHANGED);
            retval = 1;
          }
          if (msg_buf.msg.offTimes[sensor_index] != g_Sensors[sensor_index]->offTime()) {
            g_Sensors[sensor_index]->offTime(msg_buf.msg.offTimes[sensor_index]);
            g_Sensors[sensor_index]->displayState(SENSOR_STATE_CHANGED);
            retval = 1;
          }
          // Assume these won't change without one of the other items changing...
          g_Sensors[sensor_index]->minValue(msg_buf.msg.minValues[sensor_index]);
          g_Sensors[sensor_index]->maxValue(msg_buf.msg.maxValues[sensor_index]);
          g_Sensors[sensor_index]->avgValue(msg_buf.msg.avgValues[sensor_index]);
        }
        SendRemoteAck(msg_buf.msg.packet_num);
      }
    }
  }
  return retval;
}


