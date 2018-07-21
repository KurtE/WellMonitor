//====================================================================================
// Header file for my Current Sensor object
//====================================================================================

#ifndef __Remote_Data_h__
#define __Remote_Data_h__
//====================================================================================
// Includes
//====================================================================================
#include <Arduino.h>

//====================================================================================
// Defines and Enums
//====================================================================================
// MISO 5, MOSI 21 SCK 20
#define RF95_FREQ 915.0
#define RF95_BUFFER_SIZE 64

#define WM_MASTER_NODE 42
#define WM_SLAVE_NODE 200

#define SEND_MAX_TIME_BETWEEN_SENDS 50000 // every 5 seconds
#define RECV_MAX_TIME_BEFORE

enum {
  WM_MSG_ID_SENSOR_DATA,
	WM_MSG_ID_ACK  // Send an ACK to the other side...
};

//====================================================================================
// Exported functions  
//====================================================================================
extern void InitRemoteRadio();
extern uint32_t ProcessRemoteMessages();
extern void UpdateSystemTimeWithRemoteTime(time_t t);  // in Main INO file
extern void sendRemoteState();

#endif
