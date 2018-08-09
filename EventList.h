//====================================================================================
// Header file for managing events
//====================================================================================

#ifndef __event_list_h__
#define __event_list_h__
//====================================================================================
// Includes
//====================================================================================
#include <Arduino.h>

//====================================================================================
// Defines and Enums
//====================================================================================

//====================================================================================
// Class definition 
//====================================================================================
typedef struct _sensor_summary {
	uint8_t	state;		// what the last state was
	time_t	last_start_time;
	time_t	last_stop_time;

	uint8_t	cnt_cycles_today;
	time_t	delta_time_today;
	uint8_t	cnt_cycles_yesterday;
	time_t	delta_time_yesterday;
} SensorSummary;
class EventList {
public: 
	typedef enum {STARTUP, TIME_CHANGE, SENSOR_ON, SENSOR_OFF, UNKNOWN} ETYPE;
	static bool LogEvent(ETYPE event, int sensor, time_t time1, time_t time2, uint32_t extra_data);

	static void restoreSensorData();

	static SensorSummary sensors[4];
private:
	static void addEventToSummary(uint8_t iSensor, time_t start_time, time_t stop_time);
};


#endif
