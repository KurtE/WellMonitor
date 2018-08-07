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
#include <elapsedMillis.h>
#include <TimeLib.h>
#include <SD.h>

	typedef enum {STARTUP, TIME_CHANGE, SENSOR_ON, SENSOR_OFF} ETYPE;

static const char *event_names[]={"ST","TC", "ON", "OF"};

// Exported functions  
//====================================================================================
// LogEvent - add a new event to the log event list and save it out to SD Card...
//====================================================================================
bool EventList::LogEvent(ETYPE event, int sensor, time_t time1, time_t time2, uint32_t extra_data) {
	if (g_sd_detected) {
		File datafile = SD.open("well_log_data.csv", FILE_WRITE);
		if (datafile) {
			datafile.printf("%s,%d,%u,%u,%u\r\n", event_names[(uint8_t)event], sensor, time1, time2, extra_data);	
			datafile.close();
		}
	}
	return true;
}