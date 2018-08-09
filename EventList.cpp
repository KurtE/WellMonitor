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


static const char *event_names[]={"ST","TC", "ON", "OF"};
#define CNT_EVENT_NAMES (sizeof(event_names)/sizeof(event_names[0]))

SensorSummary EventList::sensors[CurrentSensor::COUNT_SENSORS];

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

//====================================================================================
// restoreSensorData - See if there is data stored out on the SD Card, if so, lets
//			see if we can restore the display data... 
//====================================================================================
void EventList::restoreSensorData() {
	if (!g_sd_detected) return;
	File datafile = SD.open("well_log_data.csv", FILE_READ);
	if (datafile) { 
		int ch;
		int field_number;
		char event_name[3];
		uint32_t field_values[10];
		uint32_t value = 0;
		ETYPE event;

		// Lets process the events data and get some summary data for each sensor
		time_t 	last_time = 0;

		// clear out sensor data. 
		for (int i = 0; i < CurrentSensor::COUNT_SENSORS; i++) {
			sensors[i].state = 0;		// what the last state was
			sensors[i].last_start_time = 0;
			sensors[i].last_stop_time = 0;

			sensors[i].cnt_cycles_today = 0;
			sensors[i].delta_time_today = 0;
			sensors[i].cnt_cycles_yesterday = 0;
			sensors[i].delta_time_yesterday = 0;
		} 

		while (datafile.available() > 2) {
			// We assume first two CHARS give us type of field
			event = ETYPE::UNKNOWN;
			datafile.read(event_name, 3);
			for (uint8_t i=0; i < CNT_EVENT_NAMES; i++) {
				const char *psz = event_names[i];
				if ((event_name[0] == psz[0]) && (event_name[1] == psz[1])) {
					event = (ETYPE)i;
					break;
				}
			}
			// This pass only deal with sensor on and off...
			if ((event == ETYPE::SENSOR_ON)  || (event == ETYPE::SENSOR_OFF)) {
				field_number = 0;
				value = 0;
				while (((ch = datafile.read()) != -1)) {
					if ((ch == ',' ) || (ch == '\r')) {
						field_values[field_number++] = value;
						value = 0;
						if (ch == '\r') break;
					}
					else if ((ch >= '0') && (ch <= '9')) {
						value = value * 10 + ch - '0';
					}
				}
				// Ok we have hopefully completed parsing the line, now extract the data... 
				uint8_t iSensor = field_values[0];	
				last_time = field_values[1];	// remember the last time mentioned in file
				if (event == ETYPE::SENSOR_ON) {
					sensors[iSensor].state = SENSOR_STATE_ON;
					sensors[iSensor].last_start_time = field_values[1];	// start time
				} else {
					// must be SENSOR OFF
					addEventToSummary(iSensor, field_values[2], field_values[1]);
				}

			} 
			// Now lets read through the end of line...
			while (((ch = datafile.read()) != -1) && (ch != '\n')) {
				; // nothin
			}
		}
		datafile.close();

		// loop through finalizing any data we have... 
		Serial.println("Restore SD Sensor data");
		for (int iSensor = 0; iSensor < CurrentSensor::COUNT_SENSORS; iSensor++) {
			if (sensors[iSensor].state == SENSOR_STATE_ON) {
				addEventToSummary(iSensor, sensors[iSensor].last_start_time, last_time);
			}
			time_t t = sensors[iSensor].last_start_time;
			Serial.printf("  Start: %d/%d/%d %d:%02d", month(t), day(t), year(t) % 100, hour(t), minute(t));
			
			t = sensors[iSensor].last_stop_time;
			Serial.printf("Stop: %d/%d/%d %d:%02d", month(t), day(t), year(t) % 100, hour(t), minute(t));
			Serial.printf(" Today: %d %d:%02d:%02d Yesterday %d %d:%02d:%02d\n", 
				sensors[iSensor].cnt_cycles_today, hour(sensors[iSensor].delta_time_today), 
				minute(sensors[iSensor].delta_time_today),second(sensors[iSensor].delta_time_today),
				sensors[iSensor].cnt_cycles_yesterday, hour(sensors[iSensor].delta_time_yesterday), 
				minute(sensors[iSensor].delta_time_yesterday),second(sensors[iSensor].delta_time_yesterday));
		} 

	}
}



//====================================================================================
// addEventToSummary: add data to summary for how many times pump ran...
//====================================================================================
void EventList::addEventToSummary(uint8_t iSensor, time_t start_time, time_t stop_time) {
	time_t todays_midnight = CurrentSensor::todaysStartTime();
	time_t yesterdays_midnight = todays_midnight - SECS_PER_DAY;

	SensorSummary *psensor = &sensors[iSensor];
	psensor->state = SENSOR_STATE_OFF;
	psensor->last_stop_time = stop_time;	
	psensor->last_start_time = start_time;

	if (start_time >= todays_midnight) {
		psensor->cnt_cycles_today++;
		psensor->delta_time_today += stop_time - start_time;
	} else {
		if (stop_time >= todays_midnight) { 
			psensor->cnt_cycles_today++;
			psensor->delta_time_today += stop_time - todays_midnight;  // Was on at midnight so how long after?
		}
		if (start_time >= yesterdays_midnight) {
			psensor->cnt_cycles_yesterday++;
			if  (stop_time < todays_midnight) psensor->delta_time_yesterday += stop_time - start_time;
			else  psensor->delta_time_yesterday += todays_midnight - start_time;
		} else if (stop_time >= yesterdays_midnight) {
			psensor->cnt_cycles_yesterday++;
			if  (stop_time < todays_midnight) psensor->delta_time_yesterday += stop_time - yesterdays_midnight;
			else  psensor->delta_time_yesterday += SECS_PER_DAY;
		}
	}

}

