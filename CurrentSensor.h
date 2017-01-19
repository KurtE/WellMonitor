//====================================================================================
// Header file for my Current Sensor object
//====================================================================================

#ifndef __Current_Sensor_h__
#define __Current_Sensor_h__
//====================================================================================
// Includes
//====================================================================================
#include <elapsedMillis.h>

//====================================================================================
// Defines and Enums
//====================================================================================
#define CYCLE_INTERVAL_TIME 50  // more than maybe 4 cycles
#define DEADBAND_FUDGE 5

enum {
  SENSOR_STATE_OFF = 0,
  SENSOR_STATE_ON,
  SENSOR_STATE_ON_BOOT
};
typedef enum _sensor_update_state{
  SENSOR_UPDATE_SAMPLING = 0,
  SENSOR_UPDATE_DONE_SAME_VALUE,
  SENSOR_UPDATE_DONE_NEW_VALUE,
  SENSOR_UPDATE_ON_DETECTED,
  SENSOR_UPDATE_OFF_DETECTED
} SensorUpdateState;

//====================================================================================
// Main Sensor class
//====================================================================================
class CurrentSensor {
public:
  // Constructor
  CurrentSensor(uint8_t analog_pin) {_pin = analog_pin; };

  // Init function
  void      init(void);                       // Our Initialize function
  SensorUpdateState   update(void);                     // Update - do analogRead - return 1 if cycle time completed
  uint32_t  curValue(void) {return _cur_value; };                // return the current value
  void      resetCounters(void);              // Reset our counters. 
  void      minOnValue(uint32_t val) {_min_on_value = val; }
  uint32_t  minOnValue() {return _min_on_value; }

  uint8_t   state() {return _state; };                            // What state are we in
  void      state(uint8_t s);                  // Will normally be called internal, but may want to call at init   
  uint8_t   previousState() {return _previous_state; }
  void      previousState(uint8_t ps) {_previous_state = ps;}
  time_t    onTime(void) {return _on_time;}   // Return the time the system turned on.
  uint32_t  minValue(void) {return _min_value;} 
  uint32_t  maxValue(void) {return _max_value;}
  uint32_t  avgValue(void) {return _sum_values/_cnt_values;}                   
private:
  uint8_t     _pin;                           // which analog pin to use    
  uint16_t    _avgOffvalue;                   // What is the average off value should be near res/2
  uint16_t    _deadband;                      // what we think the deadband is 
  uint32_t    _min_on_value;                  // What is the minimum value to trigger that we are on

  // Internal counters and sums   
  elapsedMillis _cycle_elapsed_time;        // Takes card of delta time
  uint32_t      _cycle_sum_deltas_sq;       // Sum of the square of the delta from avg
  uint16_t      _cycle_count_reads;         // Count of reads during this cycle;
  uint32_t      _cur_value;                // Last reported sqrt(sum/count)

  uint8_t       _state;              // 0=off, 1=on, maybe 2=on when program started... 
  uint8_t       _previous_state;            // The previous state...

  // Values updated while in on state. 
  time_t        _on_time;                   // What time did we turn on. 
  uint32_t      _min_value;                 // what was the minimum on value
  uint32_t      _max_value;                 // What is the maximum on value;
  uint32_t      _sum_values;                // sum of the values while on, 
  uint32_t      _cnt_values;                // count of values while on...
  
};

//====================================================================================
// Global Sensor objects
//====================================================================================
extern CurrentSensor *g_Sensors[];
extern const uint8_t CNT_SENSORS;

#endif

