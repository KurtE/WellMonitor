//====================================================================================
// Header file for my Current Sensor object
//====================================================================================

#ifndef __Current_Sensor_h__
#define __Current_Sensor_h__
//====================================================================================
// Includes
//====================================================================================
#include "globals.h"
#include <elapsedMillis.h>
#include <ADC.h>
#include <DMAChannel.h>

//====================================================================================
// Defines and Enums
//====================================================================================
#define CYCLES_TO_REPORT 400          //  Count of Intervals per cycle (About 4 AC cycles) or 15th of second
#define CYCLE_TIME_US 40000       // 25 times per second. 
//#define CYCLE_TIME_US 250 //166.666666     // Interval timer cycle time in us (1000000/(60*100))
#define DEADBAND_FUDGE 5
#define DELTA_CENTER_MAX  25

enum {
  SENSOR_STATE_OFF = 0,
  SENSOR_STATE_ON,
  SENSOR_STATE_CHANGED,    // On time or off time changed...
  SENSOR_STATE_ON_BOOT
};

enum {
  SENSOR_UPDATE_DONE_NO_UPDATE = 0,
  SENSOR_UPDATE_DONE_NEW_VALUE = 1,
  SENSOR_UPDATE_ON_DETECTED = 2,
  SENSOR_UPDATE_OFF_DETECTED = 4,
  SENSOR_UPDATE_ON_BOOT_DETECTED = 8
};

enum {
  SENSOR_SCAN_STOP = 0,
  SENSOR_SCAN_START,
  SENSOR_SCAN_RUNNING
};

//====================================================================================
// Main Sensor class
//====================================================================================
class CurrentSensor {
  public:
    // Constructor
    CurrentSensor(uint8_t analog_pin, int8_t adc_num) {
      _pin = analog_pin;
      _adc_num = adc_num;
    };
    enum  { ADC_BUFFER_SIZE = 100, MIN_DELTA_ON = 10 };
    enum {CALIBRATE_BEGIN, CALIBRATE_DONE_DISPLAY, CALIBRATE_DONE};
    static void initSensors(void);            // Move all of our init stuff into member function here.
    static void init_ADC_and_DMA(void);
    static bool checkSensors(void);           // 
    static void updateStartTimes(uint32_t dt); // update start times if sensor is active
    static void updateSensorsProc (void);
    static uint8_t sensorsOn (void);          // returns a bit mask of which sensors are on 
    static void adc0_dma_isr(void);
    static void adc1_dma_isr(void);
    static time_t todaysStartTime() {return _todays_midnight;}
    static void todaysStartTime(time_t t) {_todays_midnight = t;}
    
    // Global to call
    static volatile uint8_t   any_sensor_changed;
    static volatile bool      show_sensor_data;         // Show sensor data?
    static volatile uint8_t   sensor_scan_state;        // Should we do scanning.  0 - no, 1 - start, 2 running...
    static volatile uint8_t   sensor_timeout_count;     // How many errors have we had?
    static IntervalTimer      timer;                    // An interval timer to use with this
    static uint16_t           _interval_counter;        // only need 1 bit but should work fine.
    static time_t             _todays_midnight;         // What is linux time for midnight today...

    // Setup two DMA channels to use
    static  DMAChannel        _adc0_dma;                // Dma channel for ADC0
    static  DMAChannel        _adc1_dma;                // DMA Channel for ADC1
    static  bool              _adc0_busy;               // Is ADC0 busy?
    static  bool              _adc1_busy;               // Is ADC1 busy?

    // Init function
    void      init();                                   // Our Initialize function
    void      startAnalogRead();                        // Start a read operation
    bool      completeAnalogRead();                     // Lets complete the read operation
    uint16_t  curValue(void) {return _cur_value;};     // return the current value
    void      curValue(uint16_t val) {_cur_value = val;}

    // values to save and restore on boot
    uint16_t  centerPoint() {return _analog_center_point;}
    void      centerPoint(uint16_t val) {_analog_center_point = val;}
    uint16_t  deadband() {return _deadband;}
    void      deadband(uint16_t val) {_deadband = val; }

    uint8_t   state() {return _state;};                            // What state are we in
    void      state(uint8_t s);                  // Will normally be called internal, but may want to call at init
    uint8_t   displayState() {return _display_state; }
    void      displayState(uint8_t ps) {_display_state = ps;}
    uint16_t   displayVal() {return _display_value; }
    void      displayVal(uint16_t ps) {_display_value = ps; }
    time_t    onTime(void) {return _on_time; } // Return the time the sensor turned on.
    void      onTime(time_t t) {_on_time = t; } // Set the time - internal plus maybe init
    time_t    offTime(void) { return _off_time;} // Return the time the sensor turned off.
    void      offTime(time_t t) {_off_time = t; } // Internal plus maybe init.
    uint16_t  minValue(void) {return _min_value; }
    uint16_t  maxValue(void) {return _max_value; }
    uint16_t  avgValue(void) {return _sum_values / _cnt_values;}
    void      minValue(uint16_t val) {_min_value = val;}
    void      maxValue(uint16_t val) {_max_value = val; }
    void      avgValue(uint16_t val) {_sum_values = val; _cnt_values = 1; }
    uint8_t   calibrating() {return _calibrating;}
    void      calibrating(uint8_t cal) {_calibrating = cal;}
  private:
    uint8_t     _pin;                         // which analog pin to use
    int8_t      _adc_num;                     // which analog unit is the pin on?
    uint16_t    _analog_center_point;         // What is the average off value should be near res/2
    uint16_t    _deadband;                    // what we think the deadband is
    uint16_t    _min_on_value;                // What is the minimum value to trigger that we are on
    uint8_t     _analog_read_started;         // was the analog read started properly?

    // Internal counters and sums
    volatile uint16_t _adc_buf[ADC_BUFFER_SIZE]; // buffer 1...
    uint8_t     _calibrating;               // Are we calibrating?
    uint16_t    _cur_value;                 // Last reported sqrt(sum/count)
    uint16_t    _display_value;             // What value are we displaying?

    uint8_t     _state;                     // 0=off, 1=on, maybe 2=on when program started...
    uint8_t     _display_state;             // The previous state...

    // Values updated while in on state.
    time_t      _on_time;                   // What time did we turn on.
    time_t      _off_time;                  // What was the last time it turned off
    uint16_t    _min_value;                 // what was the minimum on value
    uint16_t    _max_value;                 // What is the maximum on value;
    uint32_t    _sum_values;                // sum of the values while on,
    uint16_t    _cnt_values;                // count of values while on...

};

//====================================================================================
// Global Sensor objects
//====================================================================================
extern CurrentSensor *g_Sensors[];
extern const uint8_t g_sensors_cnt;
extern ADC *adc;


#endif

