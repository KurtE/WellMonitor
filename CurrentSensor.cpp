#include <Arduino.h>
#include <EEPROM.h>
#include "CurrentSensor.h"
#include "globals.h"
#include "Display.h"
#include <TimeLib.h>


//==========================================================================
// Sensor global objects
//==========================================================================
CurrentSensor Sensor1(SENSOR1_PIN);
CurrentSensor Sensor2(SENSOR2_PIN);
CurrentSensor Sensor3(SENSOR3_PIN);
CurrentSensor Sensor4(SENSOR4_PIN);

CurrentSensor *g_Sensors[] = {&Sensor1, &Sensor2, &Sensor3, &Sensor4};
#define CNT_SENSORS (sizeof(g_Sensors)/sizeof(g_Sensors[0]))
const uint8_t g_sensors_cnt = CNT_SENSORS;
volatile uint32_t CurrentSensor::any_sensor_changed = 0;       // Set if any thing changed state
IntervalTimer CurrentSensor::timer;                       // An interval timer to use with this



typedef struct _well_monitor_eeprom_data {
  uint8_t version_data;   // what is our data version data
  uint8_t checksum;       // what is the checksum for the data.
  // Sensor Data
  uint16_t    avgOffvalues[CNT_SENSORS];                  // What is the average off value should be near res/2
  uint16_t    deadbands[CNT_SENSORS];                      // what we think the deadband is
} WELL_MONITOR_EEPROM_DATA;


//==========================================================================
// Static - Class level methods
//==========================================================================
void CurrentSensor::initSensors(void)
{
  // EEPROM data. - Maybe multiple sections
  // Beginning have some default settings for Avg off and deadband.
  // May have another section which keeps last on/off times.
  WELL_MONITOR_EEPROM_DATA wmee;

  EEPROM.get(EEPROM_SENSOR_INFO, wmee);

  uint8_t checksum = 0;
  uint8_t *pt = ((uint8_t *)&wmee) + 2;
  uint8_t cb = sizeof(wmee) - 2; // don't checksum the

  while (cb--) {
    checksum += *pt++;
  }

  Serial.println("Initialize Sensors");
  if ((wmee.version_data != EEPROM_SENSOR_VERSION) || (wmee.checksum != checksum)) {
    Serial.println("EEPROM Sensor init data invalid");
    // Data is not valid... init to something
    for (uint8_t i = 0; i < CNT_SENSORS; i++) {
      wmee.avgOffvalues[i] = 0xffff;
      wmee.deadbands[i] = 0;
    }
  }

  uint8_t data_changed = false;

  for (uint8_t iSensor = 0; iSensor < CNT_SENSORS; iSensor++) {
    g_Sensors[iSensor]->init(wmee.avgOffvalues[iSensor], wmee.deadbands[iSensor]);

    // See if the data changed - Maybe should add fudge factor here?
    if (g_Sensors[iSensor]->avgOffValue() != wmee.avgOffvalues[iSensor]) {
      wmee.avgOffvalues[iSensor] = g_Sensors[iSensor]->avgOffValue();
      data_changed  = true;
    }
    if (g_Sensors[iSensor]->deadband() != wmee.deadbands[iSensor]) {
      wmee.deadbands[iSensor] = g_Sensors[iSensor]->deadband();
      data_changed  = true;
    }
  }

  // If data changed save it back to eeprom;
  if (data_changed) {
    wmee.checksum = 0;
    pt = ((uint8_t *)&wmee) + 2;
    cb = sizeof(wmee) - 2; // don't checksum the

    while (cb--) {
      wmee.checksum += *pt++;
    }

    wmee.version_data = EEPROM_SENSOR_VERSION;
    EEPROM.put(EEPROM_SENSOR_INFO, wmee);
    Serial.println("EEPROM Sensor init data Updated");
  }

  // See if we need to init the display saying some of the sensors may have detected power at startup
  for (uint8_t i = 0; i < CNT_SENSORS; i++) {
    if (g_Sensors[i]->state() == SENSOR_STATE_ON_BOOT) {
      UpdateDisplaySensorData(i, SENSOR_UPDATE_ON_BOOT_DETECTED);
    }
  }


  // Start interval timer to take care of updating the sensor.
  pinMode(13, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  timer.begin(IntervalTimerProc, CYCLE_TIME_US);
}

//==========================================================================
// IntervalTimerProc - We will call off to read in the currents of each
// of our sensors.
//==========================================================================
void CurrentSensor::IntervalTimerProc ()
{
  digitalWrite(9, !digitalRead(9));
  // Lets cycle through all of the sensors
  uint32_t any_changed = 0;
  for (uint8_t iSensor = 0; iSensor < CNT_SENSORS; iSensor++) {
    any_changed =  (any_changed << 8) | g_Sensors[iSensor]->update();
  }
  if (any_changed) {
    digitalWrite(13, !digitalRead(13));
    CurrentSensor::any_sensor_changed = any_changed;
  }
}

//==========================================================================
// Init
//==========================================================================
void CurrentSensor::init(uint16_t avg_off, uint16_t db)
{
  uint32_t sum = 0;
  uint16_t min_sample = 0xffff;
  uint16_t max_sample = 0x0;
  uint16_t count_samples = 0;
  uint16_t val = analogRead(_pin);

  _cycle_elapsed_time = 0;

  // Initialize the state of some of the different members
  _min_on_value = MIN_CURRENT_ON;
  _state = SENSOR_STATE_OFF;

  while (_cycle_elapsed_time < 250) {
    val = analogRead(_pin);
    sum += val;
    if (val < min_sample) min_sample = val;
    if (val > max_sample) max_sample = val;
    count_samples++;
  }
  _avgOffvalue = sum / count_samples;
  _deadband = max(_avgOffvalue - min_sample, max_sample - _avgOffvalue) +  DEADBAND_FUDGE;

  // Lets do a quick check to see if these values look valid, if not use saved values...

  Serial.printf("Pin: %d Sum: %u, Count: %d, Avg: %u(%u), Min: %d, Max: %u, Deadband: %d(%d)\n", _pin, sum,
                count_samples, _avgOffvalue, avg_off, min_sample, max_sample, _deadband, db);

  if (avg_off != 0xffff) {
    if ((abs(_avgOffvalue - avg_off) > 5) || (abs(_deadband - db) > 5)) {
      Serial.printf("Pin: %d - Power maybe on use saved settings\n", _pin);
      _avgOffvalue = avg_off;
      _deadband = db;
    }
  }

  _cur_value = 0;
  resetCounters();
}

//==========================================================================
// state - set the state to some new state.
//==========================================================================
void CurrentSensor::state(uint8_t s)
{
  if (_state != s) {
    _state = s;  // save away the new state;
    Serial.printf("Pin: %d Set State %d\n", _pin, s);
    if (_state) {
      // We are logically turning on so init some of the counters and the like.
      onTime(now());
      _min_value = _cur_value;
      _max_value = _cur_value;
      _sum_values = _cur_value;
      _cnt_values = 1;
    } else {
      offTime(now());
    }

  }
}


//==========================================================================
// resetCounters
//==========================================================================
void CurrentSensor::resetCounters()                    // Update - do analogRead - return 1 if cycle time completed
{
  _cycle_elapsed_time = 0;
  _cycle_sum_deltas_sq = 0;
  _cycle_count_reads = 0;
}

//==========================================================================
// Update
//==========================================================================
uint8_t CurrentSensor::update(void)                    // Update - do analogRead - return 1 if cycle time completed
{
  if (_cycle_count_reads > CYCLES_TO_REPORT) {
    digitalWrite(10, !digitalRead(10));

    uint8_t return_value = SENSOR_UPDATE_DONE_SAME_VALUE;
    uint32_t dt_avg = sqrt(_cycle_sum_deltas_sq / _cycle_count_reads);
    resetCounters();
    if (dt_avg != _cur_value) {
      _cur_value = dt_avg;
      //Serial.printf("%d: %d\n", _pin, _cur_value);
      return_value = SENSOR_UPDATE_DONE_NEW_VALUE;
    }
    if (_cur_value >= _min_on_value) {
      if (!_state) {
        state(SENSOR_STATE_ON);   // turn the state on
        return_value = SENSOR_UPDATE_ON_DETECTED;
      } else {
        if (_cur_value > _max_value)
          _max_value = _cur_value;
        if (_cur_value < _min_value)
          _min_value = _cur_value;
        _sum_values += _cur_value;
        _cnt_values++;
      }
    } else {
      if (_state) {
        state(SENSOR_STATE_OFF);
        return_value = SENSOR_UPDATE_OFF_DETECTED;
      }
    }
    return return_value; // let them know we went through a cycle
  }
  else
  {
    int cur_value =  analogRead(_pin);
    int delta_from_center = cur_value - _avgOffvalue;
    _cycle_sum_deltas_sq += delta_from_center * delta_from_center;
    _cycle_count_reads++;
  }
  return SENSOR_UPDATE_SAMPLING;
}

