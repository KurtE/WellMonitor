#include <Arduino.h>
#include <EEPROM.h>
#include <ADC.h>
#include "CurrentSensor.h"
#include "globals.h"
#include "Display.h"
#include <TimeLib.h>


//==========================================================================
// Sensor global objects
//==========================================================================
CurrentSensor Sensor1(SENSOR1_PIN, ADC_1);
CurrentSensor Sensor2(SENSOR2_PIN, ADC_0);
CurrentSensor Sensor3(SENSOR3_PIN, ADC_0);
CurrentSensor Sensor4(SENSOR4_PIN, ADC_1);

CurrentSensor *g_Sensors[] = {&Sensor1, &Sensor2, &Sensor3, &Sensor4};
#define CNT_SENSORS (sizeof(g_Sensors)/sizeof(g_Sensors[0]))
const uint8_t g_sensors_cnt = CNT_SENSORS;
volatile uint8_t CurrentSensor::any_sensor_changed = 0; 
volatile uint8_t CurrentSensor::sensor_scan_state = 0;        // Should we do scanning.  0 - no, 1 - start, 2 running... 

IntervalTimer CurrentSensor::timer;                       // An interval timer to use with this
ADC *adc = new ADC(); // adc object



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
  if (!g_master_node){ 
    // Slave mode does not need this yet
    return; 
  }

  // Lets init the ADC library
  adc->setAveraging(16); // set number of averages
  adc->setResolution(12); // set bits of resolution
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::LOW_SPEED); // change the conversion speed
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::LOW_SPEED); // change the sampling speed
  adc->setAveraging(16, ADC_1); // set number of averages
  adc->setResolution(12, ADC_1); // set bits of resolution
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::LOW_SPEED, ADC_1); // change the conversion speed
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::LOW_SPEED, ADC_1); // change the sampling speed




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

  pinMode(13, OUTPUT);
  pinMode(1, OUTPUT);
  // Start interval timer to take care of updating the sensor.
  sensor_scan_state = SENSOR_SCAN_START;  // Tell inteval 
  timer.begin(IntervalTimerProc, CYCLE_TIME_US);
}

//==========================================================================
// IntervalTimerProc - We will call off to read in the currents of each
// of our sensors.
//==========================================================================
uint16_t CurrentSensor::_interval_counter; // only need 1 bit but should work fine.
void CurrentSensor::IntervalTimerProc ()
{
  // We are now doing the analog reads unblocking. 
  // We have two ADC units, so can only have two 
  // sensors doing analog reads per cycle  Sensors 0,1 and 2,3 should work
  // See if we are in a stopped state. 

  if (_interval_counter & 1) {
    g_Sensors[0]->completeAnalogRead();
    g_Sensors[1]->completeAnalogRead();
    g_Sensors[2]->startAnalogRead();
    g_Sensors[3]->startAnalogRead();
  } else {
    g_Sensors[2]->completeAnalogRead();
    g_Sensors[3]->completeAnalogRead();
    g_Sensors[0]->startAnalogRead();
    g_Sensors[1]->startAnalogRead();
  }
  _interval_counter++;

  // Probably a cleaner way to do this:
  if ((g_Sensors[0]->countAnalogReads() >= CYCLES_TO_REPORT) &&
      (g_Sensors[2]->countAnalogReads() >= CYCLES_TO_REPORT)) {

    uint8_t any_changed = 0;
    uint8_t update_retval = 0; 

    for (uint8_t iSensor = 0; iSensor < CNT_SENSORS; iSensor++) {
      update_retval = g_Sensors[iSensor]->update();
      if (update_retval) {
        if (update_retval > 1)
          any_changed = 1;      // Only set if any values or state changes...
          digitalWrite(1, !digitalRead(1));
      }
    }
    //Serial.println();
    if (any_changed) {
      CurrentSensor::any_sensor_changed = any_changed;
      //Serial.println("** IPTC **");
      digitalWrite(13, !digitalRead(13));
    }
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

  elapsedMillis cycle_elapsed_time = 0;        // Takes card of delta time

  // Initialize the state of some of the different members
  _min_on_value = MIN_CURRENT_ON;
  _state = SENSOR_STATE_OFF;
  _analog_read_started = 0;

  while (cycle_elapsed_time < 250) {
    val =  adc->analogRead(_pin, _adc_num);
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
    if ((abs(_avgOffvalue - avg_off) > 5) || ((_deadband - db) > 5)) {
      Serial.printf("Pin: %d - Power maybe on use saved settings\n", _pin);
      _avgOffvalue = avg_off;
      _deadband = db;
    }
  }

  _cur_value = 0;
  _display_state = 0;
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
  _cycle_sum_deltas_sq = 0;
  _cycle_count_reads = 0;
  _min_on_cycle = 0xffff;              // min  during this cycle of sums
  _max_on_cycle = 0;                // Max during this cycle of sums
}

//==========================================================================
// startAnalogRead
//==========================================================================
void CurrentSensor::startAnalogRead(void)                    // Update - do analogRead - return 1 if cycle time completed
{

 if (!adc->startSingleRead(_pin, _adc_num)) { // also: startSingleDifferential, analogSynchronizedRead, analogSynchronizedReadDifferential
    Serial.println("Start single failed");
    _analog_read_started = 0;
  }  else {
    _analog_read_started = 1;
  } 
}

//==========================================================================
// completeAnalogRead - complete the analog read
//==========================================================================
void CurrentSensor::completeAnalogRead(void)                    // Update - do analogRead - return 1 if cycle time completed
{
  if (!_analog_read_started) {
    return;
  }
  if (!adc->isComplete(_adc_num)) {
    Serial.printf("%d(%d)!isComplete\n", _pin, _adc_num);
  }
  uint16_t cur_value =  adc->readSingle(_adc_num);
  if (cur_value > _max_on_cycle) _max_on_cycle = cur_value;
  if (cur_value < _min_on_cycle) _min_on_cycle = cur_value;

  int delta_from_center = (int)cur_value - _avgOffvalue;
  _cycle_sum_deltas_sq += delta_from_center * delta_from_center;
  _cycle_count_reads++;
  _analog_read_started = 0;
}

//==========================================================================
// Update
//==========================================================================
uint8_t CurrentSensor::update(void)                    // Update - do analogRead - return 1 if cycle time completed
{
  uint8_t return_value = SENSOR_UPDATE_DONE_SAME_VALUE;
  uint32_t dt_avg = sqrt(_cycle_sum_deltas_sq / _cycle_count_reads);
  //Serial.printf("%d ", _max_on_cycle-_min_on_cycle);
  resetCounters();

  if (dt_avg >= _min_on_value) {
    // only set New value if we are on!
    if (dt_avg != _cur_value) {
      _cur_value = dt_avg;
      //Serial.printf("%d: %d\n", _pin, _cur_value);
      return_value = SENSOR_UPDATE_DONE_NEW_VALUE;
      //Serial.printf("CSU(%d) %d %d %d\n", _pin, _cur_value, _min_on_value, _state);
    }

    if (_state != SENSOR_STATE_ON) {
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
    _cur_value = dt_avg;
    if (_state) {
      //Serial.printf("CSU  X %d %d\n", _cur_value, _state);
      state(SENSOR_STATE_OFF);
      return_value = SENSOR_UPDATE_OFF_DETECTED;
    }
  }
  return return_value; // let them know we went through a cycle
}

