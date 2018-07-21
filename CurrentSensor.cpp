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
volatile uint8_t CurrentSensor::sensor_scan_state = 0;    // Should we do scanning.  0 - no, 1 - start, 2 running...
volatile bool CurrentSensor::show_sensor_data = false;    // default don't show data
DMAChannel  CurrentSensor::_adc0_dma;                     // Dma channel for ADC0
DMAChannel   CurrentSensor::_adc1_dma;                    // DMA Channel for ADC1
bool  CurrentSensor::_adc0_busy = false;                  // Is ADC0 busy?
bool  CurrentSensor::_adc1_busy = false;                  // Is ADC1 busy?
//volatile DMAMEM uint16_t CurrentSensor::_adc0_buf[ADC_BUFFER_SIZE]; // buffer 1...
//volatile DMAMEM uint16_t CurrentSensor::_adc1_buf[ADC_BUFFER_SIZE]; // buffer 1...

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
  if (!g_master_node) {
    // Slave mode does not need this yet
    return;
  }

  for (uint8_t iSensor = 0; iSensor < CNT_SENSORS; iSensor++) {
    g_Sensors[iSensor]->init();
  }

  // Lets init the ADC library
  // Note: we are going to use DMA and PDB to do the timing for us...
  adc->setAveraging(4); // set number of averages
  adc->setResolution(12); // set bits of resolution
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED); // change the conversion speed
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED); // change the sampling speed
  adc->setAveraging(4, ADC_1); // set number of averages
  adc->setResolution(12, ADC_1); // set bits of resolution
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED, ADC_1); // change the conversion speed
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED, ADC_1); // change the sampling speed

  // Lets setup Analog 0 dma
  _adc0_dma.source((volatile uint16_t&)ADC0_RA);
  //_adc0_dma.destinationBuffer(_adc0_buf, sizeof(_adc0_buf));
  _adc0_dma.triggerAtHardwareEvent (DMAMUX_SOURCE_ADC0);
  _adc0_dma.disableOnCompletion();
  NVIC_DISABLE_IRQ(IRQ_PDB); // we don't want or need the PDB interrupt


  // Lets setup Analog 1 dma
  _adc1_dma.source((volatile uint16_t&)ADC1_RA);
  //_adc1_dma.destinationBuffer(_adc1_buf, sizeof(_adc1_buf));
  _adc1_dma.triggerAtHardwareEvent (DMAMUX_SOURCE_ADC1);
  _adc1_dma.disableOnCompletion();

  // Not sure I will need this, will simply setup
  _adc0_dma.interruptAtCompletion();
  _adc0_dma.attachInterrupt(&adc0_dma_isr);
  _adc1_dma.interruptAtCompletion();
  _adc1_dma.attachInterrupt(&adc1_dma_isr);


  pinMode(1, OUTPUT);
  // Start interval timer to take care of updating the sensor.
  sensor_scan_state = SENSOR_SCAN_START;  // Tell inteval
#ifdef SENSORS_USE_INTERVAL_TIMER
  timer.begin(IntervalTimerProc, CYCLE_TIME_US);
#else
  // need to prime it.
  IntervalTimerProc();
#endif
}

//==========================================================================
// checkSensors - We will call off to read in the currents of each
// of our sensors.
//==========================================================================
elapsedMillis time_since_sensors_processed; 
bool CurrentSensor::checkSensors() {
  if (!g_master_node) return false;   // we are not master so don't do anything.

  // Now call the interval timer if we are not
#ifndef SENSORS_USE_INTERVAL_TIMER
  // Set some form of timeout, to make sure 
  // the system did not hang for some reason. Should not take a second let alone a few seconds
    if (time_since_sensors_processed < 5000) {
    if (_adc0_busy || _adc1_busy) return false; // still busy
  } else {
    Serial.println("Sensor timeout?");
  }

  IntervalTimerProc();
  time_since_sensors_processed = 0;
#endif
  return (CurrentSensor::any_sensor_changed);
}

//==========================================================================
// checkSensors - We will call off to read in the currents of each
// of our sensors.
//==========================================================================
void CurrentSensor::updateStartTimes(uint32_t dt) {
  for (uint8_t iSensor = 0; iSensor < CNT_SENSORS; iSensor++) {
    // See if sensor is active.
    if (g_Sensors[iSensor]->state() != SENSOR_STATE_OFF) {
      // lets try to update the on time
      g_Sensors[iSensor]->onTime(g_Sensors[iSensor]->onTime() + dt);
    }
  }
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

  bool completion_state = false;
  if (_interval_counter & 1) {
    completion_state = g_Sensors[0]->completeAnalogRead();
    completion_state |= g_Sensors[1]->completeAnalogRead();
    g_Sensors[2]->startAnalogRead();
    g_Sensors[3]->startAnalogRead();
  } else {
    completion_state = g_Sensors[2]->completeAnalogRead();
    completion_state |= g_Sensors[3]->completeAnalogRead();
    g_Sensors[0]->startAnalogRead();
    g_Sensors[1]->startAnalogRead();
  }
  _interval_counter++;  //

  // Actually now start the DMA/PDB...
  // The actual calls above set buffer and pin...
  _adc0_busy = true;
  _adc1_busy = true;
  _adc0_dma.enable();
  _adc1_dma.enable();
  adc->enableDMA(ADC_0);
  adc->enableDMA(ADC_1);
  _adc0_dma.enable();
  _adc1_dma.enable();
  adc->adc0->startPDB(60 * 50);
  adc->adc1->startPDB(60 * 50);

  //Serial.println();
  if (completion_state) {
    CurrentSensor::any_sensor_changed = 1;
    //Serial.println("** IPTC **");
    digitalWrite(13, !digitalRead(13));
  }
}

//==========================================================================
// Init
//==========================================================================
void CurrentSensor::init()
{

  // Initialize the state of some of the different members
  _min_on_value = MIN_CURRENT_ON;
  _state = SENSOR_STATE_OFF;
  _analog_read_started = 0;
  _calibrating = true;

  _cur_value = 0;
  _display_state = 0;
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
// startAnalogRead
//==========================================================================
void CurrentSensor::startAnalogRead(void)                    // Update - do analogRead - return 1 if cycle time completed
{

  if (_adc_num == 0) {
    // Each has their own buffer.
    _adc0_dma.destinationBuffer(_adc_buf, sizeof(_adc_buf));
    adc->adc0->startSingleRead(_pin);

  } else {
    _adc1_dma.destinationBuffer(_adc_buf, sizeof(_adc_buf));
    adc->adc1->startSingleRead(_pin);
  }
  _analog_read_started = 1;
}

//==========================================================================
// completeAnalogRead - complete the analog read
//==========================================================================
bool CurrentSensor::completeAnalogRead(void)                    // Update - do analogRead - return 1 if cycle time completed
{
  bool return_value = false;
  if (!_analog_read_started) {
    return false;
  }
  _analog_read_started = 0;

  // See if we are calibrating
  uint16_t min_value = 0xffff;
  uint16_t max_value = 0;

  if (_calibrating) {
    // Probably could get lots more fancy here...
    uint32_t sum_values = 0;
    for (int i = 0; i < CurrentSensor::ADC_BUFFER_SIZE; i++) {
      if (_adc_buf[i] > max_value) max_value = _adc_buf[i];
      if (_adc_buf[i] < min_value) min_value = _adc_buf[i];
      sum_values += _adc_buf[i];
      Serial.printf("%4u ", _adc_buf[i]);
      if ((i % 20) == 19) Serial.println();
    }
    _analog_center_point = sum_values / CurrentSensor::ADC_BUFFER_SIZE;

    Serial.printf("Calibrating(%d) %d <= %d <= %d\n", _pin, min_value, _analog_center_point, max_value);

    _calibrating = false;
    return false;
  }

  uint32_t sum_deltas_sq = 0;
  if (CurrentSensor::show_sensor_data) {
    Serial.printf("\nCurrent Sensor pin:%d ADC:%d\n", _pin, _adc_num);
  }
  for (int i = 0; i < CurrentSensor::ADC_BUFFER_SIZE; i++) {
    if (_adc_buf[i] > max_value) max_value = _adc_buf[i];
    if (_adc_buf[i] < min_value) min_value = _adc_buf[i];

    int delta_from_center = (int)_adc_buf[i] - _analog_center_point;
    sum_deltas_sq += delta_from_center * delta_from_center;
    if (CurrentSensor::show_sensor_data) {
      Serial.printf("%4u ", _adc_buf[i]);
      if ((i % 20) == 19) Serial.println();
    }
  }

  uint32_t dt_avg = sqrtf(sum_deltas_sq / CurrentSensor::ADC_BUFFER_SIZE);
  if (CurrentSensor::show_sensor_data) {
    Serial.printf("Min: %d Max: %d dt: %d\n", min_value, max_value, dt_avg);
  }

  // Add secondary check if (MAX-MIN) < size turn dt_avg off.
  if ((max_value - min_value) < CurrentSensor::MIN_DELTA_ON) dt_avg = 0;


  if (dt_avg >= _min_on_value) {
    // only set New value if we are on!
    if (dt_avg != _cur_value) {
      _cur_value = dt_avg;
      //Serial.printf("%d: %d\n", _pin, _cur_value);
      return_value = true;
      //Serial.printf("CSU(%d) %d %d %d\n", _pin, _cur_value, _min_on_value, _state);
    }

    if (_state != SENSOR_STATE_ON) {
      state(SENSOR_STATE_ON);   // turn the state on
      return_value = true;
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
      return_value = true;
    }
  }
  return return_value; // let them know we went through a cycle
}

void CurrentSensor::adc0_dma_isr(void)
{
  _adc0_busy = false;
  _adc0_dma.clearInterrupt();
  _adc0_dma.clearComplete();

  // Only process if both are finished.
  if (!_adc1_busy) {
    PDB0_CH1C1 = 0;
    PDB0_CH0C1 = 0;
    adc->adc0->stopPDB();
    adc->adc1->stopPDB();
    _adc0_dma.disable();
    _adc1_dma.disable();
    _adc0_dma.disable();
    adc->disableDMA(ADC_0);
    adc->disableDMA(ADC_1);
  }

}

void CurrentSensor::adc1_dma_isr(void)
{
  _adc1_busy = false;
  _adc1_dma.clearInterrupt();
  _adc1_dma.clearComplete();

  if (!_adc0_busy) {
    PDB0_CH1C1 = 0;
    PDB0_CH0C1 = 0;
    adc->adc0->stopPDB();
    adc->adc1->stopPDB();
    _adc0_dma.disable();
    _adc1_dma.disable();
    _adc0_dma.disable();
    adc->disableDMA(ADC_0);
    adc->disableDMA(ADC_1);
  }
}
