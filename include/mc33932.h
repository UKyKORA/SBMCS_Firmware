#ifndef MC33932_H
#define MC33932_H

#include <Arduino.h>

#include "sbmcs_pins.h"

#define FORWARD 1
#define REVERSE 0

// when the mc33932 fault pin reads low there is a fault condition
#define FAULT_LEVEL 0

#define DEBUG_MC33932 0

// number of analogReads to average together when measuring motor current
#define AVERAGE_SAMPLES 64

// limit us reading current and fault values to 50Hz
#define MC33932_MIN_UPDATE_PERIOD 20

class MC33932 {
 public:
 MC33932() : _m1Current(0), _m2Current(0), _m1Fault(false), _m2Fault(false),
    _m1PwmPin(M1_INB), _m2PwmPin(M2_INA), _m1PwmValue(0), _m2PwmValue(0),
    _m1Dir(REVERSE), _m2Dir(REVERSE), _lastUpdate(0) {};
  
  void begin() {
#ifdef DEBUG_MC33932
    Serial.println("Starting MC33932");
#endif
    // init fault detection pins and update local fault status
    pinMode(M1_STATUS, INPUT);
    pinMode(M2_STATUS, INPUT);
    _updateFaults();
#ifdef DEBUG_MC33932
    if( fault() ){
      Serial.println("fault status read from motor controller on startup.");
    }
#endif

    // init enable/disbale pins and place into disabled state
    pinMode(M1_ENABLE, OUTPUT);
    pinMode(M1_DISABLE, OUTPUT);
    pinMode(M2_ENABLE, OUTPUT);
    pinMode(M2_DISABLE, OUTPUT);
    disable();

    // init pwm control pins and set to 0 speed
    pinMode(M1_INA, OUTPUT);
    pinMode(M1_INB, OUTPUT);
    pinMode(M2_INA, OUTPUT);
    pinMode(M2_INB, OUTPUT);
    setM1Dir(REVERSE);
    setM2Dir(REVERSE);
    setM1Pwm(0);
    setM2Pwm(0);
    
    // init current sense analog inputs and update readings
    pinMode(IM1_SENSE, INPUT);
    pinMode(IM2_SENSE, INPUT);
    _updateCurrents();
  };

  void enable() {
    // do both pins need to be changed each time or are the enable
    // and disable pins provided by the motor controller for convenience?
    // to be sure, do both
    enableM1();
    enableM2();
  };

  // enable h bridge #1
  void enableM1() {
    digitalWrite(M1_ENABLE, HIGH);
    digitalWrite(M1_DISABLE, LOW);
    _m1Enable = true;
  };

  // enable h bridge #2
  void enableM2() {
    digitalWrite(M2_ENABLE, HIGH);
    digitalWrite(M2_DISABLE, LOW);
    _m2Enable = true;
  };
  
  void disable() {
    disableM1();
    disableM2();
  };

  void disableM1() {
    digitalWrite(M1_ENABLE, LOW);
    digitalWrite(M1_DISABLE, HIGH);
    _m1Enable = false;
  };

  void disableM2() {
    digitalWrite(M2_ENABLE, LOW);
    digitalWrite(M2_DISABLE, HIGH);
    _m2Enable = false;
  };

  bool isEnabled() {
    return isM1Enabled() | isM2Enabled();
  };

  bool isM1Enabled() {
    return _m1Enable;
  };
  
  bool isM2Enabled() {
    return _m2Enable;
  };
  
  void setM1Pwm(uint8_t pwm) {
    if( _m1PwmValue == pwm ) return;
    _m1PwmValue = pwm;
    analogWrite(_m1PwmPin, _m1PwmValue);
  };
  
  void setM2Pwm(uint8_t pwm) {
    if( _m2PwmValue == pwm ) return;
    _m2PwmValue = pwm;
    analogWrite(_m2PwmPin, _m2PwmValue);
  };

  void setM1Dir(bool dir) {
    if( dir == _m1Dir ) return;
    
    // this is a quick direction switch that may cause problems if
    // called when the motor is at rest
    if( dir == FORWARD ){
      analogWrite(_m1PwmPin, 0);
      _m1PwmPin = M1_INA;
    } else if( dir == REVERSE ){
      analogWrite(_m1PwmPin, 0);
      _m1PwmPin = M1_INB;
    }

    _m1Dir = dir;
    analogWrite(_m1PwmPin, _m1PwmValue);    
  };

  
  void setM2Dir(bool dir) {
    if( dir == _m2Dir ) return;
    
    // this is a quick direction switch that may cause problems if
    // called when the motor is at rest
    if( dir == FORWARD ){
      analogWrite(_m2PwmPin, 0);
      _m2PwmPin = M2_INB;
    } else if( dir == REVERSE ){
      analogWrite(_m2PwmPin, 0);
      _m2PwmPin = M2_INA;
    }

    _m2Dir = dir;
    analogWrite(_m2PwmPin, _m2PwmValue);    
  };

  bool getM1Dir() { return _m1Dir; };
  bool getM2Dir() { return _m2Dir; };

  float getM1Current() { return _m1Current; };
  float getM2Current() { return _m2Current; };
  
  bool fault() {
    return _m1Fault | _m2Fault;
  };

  unsigned long update(){
    unsigned long now = millis();
    return update(now);
  }
  
  unsigned long update(unsigned long now){
    if( now - _lastUpdate > MC33932_MIN_UPDATE_PERIOD ){
      _updateFaults();
      _updateCurrents();
      _lastUpdate = now;
    }
    return _lastUpdate;
  }

 private:
  void _updateFaults() {
    _m1Fault = (digitalRead(M1_STATUS) == FAULT_LEVEL);
    _m2Fault = (digitalRead(M2_STATUS) == FAULT_LEVEL);
  };

  void _updateCurrents() {
    int i1 = 0, i2 = 0, i;
    for( i = 0; i < AVERAGE_SAMPLES; i++ ){
      i1 += analogRead(IM1_SENSE);
      i2 += analogRead(IM2_SENSE);
    };
    
    // the current through the 270 ohm sense resistor is 0.24% of the motor current
    _m1Current = ( (float)i1/AVERAGE_SAMPLES ) / 1023.0 / 0.0024 / 270.0 * 3.3;
    _m2Current = ( (float)i2/AVERAGE_SAMPLES ) / 1023.0 / 0.0024 / 270.0 * 3.3;;
  };

  unsigned long _lastUpdate;
  int _m1PwmPin, _m2PwmPin;
  uint8_t _m1PwmValue, _m2PwmValue;
  float _m1Current, _m2Current;
  bool _m1Fault, _m2Fault;
  bool _m1Enable, _m2Enable;
  bool _m1Dir, _m2Dir;
};
#endif
