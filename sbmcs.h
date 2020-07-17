#ifndef SBMCS_H
#define SBMCS_H

#include <Arduino.h>
#include <Encoder.h>
#include <Servo.h>
#include "sbmcs_pins.h"
#include "mc33932.h"

#define AVERAGE_SAMPLES 64

class SBMCS {


 public:
 SBMCS() : _vBatVoltage(0.0), _5vCurrent(0.0) {};

  // library init function
  bool begin() {
    Serial.print("SBMCS Library Starting...");

    // set pin modes for LEDs, analog sensing and imu interrupts
    _initGpio();

    // init servo library
    // must be using the servo library that comes with adafruits
    // board support package. Other servo libs do not support the
    // samd51 timer structure
    servo.attach(SERVO_PWM);

    // init mc33932 dual h bridge control library
    motors.begin();    
  };

  // set servo position in degrees
  void setServoPosition(int pos) {
    servo.write(pos);
  };

  unsigned long enc1() {
    return _m1.read();
  }

  unsigned long enc2() {
    return _m2.read();
  }
  
  // read vbat battery voltage
  float getBatteryVoltage(){
    int i1 = 0, i;
    for( i = 0; i < AVERAGE_SAMPLES; i++ ){
      i1 += analogRead(VBAT_SENSE);
    };
    
    // the battery voltage is sensed through a voltage divider
    // 300k to vbat 80k to gnd
    _vBatVoltage = ( (float)i1/AVERAGE_SAMPLES ) / 1023.0 * 3.3 *  380.0 / 80.0;

    return _vBatVoltage;
  };

  // read current drawn from 5v rail
  float get5vCurrent(){
    int i1 = 0, i;
    for( i = 0; i < AVERAGE_SAMPLES; i++ ){
      i1 += analogRead(I5V_SENSE);
    };

    _5vCurrent = abs( 2.5 - (( (float)i1/AVERAGE_SAMPLES ) / 1023.0 * 3.3)) / 2.5 * 5;

    return _5vCurrent;
  };
  
  MC33932 motors;
  Servo servo;
 private:
  void _initGpio() {
    pinMode(IMU_INT, INPUT);
    pinMode(SERVO_PWM, OUTPUT);
    pinMode(ACT_LED, OUTPUT);
    pinMode(STAT_LED, OUTPUT);

    pinMode(VBAT_SENSE, INPUT);
    pinMode(I5V_SENSE, INPUT);
  }

  static Encoder _m1;
  static Encoder _m2;
  float _vBatVoltage, _5vCurrent;
};

Encoder SBMCS::_m1 = Encoder(M1_ENC_A, M1_ENC_B);
Encoder SBMCS::_m2 = Encoder(M2_ENC_A, M2_ENC_B);

#endif
