#ifndef SBMCS_H
#define SBMCS_H

#include <Arduino.h>
#include <Encoder.h>
#include <Servo.h>
#include "sbmcs_pins.h"
#include "mc33932.h"

#define AVERAGE_SAMPLES 64

// rosserial hardware class for its comms 
class SBMCS_Hardware : public ArduinoHardware
{
  public:
  SBMCS_Hardware():ArduinoHardware(&Serial1, 115200){};
};
// end rosserial hardware class

class SBMCS {

 public:
 SBMCS() : vBatVoltage(0.0), currentDraw5v(0.0) {};

  // library init function
  void begin() {
    Serial.println("SBMCS Library Starting...");

    // set pin modes for LEDs, analog sensing and imu interrupts
    pinMode(IMU_INT, INPUT);
    pinMode(SERVO_PWM, OUTPUT);
    pinMode(ACT_LED, OUTPUT);
    pinMode(STAT_LED, OUTPUT);

    pinMode(VBAT_SENSE, INPUT);
    pinMode(I5V_SENSE, INPUT);
  };

  int32_t enc1() {
    return m1.read();
  }

  int32_t enc2() {
    return m2.read();
  }
  
  // read vbat battery voltage
  float getBatteryVoltage(){
    int i1 = 0, i;
    for( i = 0; i < AVERAGE_SAMPLES; i++ ){
      i1 += analogRead(VBAT_SENSE);
    };
    
    // the battery voltage is sensed through a voltage divider
    // 300k to vbat 84.5k to gnd
    vBatVoltage = ( (float)i1/AVERAGE_SAMPLES ) / 1023.0 * 3.3 *  380.0 / 84.5;

    return vBatVoltage;
  };

  // read current drawn from 5v rail
  float get5vCurrent(){
    int i1 = 0, i;
    for( i = 0; i < AVERAGE_SAMPLES; i++ ){
      i1 += analogRead(I5V_SENSE);
    };

    currentDraw5v = abs( 2.5 - (( (float)i1/AVERAGE_SAMPLES ) / 1023.0 * 3.3)) / 2.5 * 5;

    return currentDraw5v;
  };

  static Encoder m1;
  static Encoder m2;
  float vBatVoltage, currentDraw5v;
};

Encoder SBMCS::m1 = Encoder(M1_ENC_A, M1_ENC_B);
Encoder SBMCS::m2 = Encoder(M2_ENC_A, M2_ENC_B);

#endif
