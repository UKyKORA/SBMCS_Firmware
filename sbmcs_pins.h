#ifndef SBMCS_PINS_H
#define SBMCS_PINS_H

// current sense for each motor
// v=0.24% of motor current
// i.e. analogRead(IM1_SENSE)/1023.0/0.0024/270*3.3
#define IM1_SENSE A0
#define IM2_SENSE A2

// battery voltage through voltage divider
// V_bat = analogRead(VBAT_SENSE)/1023.0*3.3*380.0/80.0
#define VBAT_SENSE A3

// output of hall effect current sensor on 5v rail from battery
#define I5V_SENSE  A6

// motor pwm
// M*_INA=LOW , M*_INB=PWM  -> PWM CW
// M*_INA=PWM , M*_INB=LOW  -> PWM CCW
#define M1_INA    9
#define M1_INB    10
#define M2_INA    12
#define M2_INB    11

// inputs, low=fault
#define M1_STATUS 25
#define M2_STATUS 13

// outputs
#define M1_ENABLE  39
#define M1_DISABLE 34
#define M2_ENABLE  37
#define M2_DISABLE 38

// output
#define SERVO_PWM 6

// input, trigger level set with IMU library
#define IMU_INT   15

// outputs, active high
#define ACT_LED   19
#define STAT_LED  36

// motor encoder inputs
#define M1_ENC_A   4
#define M1_ENC_B  35
#define M2_ENC_A  23
#define M2_ENC_B  24


#endif SBMCS_PINS_H
