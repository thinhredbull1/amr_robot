

#ifndef CONFIG_H
#define CONFIG_H
#include <math.h>

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
#define ros_serial 1
#define dir_encod_M1 1
#define dir_encod_M0 -1
const float bias=0.001;
const float coeff=0;
const float d_wheel_ff=0.105;
const float pulse_ff=800.0;
const float gear_ff=0.9875;
const float WHEEL_DIAMETER =0.105; // 32.708; // Adjust on test cm
const float wheel_cm=WHEEL_DIAMETER*100.0;

// const float cm_per_count_l=0.0787096774193548;
// const float cm_per_count_r=0.0790999567286889;

const float ENCODER_PULSES = 800.0; /// 395 count
const float GEAR_RATIO = 1.05; // robot move > khoang cach thuc te --> giam 31.38 30.25
const float speed_convert=(PI*WHEEL_DIAMETER)/(ENCODER_PULSES * GEAR_RATIO);
const float speed_convert_ff=(PI*d_wheel_ff)/(pulse_ff * gear_ff);
const float MOUSE_RADIUS = 0.156038; // 39.50; robot move < thuc te -> tang
const float ROTATION_BIAS = 0; // Negative makes robot curve to left
const float SPEED_FF_LEFT =(380.0313*speed_convert_ff)/speed_convert; //  T_lay mau = ENCODER_PULSES=400 ; Gear = 0.9875 , Wheel = 0.105
const float SPEED_FF_RIGHT=(412.0313*speed_convert_ff)/speed_convert; //412.0313
const float BIAS_FF = 0.0;
const float DEG_PER_CM_DIFFERENCE = (180.0 / (2 * MOUSE_RADIUS * PI));
const float cm_per_count_l=(1-bias)*PI*wheel_cm/(ENCODER_PULSES*GEAR_RATIO);
const float cm_per_count_r=(1+bias)*PI*wheel_cm/(ENCODER_PULSES*GEAR_RATIO);
///
const float a_coeff=0.22826091;
const float b_coeff=0.38586955;
//***************************************************************************//
const double max_speed_rot=10.5;

///min_pwm=25;
const float fre_publish=10.0;

const float LOOP_FREQUENCY =  50.0;
const int loop_to_pub=LOOP_FREQUENCY/fre_publish;
const float LOOP_INTERVAL = (1.0 / LOOP_FREQUENCY);
const float LOOP_MS=1000.0*LOOP_INTERVAL;
const float LOOP_MC=LOOP_MS*1000.0;
const float LOOP_PUB=(1/fre_publish)*1e6;
//***************************************************************************//
const uint32_t BAUDRATE = 57600;
//***************************************************************************//
const float MAX_MOTOR_VOLTS = 11.8;
// const int max_pwm_value=pow(2,resolution)-1;
const double max_speed = 0.42; //m/s
const int max_pwm_value=255;
//**** HARDWARE ***********************************************//
const int PIN_ENCOD_A_MOTOR_LEFT = 21;
const int PIN_ENCOD_B_MOTOR_LEFT = 20;
const int PIN_ENCOD_A_MOTOR_RIGHT = 19;
const int PIN_ENCOD_B_MOTOR_RIGHT = 18;
#define EN1 5
#define IN1 6
#define IN2 7
#define IN3 10
#define IN4 9
#define EN2 8
#endif
