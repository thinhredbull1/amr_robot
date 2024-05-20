#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include"config.h"
void init_motor();
void left_motor(int pwm);
void right_motor(int pwm);
#endif
