#include "motors.h"
void init_motor()
{
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(EN1, OUTPUT);
    pinMode(EN2, OUTPUT);
    analogWrite(EN1,0);
    analogWrite(EN2,0);
}
void left_motor(int pwm) {
  if (pwm == 0) {
    analogWrite(EN2, 0);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  } else {
    bool dir = pwm > 0 ? 1 : 0;
    analogWrite(EN2, abs(pwm));
    digitalWrite(IN3, dir);
    digitalWrite(IN4, 1 - dir);
  }
}
void right_motor(int pwm) {
  if (pwm == 0) {
    analogWrite(EN1, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  } else {
    bool dir = pwm > 0 ? 1 : 0;
    analogWrite(EN1, abs(pwm));
    digitalWrite(IN1, dir);
    digitalWrite(IN2, 1 - dir);
  }
}