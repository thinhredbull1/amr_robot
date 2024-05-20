#include <ros.h>
#include "digitalWriteFast.h"
#include "motors.h"
#include "config.h"
#include <util/atomic.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/time.h>
double speed_linear = 0;
double angular_speed = 0;
double speed_cmd_left = 0;
double speed_cmd_right = 0;
double speed_desired_left = 0;
double speed_desired_right = 0;
double speed_measure_l = 0;
double speed_measure_r = 0;
double speed_filter_l = 0;
double speed_filter_r = 0;
double last_speed_l = 0;
double last_speed_r = 0;
volatile int encoder_count_l = 0;
volatile int encoder_count_r = 0;
// double PID_left_param[] = {0.65,0, 0.1}; //0.645 0.242 0.025
// double PID_right_param[] = {0.65, 0, 0.1}; // 0.65 0.215 0.25
double PID_left_param[] = { 1.39, 0.239, 0 };   //0.645 0.242 0.025
double PID_right_param[] = { 1.335, 0.239, 0 };  // 0.65 0.215 0.25
// double PID_left_param[] = { 0.91, 0.2, 0.0145 };      //0.645 0.242 0.025
// double PID_right_param[] = { 0.81776, 0.2, 0.0364 };  // 0.65 0.215 0.25
long encoder_total_l = 0;
long encoder_total_r = 0;
float s_robot_rot_increment = 0;
float s_robot_angle = 0;
double p_rot = 0.085;
double d_rot = 0.002;
double angular_cmd_cal = 0;
float s_rot_error = 0;
double error_sum_l = 0;
double error_sum_r = 0;
double last_e_l = 0;
double last_e_r = 0;
bool ff = 0;
float last_e = 0;
float cm_robot = 0;
int y_speed = 0;
int index_his = 0;
const int AVERAGER_LENGTH = 5;
double m_right_his[AVERAGER_LENGTH] = { 0, 0, 0, 0, 0 };
double m_left_his[AVERAGER_LENGTH] = { 0, 0, 0, 0, 0 };
void onTwist(const geometry_msgs::Vector3Stamped& msg) {
  //forward
  speed_linear = msg.vector.x;
  angular_speed = msg.vector.z;
  y_speed = msg.vector.y;
  speed_desired_left = speed_linear - MOUSE_RADIUS * angular_speed;
  speed_desired_right = speed_linear + MOUSE_RADIUS * angular_speed;
}
void compute_rotate() {
  // float error=angular_speed-s_robot_rot_increment*LOOP_FREQUENCY;
  float error = angular_speed * LOOP_INTERVAL - s_robot_rot_increment;

  s_rot_error += error;

  float d_input = s_rot_error - last_e;
  last_e = s_rot_error;
  angular_cmd_cal = s_rot_error * p_rot + d_input * d_rot;
  if (angular_cmd_cal > max_speed_rot) angular_cmd_cal = max_speed_rot;
  else if (angular_cmd_cal < -max_speed_rot) angular_cmd_cal = -max_speed_rot;
}
ros::Subscriber<geometry_msgs::Vector3Stamped> sub("cmd_vel_2", onTwist);
geometry_msgs::Vector3Stamped speed_msg;
ros::Publisher speed_pub("speed", &speed_msg);
ros::NodeHandle nh;
void publishSpeed(int left_delta, int right_delta, double time) {
  speed_msg.header.stamp = nh.now();
  speed_msg.vector.x = left_delta;
  speed_msg.vector.y = right_delta;  // m
  speed_msg.vector.z = time;         // s
  speed_pub.publish(&speed_msg);
  nh.spinOnce();
  // nh.loginfo("Publishing odometry");
}
void reset_all_pid() {
  last_e_l = 0;
  last_e_r = 0;
  error_sum_l = 0;
  s_rot_error = 0;
  s_robot_angle = 0;
  cm_robot = 0;
  last_e = 0;
  error_sum_r = 0;
  for (int i = 0; i < AVERAGER_LENGTH; i++) {
    m_right_his[i] = 0;
    m_left_his[i] = 0;
  }
  index_his = 0;
}
bool receive_speed_command() {
  if (Serial.available()) {
    String c = Serial.readString();
    int index_now = c.indexOf("/");
    int index_l_desired = c.indexOf("L");
    int index_kp_desired = c.indexOf(":");
    int index_ff = c.indexOf("f");

    if (index_now != -1) {
      // speed_linear = c.substring(0, index_now).toFloat();
      // angular_speed = (c.substring(index_now + 1).toFloat());
      speed_desired_left = c.substring(0, index_now).toFloat();
      speed_desired_right = c.substring(index_now + 1).toFloat();
      // Serial.println("rec:" + String(speed_desired_left) + "," + String(speed_desired_right));
      return 1;
    } else if (index_ff != -1) {
      ff = c.substring(0, index_ff).toInt();
      Serial.println("ff:" + String(ff));
    }
    if (index_kp_desired != -1) {
      int index_cal = c.indexOf(",");
      int index_cal_2 = c.indexOf("#");
      if (index_cal != -1 && index_cal_2 != -1) {
        float new_kp = c.substring(index_kp_desired + 1, index_cal).toFloat();
        float new_ki = c.substring(index_cal + 1, index_cal_2).toFloat();
        float new_kd = c.substring(index_cal_2 + 1).toFloat();
        if (c[0] == 'L') {
          PID_left_param[0] = new_kp;
          PID_left_param[1] = new_ki;
          PID_left_param[2] = new_kd;
        } else {
          PID_right_param[0] = new_kp;
          PID_right_param[1] = new_ki;
          PID_right_param[2] = new_kd;
        }
        reset_all_pid();
        for (int i = 0; i < 3; i++) {
          Serial.print(PID_left_param[i]);
          Serial.print(" ");
        }
        for (int i = 0; i < 2; i++) {
          Serial.print(PID_right_param[i]);
          Serial.print(" ");
        }
        Serial.println(PID_right_param[2]);
      }
    }
  }
  return 0;
}
void compute_pid() {
  double error_l = speed_desired_left - speed_measure_l;
  double error_r = speed_desired_right - speed_measure_r;
  error_sum_l += error_l;
  error_sum_r += error_r;
  error_sum_l = constrain(error_sum_l, -max_speed, max_speed);
  error_sum_r = constrain(error_sum_r, -max_speed, max_speed);
  speed_cmd_left = error_l * PID_left_param[0] + error_sum_l * PID_left_param[1] + (error_l - last_e_l) * PID_left_param[2];
  speed_cmd_right = error_r * PID_right_param[0] + error_sum_r * PID_right_param[1] + (error_r - last_e_r) * PID_right_param[2];
  speed_cmd_left = constrain(speed_cmd_left, -max_speed, max_speed);
  speed_cmd_right = constrain(speed_cmd_right, -max_speed, max_speed);
  last_e_l = error_l;
  last_e_r = error_r;
}
void encoderLeftMotor() {
  static bool old_a = false;
  bool newA = digitalReadFast(PIN_ENCOD_B_MOTOR_LEFT);
  bool newB = digitalReadFast(PIN_ENCOD_A_MOTOR_LEFT);
  int delta = 0;
  if (newA && !old_a) {
    delta = newB > 0 ? dir_encod_M1 : -dir_encod_M1;
  }  // rising
  else {
    delta = newB > 0 ? -dir_encod_M1 : dir_encod_M1;
  }
  encoder_count_l -= delta;
  old_a = newA;
}
void encoderRightMotor() {
 static bool old_a = false;
  bool newA = digitalReadFast(PIN_ENCOD_B_MOTOR_RIGHT);
  bool newB = digitalReadFast(PIN_ENCOD_A_MOTOR_RIGHT);
  int delta = 0;
  if (newA && !old_a) {
    delta = newB > 0 ? dir_encod_M0 : -dir_encod_M0;
  }  // rising
  else {
    delta = newB > 0 ? -dir_encod_M0 : dir_encod_M0;
  }
  encoder_count_r -= delta;
  old_a = newA;
}
void setup() {
  // put your setup code here, to run once:
  init_motor();
  left_motor(0);
  right_motor(0);
  //analogWrite(EN1, 40);
  //analogWrite(EN2, 35);
  if (ros_serial) {
    nh.initNode();
    nh.getHardware()->setBaud(BAUDRATE);
    nh.subscribe(sub);
    nh.advertise(speed_pub);
  } else {
    Serial.begin(BAUDRATE);
    Serial.println("serial init");
  }
  pinMode(PIN_ENCOD_A_MOTOR_LEFT, INPUT_PULLUP);
  pinMode(PIN_ENCOD_B_MOTOR_LEFT, INPUT_PULLUP);
  pinMode(PIN_ENCOD_A_MOTOR_RIGHT, INPUT_PULLUP);
  pinMode(PIN_ENCOD_B_MOTOR_RIGHT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCOD_B_MOTOR_LEFT), encoderLeftMotor, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCOD_B_MOTOR_RIGHT), encoderRightMotor, CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (ros_serial) nh.spinOnce();
  static unsigned long last_time = micros();
  static unsigned long time_micros_meas = micros();
  static uint8_t count_print = 0;
  unsigned long time_loop_mc = micros() - last_time;
  static int left_publish = 0;
  static int right_publish = 0;
  static double publish_time = 0;
  if (time_loop_mc >= LOOP_MC) {
    static uint8_t count_publish = 0;
    count_publish += 1;
    last_time = micros();

    int left_delta = 0;
    int right_delta = 0;
    count_print++;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      left_delta = encoder_count_l;
      right_delta = encoder_count_r;
      encoder_count_l = 0;
      encoder_count_r = 0;
    }
    left_publish += left_delta;
    right_publish += right_delta;

    static double m_right_total = 0;
    static double m_left_total = 0;


    encoder_total_l += left_delta;
    encoder_total_r += right_delta;
    // s_robot_rot_increment = (-left_delta * cm_per_count_l + right_delta * cm_per_count_r) * DEG_PER_CM_DIFFERENCE;
    // s_robot_angle += s_robot_rot_increment;
    // compute_rotate();
    // speed_desired_left = speed_linear - (PI / 180.0) * MOUSE_RADIUS * angular_cmd_cal;
    // speed_desired_right = speed_linear + (PI / 180.0) * MOUSE_RADIUS * angular_cmd_cal;
    unsigned long currT = micros();
    float delta_time = ((float)(currT - time_micros_meas)) / 1.0e6;
    time_micros_meas = currT;
    publish_time += delta_time;
    if (abs(left_delta) < 1) speed_measure_l = 0;
    else speed_measure_l = ((left_delta * speed_convert) / delta_time);
    if (abs(right_delta) < 1) speed_measure_r = 0;
    else speed_measure_r = ((right_delta * speed_convert) / delta_time);  //mm/s
    speed_filter_l = a_coeff * speed_filter_l + b_coeff * speed_measure_l + last_speed_l * b_coeff;
    speed_filter_r = a_coeff * speed_filter_r + b_coeff * speed_measure_r + last_speed_r * b_coeff;
    last_speed_l = speed_measure_l;
    last_speed_r = speed_measure_r;
    compute_pid();
    int pwm_left = 0;
    int pwm_right = 0;
    if (ff == 1) {
      pwm_left = speed_desired_left * SPEED_FF_LEFT + sgn(speed_desired_left) * BIAS_FF;
      pwm_right = speed_desired_right * SPEED_FF_RIGHT + sgn(BIAS_FF);  // thuc chat la speed -> to voltage chu k phai speed -> pwm
    }
    pwm_left += speed_cmd_left * SPEED_FF_LEFT;
    pwm_right += speed_cmd_right * SPEED_FF_RIGHT;
    pwm_right = constrain(pwm_right, -max_pwm_value, max_pwm_value);
    pwm_left = constrain(pwm_left, -max_pwm_value, max_pwm_value);

    if (ros_serial) {
      if (speed_desired_left == 0) {
        s_rot_error = 0;
        pwm_left = 0;
      }
      if (speed_desired_right == 0) {
        pwm_right = 0;
        s_rot_error = 0;
      }
      left_motor(pwm_left);
      right_motor(pwm_right);

      if (micros() - last_time >= LOOP_MC) nh.loginfo("over time");
    } else {
      static bool rec_uart = 0;
      static unsigned long time_start = millis();
      // Serial.print(encoder_total_l);
      // Serial.print(",");
      // Serial.println(encoder_total_r);
      if (receive_speed_command() && rec_uart == 0) {
        rec_uart = 1;
        time_start = millis();
        reset_all_pid();
      }
      if (rec_uart == 1) {
        left_motor(pwm_left);
        right_motor(pwm_right);
        static int period = 800;
        uint32_t time_now = millis() - time_start;
        float sinus = sin(2 * PI * time_now / period);  // base pattern
        //  speed_desired_left = (2 * 0.16 / PI) * asin(sinus); // triangle
        if (count_print >= 1) {
          Serial.print(speed_filter_l, 4);
          Serial.print(",");
          Serial.println(speed_filter_r, 4);
          count_print = 0;
        }
        if (time_now >= 2000) {
          speed_desired_left = 0;
          speed_desired_right = 0;
          rec_uart = 0;
          count_print = 0;
          left_motor(0);
          right_motor(0);
          Serial.println("done");
        }
      }
    }
  }
  static unsigned long timer_2 = micros();
  if (micros() - timer_2 > LOOP_PUB && ros_serial) {
    timer_2 = micros();
    publishSpeed(left_publish, right_publish, publish_time);
    left_publish = 0;
    right_publish = 0;
    publish_time = 0;
  }
}
