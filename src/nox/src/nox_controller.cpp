#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <cmath>

// double radius = 0.04;                              //Wheel radius, in m
// double wheelbase = 0.187;                          //Wheelbase, in m
double m_per_count_l;
double  m_per_count_r;
double two_pi = 6.28319;
int speed_act_left = 0;
int speed_act_right = 0;
double speed_req1 = 0.0;
double speed_req2 = 0.0;
double speed_dt = 0.0;
double x_pos = 0.0;
double y_pos = 0.0;
double theta = 0.0;
double speed_linear=0;
double p_rot=0;
double d_rot=0.0;
double speed_angular=0;
double angular_cmd_cal=0;
double s_rot_error=0;
double last_e=0;
const double PI=3.141592653589793;
const double ENCODER_PULSES = 400.0; /// 395 count
const float WHEEL_DIAMETER =0.105; // 32.708; // Adjust on test cm
ros::Time current_time;
ros::Time speed_time(0.0);
void handle_speed( const geometry_msgs::Vector3Stamped& speed) {
  speed_act_left = speed.vector.x;
  ROS_INFO("speed left : %d", speed_act_left);
  speed_act_right =speed.vector.y;
  ROS_INFO("speed right : %d", speed_act_right);
  speed_dt = speed.vector.z;
  speed_time = speed.header.stamp;
}
void handle_cmd_vel(const geometry_msgs::Twist& msg) {
  speed_linear=msg.linear.x;
  speed_angular=msg.angular.z;
  // ROS_INFO("angular: %f", speed_angular);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "nox_controller");
  ros::NodeHandle n;
  ros::NodeHandle nh_private_("~");
  ros::Subscriber sub = n.subscribe("speed", 40, handle_speed);
  ros::Subscriber sub_cmd = n.subscribe("cmd_vel", 40, handle_cmd_vel);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Vector3Stamped>("cmd_vel_2", 40);
  tf::TransformBroadcaster broadcaster;  
  double radius = 0.04;                              //Wheel radius, in m
  double robot_width = 0.3126;  
  double rate = 20.0;
  double rate_odom=20.0;
  double bias=-0.0012; // negative = L
  double GEAR_RATIO = 1.015; // robot move > khoang cach thuc te --> giam 31.38 30.25
 
  bool publish_tf = true;
  double dt = 0.0;
  double dx = 0.0;
  double dy = 0.0;
  double dth = 0.0;
  double dxy = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double vth = 0.0;
  char base_link[] = "base_footprint";
  char odom[] = "odom";
  char laser[]="/laser_link";
  char kinect[] = "/kinect";
  char camera_link[] = "/camera_link";
  std::string base_link_temp;


  ros::Duration d(1.0);
  nh_private_.getParam("publish_rate", rate);
  nh_private_.getParam("robot_width", robot_width);
  nh_private_.getParam("publish_tf", publish_tf);
  nh_private_.getParam("GEAR_RATIO", GEAR_RATIO);
  nh_private_.getParam("bias", bias);
  nh_private_.getParam("p_rot", p_rot);
  nh_private_.getParam("d_rot", d_rot);
  ROS_INFO("P_rot: %f", p_rot);
  ROS_INFO("bias:%f",bias);
  ROS_INFO("rate:%f",rate);
  ROS_INFO("gear:%f",GEAR_RATIO);
    m_per_count_l=(1-bias)*PI*WHEEL_DIAMETER/(ENCODER_PULSES*GEAR_RATIO);
   m_per_count_r=(1+bias)*PI*WHEEL_DIAMETER/(ENCODER_PULSES*GEAR_RATIO); 
  ros::Rate r(rate);
  while(n.ok()){
    ros::spinOnce();
    current_time = speed_time;
    dt = speed_dt;					//Time in s
    //ROS_INFO("dt : %f", dt);
    if(dt!=0.0)
    {
	    double delta_l=speed_act_left*m_per_count_l;
	    double delta_r=speed_act_right*m_per_count_r;
	    dxy = (delta_l+delta_r)/2;
	    // ROS_INFO("dxy : %f", dxy);
	    dth = ((delta_r-delta_l))/robot_width;
	    theta += dth;
	    dx = cos(theta) * dxy;
	    dy = sin(theta) * dxy;
	    
     	    if(theta >= two_pi/2.0) theta -= two_pi;
	    if(theta <= -two_pi/2.0) theta += two_pi;
	    x_pos+=dx;
	    y_pos+=dy;
	    //x_pos += (cos(theta) * dx - sin(theta) * dy);
	    //y_pos += (sin(theta) * dx + cos(theta) * dy);
	    
	    
	    if(p_rot==0)angular_cmd_cal=speed_angular;
	    else{
	    
	    double error = speed_angular - (dth)/dt;
	    s_rot_error += error;
	    
	    double d_input = s_rot_error - last_e;
	    last_e = s_rot_error;
	    angular_cmd_cal = s_rot_error * p_rot + d_input * d_rot;
	    if(speed_linear==0 && speed_angular==0)
	    {
	    	s_rot_error=0;
	    	angular_cmd_cal=0;
    		}
	    if (angular_cmd_cal > 0.65) angular_cmd_cal = 0.65;
	    else if (angular_cmd_cal < -0.65) angular_cmd_cal = -0.65;
	    }
	    ROS_INFO("angular:%f",theta*57.29);
	    // ROS_INFO("dt : %f", dt);
	    ROS_INFO("x : %f", x_pos);
	    ROS_INFO("y : %f", y_pos);
	   

	    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
	    geometry_msgs::Quaternion empty_quat = tf::createQuaternionMsgFromYaw(0);

	    if(publish_tf) {
	      geometry_msgs::TransformStamped t;
	      // geometry_msgs::TransformStamped k;
	      
	      t.header.frame_id = odom;
	      t.child_frame_id = base_link;
	      t.transform.translation.x = x_pos;
	      t.transform.translation.y = y_pos;
	      t.transform.translation.z = 0.0;
	      t.transform.rotation = odom_quat;
	      t.header.stamp = current_time;
	      
	      // k.header.frame_id = kinect;
	      // k.child_frame_id = camera_link;
	      // k.transform.translation.x = 0.0;
	      // k.transform.translation.y = 0.0;
	      // k.transform.translation.z = 0.0;
	      // k.transform.rotation = empty_quat;
	      // k.header.stamp = current_time;

	      broadcaster.sendTransform(t);
	      // broadcaster.sendTransform(k);
	    }

	    nav_msgs::Odometry odom_msg;
	    odom_msg.header.stamp = current_time;
	    odom_msg.header.frame_id = odom;
	    odom_msg.pose.pose.position.x = x_pos;
	    odom_msg.pose.pose.position.y = y_pos;
	    odom_msg.pose.pose.position.z = 0.0;
	    odom_msg.pose.pose.orientation = odom_quat;
	    if (speed_act_left == 0 && speed_act_right == 0){
	      odom_msg.pose.covariance[0] = 1e-9;
	      odom_msg.pose.covariance[7] = 1e-3;
	      odom_msg.pose.covariance[8] = 1e-9;
	      odom_msg.pose.covariance[14] = 1e6;
	      odom_msg.pose.covariance[21] = 1e6;
	      odom_msg.pose.covariance[28] = 1e6;
	      odom_msg.pose.covariance[35] = 1e-9;
	      odom_msg.twist.covariance[0] = 1e-9;
	      odom_msg.twist.covariance[7] = 1e-3;
	      odom_msg.twist.covariance[8] = 1e-9;
	      odom_msg.twist.covariance[14] = 1e6;
	      odom_msg.twist.covariance[21] = 1e6;
	      odom_msg.twist.covariance[28] = 1e6;
	      odom_msg.twist.covariance[35] = 1e-9;
	    }
	    else{
	      odom_msg.pose.covariance[0] = 1e-3;
	      odom_msg.pose.covariance[7] = 1e-3;
	      odom_msg.pose.covariance[8] = 0.0;
	      odom_msg.pose.covariance[14] = 1e6;
	      odom_msg.pose.covariance[21] = 1e6;
	      odom_msg.pose.covariance[28] = 1e6;
	      odom_msg.pose.covariance[35] = 1e3;
	      odom_msg.twist.covariance[0] = 1e-3;
	      odom_msg.twist.covariance[7] = 1e-3;
	      odom_msg.twist.covariance[8] = 0.0;
	      odom_msg.twist.covariance[14] = 1e6;
	      odom_msg.twist.covariance[21] = 1e6;
	      odom_msg.twist.covariance[28] = 1e6;
	      odom_msg.twist.covariance[35] = 1e3;
	    }
	    vx = (dt == 0)?  0 : dxy/dt;
	    vth = (dt == 0)? 0 : dth/dt;
	    odom_msg.child_frame_id = base_link;
	    odom_msg.twist.twist.linear.x = vx;
	    odom_msg.twist.twist.linear.y = 0.0;
	    odom_msg.twist.twist.angular.z = vth;
	    geometry_msgs::Vector3Stamped vel_msg;
	    
	    
	    vel_msg.header.stamp =current_time;
	    vel_msg.vector.x = speed_linear;
	    vel_msg.vector.y = 0;  // m
	    vel_msg.vector.z = angular_cmd_cal;         // s
	    odom_pub.publish(odom_msg);
	    vel_pub.publish(vel_msg);
    }
    r.sleep();
  }
}

