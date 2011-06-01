#include <ros/ros.h>
#include <std_msgs/String.h>
#include <joy/Joy.h>
#include <std_msgs/Float32.h>
#include <dynamic_reconfigure/server.h>
#include <reflexive_plus/SpeedLimitConfig.h>
#include <math.h>

// PARAMETERS
float joystickScaleMin_ = 0.0;
float frontRangeMin_ = 0.0;
float frontRangeMax_ = 1.0;
float sideRangeMin_ = 0.0;
float sideRangeMax_ = 1.0;

float frontVal_ = 1000.0;
float sideVal_ = 1000.0;

ros::Publisher joyout_pub;

float scaleSaturate(float scale){
  if(scale < joystickScaleMin_){
    return joystickScaleMin_;
  } else if(scale > 1.0){
    return 1.0;
  }
  return scale;
}

void joyCallback(const joy::Joy::ConstPtr& msg)
{
	/* axes[0] is Wz
	axes[1] is Vx*/

	// Publish new joy message
	joy::Joy newJoy;
	newJoy.axes = msg->axes;
	newJoy.buttons = msg->buttons;
	
	float vscale = scaleSaturate((frontVal_-frontRangeMin_)/(frontRangeMax_-frontRangeMin_));
	newJoy.axes[1] = vscale*newJoy.axes[1];

	float wscale = scaleSaturate((sideVal_-sideRangeMin_)/(sideRangeMax_-sideRangeMin_));
	newJoy.axes[0] = wscale*newJoy.axes[0];
	
	joyout_pub.publish(newJoy);
}

void frontFloatCallback(const std_msgs::Float32::ConstPtr& msg){
  frontVal_ = msg->data;
}

void sideFloatCallback(const std_msgs::Float32::ConstPtr& msg){
  sideVal_ = msg->data;
}

void callback(reflexive_plus::SpeedLimitConfig &config, uint32_t level)
{
  joystickScaleMin_ = config.joystickScaleMin;
  frontRangeMin_ = config.frontRangeMin;
  frontRangeMax_ = config.frontRangeMax;
  sideRangeMin_ = config.sideRangeMin;
  sideRangeMax_ = config.sideRangeMax;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joystick_speed_limit");
  ros::NodeHandle n;
  joyout_pub = n.advertise<joy::Joy>("joy_output", 1);
  
  dynamic_reconfigure::Server<reflexive_plus::SpeedLimitConfig> srv;
  dynamic_reconfigure::Server<reflexive_plus::SpeedLimitConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  srv.setCallback(f);

  ros::Subscriber joy_sub = n.subscribe("joy", 100, joyCallback);
  
  ros::Subscriber front_sub = n.subscribe("/bubble/forward", 100, frontFloatCallback);
  
  ros::Subscriber side_sub = n.subscribe("/bubble/side", 100, sideFloatCallback);

  ros::spin();

  return 0;
}




