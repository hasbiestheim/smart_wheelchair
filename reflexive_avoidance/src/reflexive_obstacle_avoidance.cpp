#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <reflexive_avoidance/ReflexConfig.h>
#include <std_msgs/String.h>
#include <joy/Joy.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

// PARAMETERS
double frontLaserDist_;
double frontLaserAngle_;
double sideLaserDist_;
double sideLaserAngle_;

float rangeFrontThreshold_;
float rangeSideThreshold_;

int kinectPoints_;

// CALLBACKS

bool laserFrontAllow = true;
bool laserLeftAllow = true;
bool laserRightAllow = true;

bool uselaser_;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	// Start by assuming a clear front, yay optimism!
	laserFrontAllow = true;
	laserLeftAllow = true;
	laserRightAllow = true;
	
	float centerPoint = (msg->angle_max - msg->angle_min)/2.0;
	
	float frontLimitLow = centerPoint - frontLaserAngle_/2.0;
	float frontLimitHigh = centerPoint + frontLaserAngle_/2.0;
	
	float rightLimitLow = msg->angle_min;
	float rightLimitHigh = msg->angle_min + sideLaserAngle_;
	
	float leftLimitLow = msg->angle_max - sideLaserAngle_;
	float leftLimitHigh = msg->angle_max;

	float angle = msg->angle_min;
	for(uint i = 0; i <= msg->ranges.size(); i++){
	  // Check if valid beam
	  float range = msg->ranges[i];
	  if(range >= msg->range_min && range <= msg->range_max){
	    // Check if within center range
	    if(angle >= frontLimitLow && angle <= frontLimitHigh && laserFrontAllow){
	      if(range < frontLaserDist_){
		laserFrontAllow = false;
		ROS_DEBUG("Danger according to Front Laser.");
	      }
	    }
	    // Check if within right range
	    else if(angle >= rightLimitLow && angle <= rightLimitHigh && laserRightAllow){
	      if(range < sideLaserDist_){
		laserRightAllow = false;
		ROS_DEBUG("Danger according to Right Laser.");
	      }
	    }
	    // Check if within left range
	    else if(angle >= leftLimitLow && angle <= leftLimitHigh && laserLeftAllow){
	      if(range < sideLaserDist_){
		laserLeftAllow = false;
		ROS_DEBUG("Danger according to Left Laser.");
	      }
	    }
	  }
	}	
}

bool rangerFrontAllow = true;
bool rangerLeftAllow = true;
bool rangerRightAllow = true;

bool useranger_;

void rangeCallback(const sensor_msgs::Range::ConstPtr& msg)
{
	// Record from message
	std::string frameID = msg->header.frame_id;
	float range = msg->range;
	// Compare to min to make sure not misfire
	if(range >= msg->min_range){
		// Compare each sonar
		if(frameID == "frontLeftSonar" || frameID == "frontRightSonar"){ 
			if(range < rangeFrontThreshold_){
				rangerFrontAllow = false;
				ROS_DEBUG("Danger according to Front Rangers");
			} else {
				rangerFrontAllow = true;
			}
		} else if(frameID == "diagLeftSonar" || frameID == "diagRightSonar"){

		} else if(frameID == "sideLeftSonar"){
			if(range < rangeSideThreshold_){
				rangerLeftAllow = false;
				ROS_DEBUG("Danger according to Side Left Ranger");
			} else {
				rangerLeftAllow = true;
			}
		} else if(frameID == "sideRightSonar"){
			if(range < rangeSideThreshold_){
				rangerRightAllow = false;
				ROS_DEBUG("Danger according to Side Right Laser");
			} else {
				rangerRightAllow = true;
			}
		}
	}
}

bool usekinect_;

bool kinectAllow = true;

void pointcloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
	// If number of points is > N, then report that we should not continue forward

	size_t currentPoints = cloud->points.size();

	//ROS_INFO_STREAM("Number of Kinect Points: " << currentPoints);

	if((int)currentPoints > kinectPoints_){
		kinectAllow = false;
		ROS_DEBUG("Danger according to Kinect");
	} else {
		kinectAllow = true;
	}
}

ros::Publisher joyout_pub;

ros::Publisher collision_warn_pub;

void joyCallback(const joy::Joy::ConstPtr& msg)
{
	/* axes[0] is Wz
	axes[1] is Vx*/

	// Publish new joy message
	joy::Joy newJoy;
	newJoy.axes = msg->axes;
	newJoy.buttons = msg->buttons;
	
	std_msgs::Bool collision_msg;
	collision_msg.data = true;

	if(((!laserFrontAllow && uselaser_) || (!rangerFrontAllow && useranger_) || (!kinectAllow && usekinect_)) && (newJoy.axes[1] > 0.0)){
		newJoy.axes[1] = 0.0;
		collision_warn_pub.publish(collision_msg);
	}
	if(((!laserLeftAllow && uselaser_) || (!rangerLeftAllow && useranger_)) && (newJoy.axes[0] > 0.0)){
		newJoy.axes[0] = 0.0;
		collision_warn_pub.publish(collision_msg);
	}
	if(((!laserRightAllow && uselaser_) || (!rangerRightAllow && useranger_)) && (newJoy.axes[0] < 0.0)){
		newJoy.axes[0] = 0.0;
		collision_warn_pub.publish(collision_msg);
	}
	joyout_pub.publish(newJoy);
}

void callback(reflexive_avoidance::ReflexConfig &config, uint32_t level)
{
  usekinect_ = config.use_kinect;
  kinectPoints_ = config.num_points;
  useranger_ = config.use_sonars;
  rangeFrontThreshold_ = config.front_sonar_thresh;
  rangeSideThreshold_ = config.side_sonar_thresh;
  uselaser_ = config.use_laser;
  frontLaserDist_ = config.front_laser_thresh;
  frontLaserAngle_ = config.front_laser_angle;
  sideLaserDist_ = config.side_laser_thresh;
  sideLaserAngle_ = config.side_laser_angle;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "new_joystick_limiter");
  ros::NodeHandle n;
  
  collision_warn_pub = n.advertise<std_msgs::Bool>("collision_warning", 1);
  joyout_pub = n.advertise<joy::Joy>("joy_output", 1);
    
  dynamic_reconfigure::Server<reflexive_avoidance::ReflexConfig> srv;
  dynamic_reconfigure::Server<reflexive_avoidance::ReflexConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  srv.setCallback(f);
  
  ros::Subscriber laser_sub = n.subscribe("scan", 100, laserCallback);

  ros::Subscriber ranger_sub = n.subscribe("range", 100, rangeCallback);

  ros::Subscriber kinect_sub = n.subscribe("input_cloud", 1, pointcloudCallback);

  ros::Subscriber joy_sub = n.subscribe("joy", 100, joyCallback);

  ros::spin();

  return 0;
}




