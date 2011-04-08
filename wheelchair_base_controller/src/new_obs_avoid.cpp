#include <ros/ros.h>
#include <std_msgs/String.h>
#include <joy/Joy.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

// PARAMETERS
int frontLaserCenter = 180;
int frontLaserWindow = 20;
int frontLaserDist = 0.5;
int rearLaserCenter = 0;
int rearLaserWindow = 10;
int rearLaserDist = 1.3;

int numberOfLaserBeams = 360;

float rangeFrontThreshold = 0.30;
float rangeSideThreshold = 0.20;

int kinectPoints = 100;

// CALLBACKS

bool laserFrontAllow = true;
bool laserRearAllow = true;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	// Start by assuming a clear front, yay optimism!
	laserFrontAllow = true;

	// For certain beams up front (around ping 180 since reversed), find and report the min
	float minRange = msg->range_min;
	for(int i = -frontLaserWindow/2; i <= frontLaserWindow/2; i++){

		int checkNum = frontLaserCenter + i;
		if(checkNum < 0){
			checkNum += numberOfLaserBeams;
		} else if(checkNum >= numberOfLaserBeams){
			checkNum -= numberOfLaserBeams;
		}

		float beamRange = msg->ranges[frontLaserCenter + i];
		if(beamRange < frontLaserDist && beamRange > minRange){
			laserFrontAllow = false;  // Oops, found a dangerous return
			ROS_DEBUG("Danger according to Front Laser");
			break;
		}
	}

	laserRearAllow = true;	

	for(int i = -rearLaserWindow/2; i <= rearLaserWindow/2; i++){

		int checkNum = rearLaserCenter + i;
		if(checkNum < 0){
			checkNum += numberOfLaserBeams;
		} else if(checkNum >= numberOfLaserBeams){
			checkNum -= numberOfLaserBeams;
		}

		float beamRange = msg->ranges[checkNum];
		if(beamRange < rearLaserDist && beamRange > minRange){
			laserRearAllow = false;  // Oops, found a dangerous return
			ROS_DEBUG("Danger according to Rear Laser");
			break;
		}
	}	
}

bool rangerFrontAllow = true;
bool rangerLeftAllow = true;
bool rangerRightAllow = true;

void rangeCallback(const sensor_msgs::Range::ConstPtr& msg)
{
	// Record from message
	std::string frameID = msg->header.frame_id;
	float range = msg->range;
	// Compare to min to make sure not misfire
	if(range >= msg->min_range){
		// Compare each sonar
		if(frameID == "frontLeftSonar" || frameID == "frontRightSonar"){ 
			if(range < rangeFrontThreshold){
				rangerFrontAllow = false;
				ROS_DEBUG("Danger according to Front Rangers");
			} else {
				rangerFrontAllow = true;
			}
		} else if(frameID == "diagLeftSonar" || frameID == "diagRightSonar"){

		} else if(frameID == "sideLeftSonar"){
			if(range < rangeSideThreshold){
				rangerLeftAllow = false;
				ROS_DEBUG("Danger according to Side Left Ranger");
			} else {
				rangerLeftAllow = true;
			}
		} else if(frameID == "sideRightSonar"){
			if(range < rangeSideThreshold){
				rangerRightAllow = false;
				ROS_DEBUG("Danger according to Side Right Laser");
			} else {
				rangerRightAllow = true;
			}
		}
	}
}

bool kinectAllow = true;

void pointcloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
	// If number of points is > N, then report that we should not continue forward

	size_t currentPoints = cloud->points.size();

	//ROS_INFO_STREAM("Number of Kinect Points: " << currentPoints);

	if((int)currentPoints > kinectPoints){
		kinectAllow = false;
		ROS_DEBUG("Danger according to Front Kinect");
	}
}

ros::Publisher joyout_pub;

void joyCallback(const joy::Joy::ConstPtr& msg)
{
	/* axes[0] is Wz
	axes[1] is Vx*/

	// Publish new joy message
	joy::Joy newJoy;
	newJoy.axes = msg->axes;
	newJoy.buttons = msg->buttons;

	if((!laserFrontAllow || !rangerFrontAllow || !kinectAllow) && newJoy.axes[1] > 0.0){
		newJoy.axes[1] = 0.0;
	}
	if(!laserRearAllow && newJoy.axes[1] < 0.0){
		newJoy.axes[1] = 0.0;
	}
	if(!rangerLeftAllow && newJoy.axes[0] > 0.0){
		newJoy.axes[0] = 0.0;
	}
	if(!rangerRightAllow && newJoy.axes[0] < 0.0){
		newJoy.axes[0] = 0.0;
	}
	
	joyout_pub.publish(newJoy);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "new_joystick_limiter");
  ros::NodeHandle n;
  joyout_pub = n.advertise<joy::Joy>("joy_output", 1);

  ros::Subscriber laser_sub = n.subscribe("scan", 100, laserCallback);

  ros::Subscriber ranger_sub = n.subscribe("sonars", 100, rangeCallback);

  ros::Subscriber kinect_sub = n.subscribe("input_cloud", 1, pointcloudCallback);

  ros::Subscriber joy_sub = n.subscribe("joy", 100, joyCallback);

  ros::spin();

  return 0;
}




