#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <dynamic_reconfigure/server.h>
#include <otto_sensors/ScanThreshConfig.h>

// PARAMETERS
float angle_min_;
float angle_width_;
float thresh_dist_;
bool reflect_angles_;

ros::Publisher laser_pub;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	// Record from message
	sensor_msgs::LaserScan scan = *msg;
	float angle = scan.angle_min;
	for (unsigned int i = 0; i < scan.ranges.size(); i++){
	  if(angle >= angle_min_ + scan.angle_min && angle <= angle_min_ + scan.angle_min + angle_width_){
	    if(scan.ranges[i] < thresh_dist_){
	      scan.ranges[i] = scan.range_max+1.0;
	    }
	  }
	  if(reflect_angles_){
	    if(angle <= (scan.angle_max - angle_min_) && angle >= (scan.angle_max - angle_min_ - angle_width_)){
	      if(scan.ranges[i] < thresh_dist_){
		scan.ranges[i] = scan.range_max+1.0;
	      }
	    }
	  }
	  angle += scan.angle_increment;
	}
	laser_pub.publish(scan);
}

void callback(otto_sensors::ScanThreshConfig &config, uint32_t level)
{
  angle_min_ = config.angle_min;
  angle_width_ = config.angle_width;
  thresh_dist_ = config.thresh_dist;
  reflect_angles_ = config.reflect_angles;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserscan_threshold");
  ros::NodeHandle n;
  
  laser_pub = n.advertise<sensor_msgs::LaserScan>("scan_filtered", 1);
  
  dynamic_reconfigure::Server<otto_sensors::ScanThreshConfig> srv;
  dynamic_reconfigure::Server<otto_sensors::ScanThreshConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  srv.setCallback(f);

  ros::Subscriber laser_sub = n.subscribe("scan", 100, laserCallback);

  ros::spin();

  return 0;
}




