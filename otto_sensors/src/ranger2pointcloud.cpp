#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <dynamic_reconfigure/server.h>
#include <otto_sensors/Range2PointConfig.h>
#include <math.h>

// PARAMETERS
size_t angular_slices_;
size_t axial_slices_;
double_t maximum_range_;

ros::Publisher pcl_pub;

void rangeCallback(const sensor_msgs::Range::ConstPtr& msg)
{
	// Record from message
	std::string frameID = msg->header.frame_id;
	float range = msg->range;
	float field_of_view = msg->field_of_view;
	// Compare to min to make sure not misfire
	if(range >= msg->min_range && range <= msg->max_range && range <= maximum_range_){
	  pcl::PointCloud<pcl::PointXYZ> cloud;
	  cloud.header = msg->header;
	  
	  bool angular_odd = (angular_slices_ % 2 == 1);
	  cloud.width = angular_slices_*axial_slices_;
	  if(angular_odd){ // number of points for odd angular slices, adjust for overlapping centers
	    cloud.width += -axial_slices_+1;
	  }
	  cloud.height = 1;
	  cloud.is_dense = false;
	  cloud.points.resize(cloud.width*cloud.height);
	  if(cloud.points.size() == 1){
	    cloud.points[0].x = range;
	    cloud.points[0].y = 0.0;
	    cloud.points[0].z = 0.0;
	  } else {
	    float dAngle = field_of_view/(float)(angular_slices_-1);
	    float dAxis = 3.14159265/(float)(axial_slices_);
	    int count = 0;
	    for(size_t i = 0; i < axial_slices_; i++){
	      for(size_t j = 0; j < angular_slices_; j++){
		if(!angular_odd || angular_slices_ / 2 + 1 != j + 1 || i == 0){ // Avoid duplicate centers
		  float angle = -field_of_view/2.0 + j*dAngle;
		  float axis_angle = i * dAxis;
		  cloud.points[count].x = range*cos(angle);
		  cloud.points[count].y = range*sin(angle)*cos(axis_angle);
		  cloud.points[count].z = range*sin(angle)*sin(axis_angle);
		  count++;
		}
	      }
	    }
	  }
	  
	  pcl_pub.publish(cloud);
	}
}

void callback(otto_sensors::Range2PointConfig &config, uint32_t level)
{
  angular_slices_ = config.angular_slices;
  axial_slices_ = config.radial_slices;
  maximum_range_ = config.maximum_range;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ranger2pointcloud");
  ros::NodeHandle n;
  pcl_pub = n.advertise<pcl::PointCloud<pcl::PointXYZ> >("ranger_pointcloud", 1);
  
  dynamic_reconfigure::Server<otto_sensors::Range2PointConfig> srv;
  dynamic_reconfigure::Server<otto_sensors::Range2PointConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  srv.setCallback(f);
  
  angular_slices_ = 4;
  axial_slices_ = 3;

  ros::Subscriber ranger_sub = n.subscribe("rangers", 100, rangeCallback);

  ros::spin();

  return 0;
}




