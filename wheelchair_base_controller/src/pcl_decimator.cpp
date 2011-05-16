#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <wheelchair_base_controller/PCLDecimatorConfig.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include "pcl/io/pcd_io.h"
#include "pcl/filters/passthrough.h"
#include <string>

ros::Publisher cloudout_pub_;

std::string fieldName_ = "y";
float bandWidth_ = 0.01;
int numBands_ = 11;
float startPoint_ = -1.0;
float endPoint_ = 1.0;

// CALLBACKS
void pointcloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
   // Create the filtering objects
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName (fieldName_);
  // Do the first one manually so that certain parameters like frame_id always match
  pass.setFilterLimits (startPoint_, startPoint_+bandWidth_);
  pass.filter(*cloud_filtered);
  *output_cloud = *cloud_filtered;
  
  
  float dL = endPoint_-startPoint_;
  
  for(int i = 1; i < numBands_; i++){
    float sLine = startPoint_ + i*dL/numBands_;
    float eLine = startPoint_ + i*dL/numBands_ + bandWidth_;
    pass.setFilterLimits (sLine, eLine);
    pass.filter(*cloud_filtered);
    *output_cloud += *cloud_filtered;
  }
  cloudout_pub_.publish(*output_cloud);
}



void callback(wheelchair_base_controller::PCLDecimatorConfig &config, uint32_t level)
{
  fieldName_ = config.field_name;
  numBands_ = config.num_bands;
  bandWidth_ = config.band_width;
  startPoint_ = config.start_threshold;
  endPoint_ = config.end_threshold;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcl_decimator");
  ros::NodeHandle n;
  cloudout_pub_ = n.advertise< pcl::PointCloud<pcl::PointXYZ> >("/camera/depth/points_passthrough", 1);

  ros::Subscriber kinect_sub = n.subscribe("/camera/depth/points", 1, pointcloudCallback);
  
  dynamic_reconfigure::Server<wheelchair_base_controller::PCLDecimatorConfig> srv;
  dynamic_reconfigure::Server<wheelchair_base_controller::PCLDecimatorConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  srv.setCallback(f);

  ros::spin();

  return 0;
}




