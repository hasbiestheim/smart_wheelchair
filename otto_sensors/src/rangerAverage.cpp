#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <iostream>

ros::Publisher for_pub;
ros::Publisher side_pub;

float forAvg = 0.0;
float sideAvg = 0.0;

std::list<float> forVals;
std::list<float> sideVals;

// PARAMETERS
std::string tframeID;
int N_;

void rangeCallback(const tf::TransformListener& listener, const sensor_msgs::Range::ConstPtr& msg)
{
    try{
	listener.waitForTransform(tframeID, msg->header.frame_id, ros::Time::now(), ros::Duration(1.0));
	// Record from message
	std::string frameID = msg->header.frame_id;
	float range = msg->range;
	// Compare to min to make sure not misfire
	if(range >= msg->min_range && range <= msg->max_range){
	    geometry_msgs::Vector3Stamped rangeVec = geometry_msgs::Vector3Stamped();
	    rangeVec.header = msg->header;
	    rangeVec.vector.x = range;
	    
	    geometry_msgs::Vector3Stamped newRange;
	    listener.transformVector(tframeID, rangeVec, newRange);
	    if(abs(newRange.vector.x) > 0.01){
		forVals.pop_front();
		forVals.push_back(abs(newRange.vector.x));
		float avg = 0.0;
		std::list<float>::iterator itr;
		for(itr = forVals.begin(); itr != forVals.end(); itr++){
		  avg += *itr;
		}
		std_msgs::Float32 out;
		out.data = avg/(float)forVals.size();
		for_pub.publish(out);
	    }
	    if(abs(newRange.vector.y) > 0.01){
		sideVals.pop_front();
		sideVals.push_back(abs(newRange.vector.y));
		float avg = 0.0;
		std::list<float>::iterator itr;
		for(itr = sideVals.begin(); itr != sideVals.end(); itr++){
		  avg += *itr;
		}
		std_msgs::Float32 out;
		out.data = avg/(float)forVals.size();
		side_pub.publish(out);
	    }
	}
	    
    } catch(tf::TransformException& ex){
	ROS_ERROR("Received an exception trying to transform the Rangers: %s", ex.what());
    }
    
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ranger2pointcloud");
  ros::NodeHandle n;
  ros::NodeHandle nh_private("~");
  for_pub = n.advertise<std_msgs::Float32>("bubble/forward", 1);
  side_pub = n.advertise<std_msgs::Float32>("bubble/side", 1);
  
  tf::TransformListener listener(ros::Duration(10));
  
  ros::Subscriber ranger_sub = n.subscribe<sensor_msgs::Range>("sonars", 100, boost::bind(&rangeCallback,boost::ref(listener),_1));
  
  nh_private.param("target_frame", tframeID, std::string("/base_link"));
  nh_private.param("num_samples", N_, 20);
  
  std::list<float> empty(N_);
  
  forVals = empty;
  sideVals = empty;

  ros::spin();

  return 0;
}




