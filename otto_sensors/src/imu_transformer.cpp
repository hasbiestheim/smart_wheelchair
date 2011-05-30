#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/transform_listener.h>

ros::Publisher out_pub;

std::string tframeID;

void imuCallback(const tf::TransformListener& listener, const sensor_msgs::Imu::ConstPtr& msg)
{
    try{
	// Transforms everything but the covariances right now into target frame
	
	listener.waitForTransform(tframeID, msg->header.frame_id, ros::Time::now(), ros::Duration(1.0));
	
	sensor_msgs::Imu newMsg = sensor_msgs::Imu();
	newMsg.header = msg->header;
	
	geometry_msgs::QuaternionStamped quat = geometry_msgs::QuaternionStamped();
	quat.header = msg->header;
	quat.quaternion = msg->orientation;
	
	geometry_msgs::QuaternionStamped newQuat;
	listener.transformQuaternion(tframeID, quat, newQuat);
	newMsg.orientation = newQuat.quaternion;
	
	geometry_msgs::Vector3Stamped angv = geometry_msgs::Vector3Stamped();
	angv.header = msg->header;
	angv.vector = msg->angular_velocity;
	
	geometry_msgs::Vector3Stamped newOmega;
	listener.transformVector(tframeID, angv, newOmega);
	newMsg.angular_velocity = newOmega.vector;
	
	geometry_msgs::Vector3Stamped accel = geometry_msgs::Vector3Stamped();
	accel.header = msg->header;
	accel.vector = msg->linear_acceleration;
	
	geometry_msgs::Vector3Stamped newAccel;
	listener.transformVector(tframeID, accel, newAccel);
	newMsg.linear_acceleration = newAccel.vector;
	
	out_pub.publish(newMsg);
	
    } catch(tf::TransformException& ex){
	ROS_ERROR("Received an exception trying to transform the IMU: %s", ex.what());
    }
	
}
	
int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_transformer");
  
  ros::NodeHandle n;

  tf::TransformListener listener(ros::Duration(10));
  
  out_pub = n.advertise<sensor_msgs::Imu>("imu_transformed", 1);

  ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu>("imu", 100, boost::bind(&imuCallback,boost::ref(listener),_1));
  
  n.param<std::string>("target_frame", tframeID, "/base_link");

  ros::spin();

  return 0;
}




