#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <octoflex/OctoFlexConfig.h>
#include <geometry_msgs/Twist.h>
#include <octocostmap/costmap_3d.h>
#include <tf/transform_listener.h>

bool checkFullVolume_;
float simTime_;
int numSteps_;
float length_;
float width_;
float height_;
float originOffsetX_;
float originOffsetY_;
float originOffsetZ_;
float resolution_;
float rate_;
float visPause_;

geometry_msgs::Twist cmd_;
boost::shared_ptr<octocostmap::Costmap3D> costmap_;
ros::Timer loopTimer_;

void allowCommand(const tf::TransformListener& listener){
  float dt = simTime_/(float)numSteps_;
  // Offset current pose to the front left corner of the vehicle outline
  geometry_msgs::PoseStamped currentPose;
  currentPose.header.frame_id = "/base_link";
  currentPose.pose.position.x = -length_/2.0 - originOffsetX_;
  currentPose.pose.position.y = -width_/2.0 - originOffsetY_;
  currentPose.pose.position.z = originOffsetZ_;
  currentPose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  // Transform this into the map frame?
  //tf_listener_.transformPose("/odom", ros::Time(0), currentPose, "/base_link", currentPose);
  
  for(int i = 1; i <= numSteps_; i++){
    // Propigate curvature addition through time
    float th = tf::getYaw(currentPose.pose.orientation);
    float v = cmd_.linear.x;
    float w = cmd_.angular.z;
    
    float dx;
    float dy;
    float dTh = w*i*dt;
    float thNew = th + dTh;
    if(fabs(w) < 0.001){ // Straight line
      dx = v*i*dt*cos(th);
      dy = v*i*dt*sin(th);
    } else {
      float r = v/w;
      dx = -r*sin(th)+r*sin(thNew);
      dy = r*cos(th)-r*cos(thNew);
    }
    // Check this position for collisions
    geometry_msgs::PoseStamped origin;
    tf::Stamped<tf::Pose > tf_origin;
    tf::poseStampedMsgToTF(currentPose, tf_origin);
    tf::Pose shift_amount(tf::createQuaternionFromYaw(dTh), tf::Vector3(dx, dy, 0.0));
    tf_origin.setData(tf_origin * shift_amount);
    tf::poseStampedTFToMsg(tf_origin, origin);
    std::cout << "x: " << origin.pose.position.x << " y: " << origin.pose.position.y << " th: " << tf::getYaw(origin.pose.orientation) << std::endl;
    try{
      bool collision_detected = costmap_->checkRectangularPrismBase(origin, width_, height_, length_, resolution_, checkFullVolume_);
      if(collision_detected){
	ROS_DEBUG("Collision detected, do not execute command!");
      }
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
    }
    ros::Duration(visPause_).sleep();
  }
  // Did not detect a collision, so assume this control is safe!
}

void twistCallback(const geometry_msgs::Twist::ConstPtr& msg){
  cmd_ = *msg;
}

void callback(octoflex::OctoFlexConfig &config, uint32_t level)
{
  checkFullVolume_ = config.checkFullVolume;
  simTime_ = config.forwardTime;
  numSteps_ = config.numberOfSteps;
  length_ = config.length;
  width_ = config.width;
  height_ = config.height;
  originOffsetX_ = config.originOffsetX;
  originOffsetY_ = config.originOffsetY;
  originOffsetZ_ = config.originOffsetZ;
  resolution_ = config.resolution;
  rate_ = config.rate;
  visPause_ = config.visualizationPause;
  loopTimer_.setPeriod(ros::Duration(1.0/rate_));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "octoflex");
  ros::NodeHandle n;
  tf::TransformListener tf_listener;
  //joyout_pub = n.advertise<joy::Joy>("joy_output", 1);
  
  dynamic_reconfigure::Server<octoflex::OctoFlexConfig> srv;
  dynamic_reconfigure::Server<octoflex::OctoFlexConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  srv.setCallback(f);
  
  costmap_ = boost::shared_ptr<octocostmap::Costmap3D>(new octocostmap::Costmap3D("octocostmap", tf_listener));

  ros::Subscriber cmd_sub = n.subscribe("cmd_vel", 100, twistCallback);
  
  //ros::Subscriber front_sub = n.subscribe("/bubble/forward", 100, frontFloatCallback);
  
  //ros::Subscriber side_sub = n.subscribe("/bubble/side", 100, sideFloatCallback);
  
  loopTimer_ = n.createTimer(ros::Duration(1.0/rate_), boost::bind(&allowCommand,boost::ref(tf_listener)));
  
  ros::spin();

  return 0;
}