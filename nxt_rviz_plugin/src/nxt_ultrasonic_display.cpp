#include "nxt_ultrasonic_display.h"
#include "rviz/visualization_manager.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"
#include "rviz/common.h"
#include "rviz/frame_manager.h"
#include "rviz/validate_floats.h"

#include <tf/transform_listener.h>

#include <ogre_tools/shape.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

namespace nxt_rviz_plugin
{
NXTUltrasonicDisplay::NXTUltrasonicDisplay( const std::string& name, rviz::VisualizationManager* manager )
: Display( name, manager )
, color_( 0.1f, 1.0f, 0.0f )
, messages_received_(0)
, tf_filter_(*manager->getTFClient(), "", 10, update_nh_)
{
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  scene_node_->setVisible( false );

  setAlpha( 0.5f );
  setBuffer( 1 );
  Ogre::Vector3 scale( 0, 0, 0);
  rviz::scaleRobotToOgre( scale );

  tf_filter_.connectInput(sub_);
  tf_filter_.registerCallback(boost::bind(&NXTUltrasonicDisplay::incomingMessage, this, _1));
  vis_manager_->getFrameManager()->registerFilterForTransformStatusCheck(tf_filter_, this);
}

NXTUltrasonicDisplay::~NXTUltrasonicDisplay()
{
  unsubscribe();
  clear();
  for (size_t i = 0; i < cones_.size(); i++) {
    delete cones_[i];
  }
}

void NXTUltrasonicDisplay::clear()
{

  messages_received_ = 0;
  setStatus(rviz::status_levels::Warn, "Topic", "No messages received");
}

void NXTUltrasonicDisplay::setTopic( const std::string& topic )
{
  unsubscribe();

  topic_ = topic;

  subscribe();

  propertyChanged(topic_property_);

  causeRender();
}

void NXTUltrasonicDisplay::setColor( const rviz::Color& color )
{
  color_ = color;

  propertyChanged(color_property_);

  processMessage(current_message_);
  causeRender();
}

void NXTUltrasonicDisplay::setBuffer( int buffer )
{
  if(buffer < 1)
    buffer = 1;
  buffer_len_ = buffer;

  propertyChanged(bufferLen_property_);
  
  // new ogre_tools::Shape(ogre_tools::Shape::Cone, vis_manager_->getSceneManager(), scene_node_);

  //processMessage(current_message_);
  //causeRender();
  
  for (size_t i = 0; i < cones_.size(); i++) {
    delete cones_[i];
  }
  cones_.resize(buffer_len_);
  for (size_t i = 0; i < cones_.size(); i++) {
    cones_[i] = new ogre_tools::Shape(ogre_tools::Shape::Cone, vis_manager_->getSceneManager(), scene_node_);
  }
  
}

void NXTUltrasonicDisplay::setAlpha( float alpha )
{
  alpha_ = alpha;

  propertyChanged(alpha_property_);

  processMessage(current_message_);
  causeRender();
}

void NXTUltrasonicDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  sub_.subscribe(update_nh_, topic_, 10);
}

void NXTUltrasonicDisplay::unsubscribe()
{
  sub_.unsubscribe();
}

void NXTUltrasonicDisplay::onEnable()
{
  scene_node_->setVisible( true );
  subscribe();
}

void NXTUltrasonicDisplay::onDisable()
{
  unsubscribe();
  clear();
  scene_node_->setVisible( false );
}

void NXTUltrasonicDisplay::fixedFrameChanged()
{
  clear();

  tf_filter_.setTargetFrame( fixed_frame_ );
}

void NXTUltrasonicDisplay::update(float wall_dt, float ros_dt)
{
}


void NXTUltrasonicDisplay::processMessage(const sonar_joy::Sonar::ConstPtr& msg)
{
  if (!msg)
  {
    return;
  }

  ++messages_received_;
  
  ogre_tools::Shape* cone_ = cones_[messages_received_ % buffer_len_];

  {
    std::stringstream ss;
    ss << messages_received_ << " messages received";
    setStatus(rviz::status_levels::Ok, "Topic", ss.str());
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  geometry_msgs::Pose pose;
  pose.position.z = pose.position.y = 0;
  pose.position.x = msg->range/2;
  pose.orientation.x = 0.707;
  pose.orientation.z = -0.707;
  if (!vis_manager_->getFrameManager()->transform(msg->header.frame_id,msg->header.stamp,pose, position, orientation, true))
  {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(), fixed_frame_.c_str() );
  }

  cone_->setPosition(position);
  cone_->setOrientation(orientation); 
  Ogre::Vector3 scale( sin(msg->beam_angle) * msg->range, sin(msg->beam_angle) * msg->range , msg->range);
  rviz::scaleRobotToOgre( scale );
  cone_->setScale(scale);
  cone_->setColor(color_.r_, color_.g_, color_.b_, alpha_);

}

void NXTUltrasonicDisplay::incomingMessage(const sonar_joy::Sonar::ConstPtr& msg)
{
  processMessage(msg);
}

void NXTUltrasonicDisplay::reset()
{
  Display::reset();
  clear();
}

void NXTUltrasonicDisplay::createProperties()
{
  topic_property_ = property_manager_->createProperty<rviz::ROSTopicStringProperty>( "Topic", property_prefix_, boost::bind( &NXTUltrasonicDisplay::getTopic, this ),
                                                                                boost::bind( &NXTUltrasonicDisplay::setTopic, this, _1 ), parent_category_, this );
  setPropertyHelpText(topic_property_, "nxt_msgs::Ranger topic to subscribe to.");
  rviz::ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
  topic_prop->setMessageType(ros::message_traits::datatype<sonar_joy::Sonar>());
  color_property_ = property_manager_->createProperty<rviz::ColorProperty>( "Color", property_prefix_, boost::bind( &NXTUltrasonicDisplay::getColor, this ),
                                                                      boost::bind( &NXTUltrasonicDisplay::setColor, this, _1 ), parent_category_, this );
  setPropertyHelpText(color_property_, "Color to draw the range.");
  alpha_property_ = property_manager_->createProperty<rviz::FloatProperty>( "Alpha", property_prefix_, boost::bind( &NXTUltrasonicDisplay::getAlpha, this ),
                                                                       boost::bind( &NXTUltrasonicDisplay::setAlpha, this, _1 ), parent_category_, this );
  setPropertyHelpText(alpha_property_, "Amount of transparency to apply to the range.");
  bufferLen_property_ = property_manager_->createProperty<rviz::IntProperty>( "Buffer Length", property_prefix_, boost::bind( &NXTUltrasonicDisplay::getBuffer, this ),
                                                                       boost::bind( &NXTUltrasonicDisplay::setBuffer, this, _1 ), parent_category_, this );
  setPropertyHelpText(bufferLen_property_, "Number of prior measurements to display.");
  
}

const char* NXTUltrasonicDisplay::getDescription()
{
  return "Displays data from a nxt_msgs::Range message as a cone.";
}
} // namespace nxt_rviz_plugin
