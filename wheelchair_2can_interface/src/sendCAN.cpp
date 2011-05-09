#include "ros/ros.h"
#include <wheelchair_2can_interface/CANManager.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

int main(int argc, char *argv[]) { 
  ros::init(argc, argv, "send2CANMessages");
  ros::NodeHandle n;
  boost::asio::io_service io_service;
  wheelchair_2can_interface::CANManager ottoAttendant(io_service);
  io_service.run();
  
  // Need to add functions to keep the loops in here while using ros::ok() so that CANManager doesn't depend on ros
  boost::thread RXt(boost::bind(&wheelchair_2can_interface::CANManager::startListener,&ottoAttendant));
  boost::thread TXt(boost::bind(&wheelchair_2can_interface::CANManager::startSender,&ottoAttendant));
  
  RXt.join();
  TXt.join();
  
  return 0;
}