#include "ros/ros.h"
#include "2CANObjects.h"
#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

using boost::asio::ip::udp;

namespace wheelchair_2can_interface {
  class CANManager {
    public:
      CANManager(boost::asio::io_service& io_service);
      void startSender();
      void startListener();
      
    private:
      udp::socket transmitSocket_;
      udp::socket receiveSocket_;
      udp::endpoint receiver_endpoint_;
      udp::endpoint sender_endpoint_;
      boost::mutex mutex_;
      bool startCommands_;
      
      void printArray(uint8_t *array, uint8_t arraySize);
      bool compareArrays(uint8_t *array1, uint8_t *array2, uint8_t arraySize);
      void sendHeartBeat();
      unsigned short CheckSum(unsigned short * data, unsigned long byte_len);
      void sendMessage(uint32_t messageID, const uint8_t *data, uint8_t dataSize);
      int32_t processRXMessage(RxCANPacket messageBytes, uint32_t &messageID, uint8_t *data, uint8_t &dataSize);
  };
};