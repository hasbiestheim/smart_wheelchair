#include <wheelchair_2can_interface/CANManager.h>

namespace wheelchair_2can_interface {

  CANManager::CANManager(boost::asio::io_service& io_service) : 
    transmitSocket_(io_service), receiveSocket_(io_service, udp::endpoint(udp::v4(), 1218)), receiver_endpoint_(boost::asio::ip::address::from_string("192.168.0.50"), 1217)
  {
    transmitSocket_.open(udp::v4());
    startCommands_ = false;
  }
  
  void CANManager::printArray(uint8_t *array, uint8_t arraySize){
    for(int i = 0; i < arraySize; i++){
      printf("%.2X ",array[i]);
    }
  }
  
  bool CANManager::compareArrays(uint8_t *array1, uint8_t *array2, uint8_t arraySize){
    /*printArray(array1, arraySize);
    std::cout << "|";
    printArray(array2, arraySize);
    std::cout << std::endl;*/
    
    for(int i = 0; i < arraySize; i++){
      if(array1[i] != array2[i]){
	//std::cout << "Arrays not the same @ position:" << i+1 << std::endl;
	return false;
      }
    }
    return true;
  }

  unsigned short CANManager::CheckSum(unsigned short * data, unsigned long byte_len) 
  {
	  unsigned int i=0;
	  unsigned short c=0;
	  c = 0;
	  for (; i<byte_len/2; ++i) {
		  c += data[i];
	  }
	  return ~c + 1;
  }

  /*
  * messageID: The 29-bit CAN message ID in the lsbs.  Bit31 must be set if remote extended frames are desired.
  * data: A pointer to a buffer containing between 0 and 8 bytes to send with the message.  May be NULL if dataSize is 0.
  * dataSize: The number of bytes to send with the message.
  */
  void CANManager::sendMessage(uint32_t messageID, const uint8_t *data, uint8_t dataSize)
  {
    //create a tx frame holding CAN frame to send.
    CANFrame frame = { 0, 0, 0, {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}};
    //len_options holds CAN DLC and options for extended/remote frames.
    //Assume caller wants extended frames
    frame.len_options = ((uint16_t)dataSize)<<8 | (1);
    //caller will set bit31 if remote extended frame is desired.
    if(messageID & 0x80000000) // check bit31
    {
	    frame.len_options |= (2); // bit31 => remote frame
	    messageID &= ~0x80000000; // clear bit31
    }
    //stuff CAN arbitration id.
    frame.arbid_h = messageID>>16;
    frame.arbid_l = messageID & 0xffff;
    //fill data bytes
    if (dataSize > 16)
	    dataSize = 16;
    for (uint8_t i=0; i<dataSize; ++i)
	    frame.data[i] = data[i];

    //copy frame into our comm structure and update checksum
    TxCANPacket TXPacketToSend;
    TXPacketToSend.iSig = 0xAAA1;
    TXPacketToSend.iByteLen = sizeof(TXPacketToSend);
    TXPacketToSend.iOptions = 0;
    TXPacketToSend.iFrameCnt = 1;
    TXPacketToSend.CANFrames[0] = frame;
    TXPacketToSend.iCRC = 0;
    TXPacketToSend.iCRC = CheckSum((unsigned short*)&TXPacketToSend,sizeof(TXPacketToSend));
    
    mutex_.lock();
    transmitSocket_.send_to(boost::asio::buffer((char*)&TXPacketToSend,sizeof(TXPacketToSend)), receiver_endpoint_);
    mutex_.unlock(); 
  }

  /*
  * messageID A reference to be populated with a received 29-bit CAN message ID in the lsbs.
  * data A pointer to a buffer of 16 bytes to be populated with data received with the message.
  * dataSize A reference to be populated with the size of the data received (0 - 16 bytes).
  * DO I WANT TO ADD AN ALREADY CAUGHT UDP PACKET HERE?
  */
  int32_t CANManager::processRXMessage(RxCANPacket messageBytes, uint32_t &messageID, uint8_t *data, uint8_t &dataSize)
  {
	  
	  if(messageBytes.iSig != 0xAAA2)
	  {
		  //ROS_ERROR("RX Message has incorrect iSig.  Not from 2CAN?");
		  return -1;
	  }
	  else
	  {
		  unsigned short expectedCRC = messageBytes.iCRC;
		  unsigned short calculatedCRC = CheckSum(messageBytes.Words,sizeof(messageBytes.Words));
		  if(expectedCRC != calculatedCRC) 
		  {
			  //ROS_ERROR("RX Message has incorrect checksum: expected %d, calculated %d",expectedCRC, calculatedCRC);
			  return -1;
		  }
		  else
		  {
			  //copy received CAN frame into caller's parameters
			  const CANFrame front = messageBytes.CANFrames[0];
			  
			  messageID = ((int32_t)front.arbid_h) << 16 | front.arbid_l; // arbid
			  dataSize = front.len_options>>8; // dlc
			  for(uint8_t i=0;i<dataSize;++i) // data bytes
				  data[2*i] = front.data[i];
			  return 0;
		  }
	  }
	  return -1;
	  
	  //int messageID = ((int32_t)recv_buf[0].CANFrames[0].arbid_h) << 16 | recv_buf[0].CANFrames[0].arbid_l;
  }

  void CANManager::sendHeartBeat(){
    // 2CAN firmware 2.0 and on uses 'robot state' to control LEDs
    //Send the enable datagram to the 2CAN.  This is what controls the LED status.
    EnablePacket enableOut;
    enableOut.enableState = 2;//_robotState;
    enableOut.iSig = 0xAAAC;
    enableOut.iByteLen = sizeof(enableOut);
    enableOut.iCRC = 0;
    enableOut.iCRC = CheckSum((unsigned short*)&enableOut.Words,sizeof(enableOut));
    mutex_.lock();
    transmitSocket_.send_to(boost::asio::buffer((char*)&enableOut,sizeof(enableOut)), receiver_endpoint_);
    mutex_.unlock();
  }
  
  void CANManager::startSender(){
    ros::Rate loop_rate(1000);
    
    // Now send packets at 1000Hz, and once every 20ms, send the 2can heartbeat
    while(ros::ok()){
      for(int i = 0; i < 20; i++){
	// Every time send a command if we have started that process
	if(startCommands_ and false){
	  uint8_t cmd_v = 0x80;
	  uint8_t cmd_w = 0x80;
	  uint8_t cmd_speed = 0xFF;
	  ///3215361: a0 00 80 00 80 00 FF 00
	  uint32_t commandID = 3215361;
	  const uint8_t commandData [8] = {0xa0, 0x00, cmd_v, 0x00, cmd_w, 0x00, cmd_speed, 0x00};
	  sendMessage(commandID, commandData, sizeof(commandData));
	}
	
	// Check if we need to send heartbeat for 2CAN
	if(i == 0){
	  sendHeartBeat();
	}
	
	// Sleep so that we meet the 1000Hz rate
	loop_rate.sleep();
      }
    }
  }
  
  // Need to replace this so that it only listens and processes once
  void CANManager::startListener(){
    while(ros::ok()){
      uint32_t messageID;
      uint8_t dataBytes [16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
      uint8_t dataSize;
      boost::array<RxCANPacket, 1> recv_buf;
      
      receiveSocket_.receive_from(boost::asio::buffer(recv_buf), sender_endpoint_);
      
      int result = processRXMessage(recv_buf[0], messageID, dataBytes, dataSize);
      
      if(result == 0){ // Time to do something according to the state machine
	// Startup
	if(messageID == 32928 && compareArrays(dataBytes,e32928,8)){
	  // Send 36865
	  sendMessage(36865, r36865, sizeof(r36865));
	} else if(messageID == 65696 && compareArrays(dataBytes,e65696_25,8)){
	  // Send 3231745
	  sendMessage(3231745, r3231745, sizeof(r3231745));
	  // Send 13701121
	  sendMessage(13701121, r13701121, sizeof(r13701121));
	} else if(messageID == 65696 && compareArrays(dataBytes,e65696_01,8)){
	  // Send 69633_aa
	  sendMessage(69633, r69633_aa, sizeof(r69633_aa));
	} else if(messageID == 13697184 && compareArrays(dataBytes,e13697184,8)){
	  // Send 69633_55
	  sendMessage(69633, r69633_55, sizeof(r69633_55));
	} else if(messageID == 3227808 && compareArrays(dataBytes,e3227808,8)){
	  // Send r3215361_a0
	  sendMessage(3215361, r3215361_a0, sizeof(r3215361_a0));
	}
	// Heartbeat
	else if(messageID == 69792 && compareArrays(dataBytes,e69792_55,8)){
	  // Send 69633_55
	  sendMessage(69633, r69633_55, sizeof(r69633_55));
	} else if(messageID == 69792 && compareArrays(dataBytes,e69792_aa,8)){
	  // Send 69633_aa
	  sendMessage(69633, r69633_aa, sizeof(r69633_aa));
	}
	// Trigger
	 else if(messageID == 3211424 && compareArrays(dataBytes,e3211424,8)){
	  // Allow commands to start sending at 1000Hz
	  startCommands_ = true;
	}
      }
    }
  }
};