#!/usr/bin/env python

import socket
import time
import struct

class FakeAttendant:    
  def __init__(self, gX):
    self.outgoingUDP = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.address = ("192.168.0.50", 1217)
    self.tx_can_packet_fmt_string = "<HHHH22s10xH"
    self.rx_can_packet_fmt_string = "<HHHH22s10xH"
    self.can_frame_fmt_string = "<HHH16s"
    self.atdKeepAliveStart = 0xAA
    self.heartCount = 0
    
    while True:
      self.sendHeartBeatPacket()
      self.testTXPacket()
      #loop at 50ms
      time.sleep(0.02)

    outgoingUDP.close()


  def calcCRC(self,iSig, iByteLen, robot_state, i):
    s = iSig + iByteLen + robot_state + i
    return (~s % 2**16 +1)

  def calcCRCNew(self, values):
    s = 0
    for elem in values:
      if(type(elem) == str):
	elem = struct.unpack("<11H", elem)
	for tup in elem:
	  s += tup
      else:
	while(elem != 0):
	  s += elem & 0xFFFF
	  elem = elem >> 16
    return (~s % 2**16 +1)
    
  """
 
  unsigned short C2CAN::CheckSum(unsigned short * data, unsigned long byte_len) 
  {
	  unsigned int i=0;
	  unsigned short c=0;
	  c = 0;
	  for (; i<byte_len/2; ++i) {
		  c += data[i];
	  }
	  return ~c + 1;
  }


  typedef union _STo2CAN_TxCANPacket { //44 bytes
	  struct {
		  uint16_t iSig;
		  uint16_t iByteLen;

		  uint16_t iOptions;
		  uint16_t iFrameCnt;
		  STo2CAN_CANFrame CANFrames[1];
	  };
	  struct {
		  uint16_t Words[20];
		  uint16_t iCRC;
	  };
  } STo2CAN_TxCANPacket;
 
  """
	  
  """
  typedef union _STo2CAN_RxCANPacket {
	  struct {
		  uint16_t iSig;
		  uint16_t iByteLen;

		  uint16_t iOptions;
		  uint16_t iFrameCnt;
		  STo2CAN_CANFrame CANFrames[1];
	  };
	  struct {
		  uint16_t Words[20];
		  uint16_t iCRC;
	  };
  } STo2CAN_RxCANPacket;
  """


  """
  typedef struct _STo2CAN_CANFrame // 22 bytes
  {
	  uint16_t arbid_h;
	  uint16_t arbid_l;

	  #define STo2CAN_CANFrameOption_ExtendedID	(1)
	  #define STo2CAN_CANFrameOption_Remote		(2)
	  #define STo2CAN_CANFrameOption_BaseFrame	(4)
	  #define STo2CAN_CANFrameOption_Transmitted	(8)
	  uint16_t len_options;

	  uint16_t data[8];
  }STo2CAN_CANFrame;
  """
  

  # These messages only appear when the attendant is plugged in and only at the begining.
  def sendInitializationPackets(self):
    # Need to send:
    #36865: a0 00 51 00 27 [11(00)]
    #13697184: 81 00 80 00 aa 00 55 [9(00)]
    #13701121: a0 00 02 00 03 00 00 00 02 [7(00)]
    msg = 1

  # The remote attendant appears to need these alternating messages at one message per second.
  
  def sendKeepAlive(self):
    if(atdKeepAliveStart == 0xAA):
      # send 69633: ff 00 80 aa 00 55 [9(00)]
      atdKeepAliveStart == 0x55
    else:
      # send 69633: ff 00 80 55 00 aa [9(00)]
      atdKeepAliveStart == 0xAA
      
  # Try to send a packet!
  def testTXPacket(self):
    arbid_h = 0x0000
    arbid_l = 0xBEEF
    len_options = 0x1001 # Set first byte to length of data (16); Set last byte to 1 for 29 bit frames
    data = "\xa0005100270000000000000000000000"
    
    can_frame = struct.pack(self.can_frame_fmt_string, arbid_h, arbid_l, len_options, data)
    
    iSig = 0xAAA1 # As defined for 2CAN for a TX Packet
    iByteLen = 0x002C # Guessing 44 bytes  sizeof(STo2CAN_TxCANPacket )
    iOptions = 0x0000
    iFrameCnt = 0x0001
    
    values = [iSig, iByteLen, iOptions, iFrameCnt, can_frame];
    
    iCRC = self.calcCRCNew(values)
    
    msg = struct.pack(self.tx_can_packet_fmt_string, iSig, iByteLen, iOptions, iFrameCnt, can_frame, iCRC)
    self.outgoingUDP.sendto(msg, self.address)

  # This keeps the 2CAN communicating to the last computer to ping it
  def sendHeartBeatPacket(self):
    heartbeat_format = "<HHHQHH"
    robot_state = 0x0000
    iSig = 0xAAAC
    iByteLen = 0x0012
    crc = self.calcCRC(iSig, iByteLen, robot_state, self.heartCount)
    msg = struct.pack(heartbeat_format, iSig, iByteLen, robot_state, 0, self.heartCount, crc)
    self.heartCount += 1
    self.heartCount = self.heartCount % 2**16
    self.outgoingUDP.sendto(msg, self.address)

if __name__ == '__main__':
  faker = FakeAttendant(None)