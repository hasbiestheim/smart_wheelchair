#include "2CANObjects.h"

#include <string.h> // For memset???

TxCANPacket TXPacketToSend;

unsigned short CheckSum(unsigned short * data, unsigned long byte_len) 
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
int32_t sendMessage(uint32_t messageID, const uint8_t *data, uint8_t dataSize)
{
  //create a tx frame holding CAN frame to send.
  CANFrame frame = { 0, 0, 0, {0} };
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
  if (dataSize > 8)
	  dataSize = 8;
  for (uint8_t i=0; i<dataSize; ++i)
	  frame.data[i] = data[i];

  //copy frame into our comm structure and update checksum
  TXPacketToSend.CANFrames[0] = frame;
  TXPacketToSend.iOptions = 0;
  TXPacketToSend.iFrameCnt = 1;
  TXPacketToSend.iCRC = 0;
  TXPacketToSend.iCRC = CheckSum((unsigned short*)&TXPacketToSend,sizeof(TXPacketToSend));
  
  //send 2CAN comm frame
  return 0; // CANTxSocket->Send((char*)&TXPacketToSend, sizeof(TXPacketToSend));
}

/*
 * messageID A reference to be populated with a received 29-bit CAN message ID in the lsbs.
 * data A pointer to a buffer of 8 bytes to be populated with data received with the message.
 * dataSize A reference to be populated with the size of the data received (0 - 8 bytes).
 * DO I WANT TO ADD AN ALREADY CAUGHT UDP PACKET HERE?
 */
int32_t receiveMessage(uint32_t &messageID, uint8_t *data, uint8_t &dataSize)
{
	char m_ucRx[1500]; // data cache for recieved UDP datagrams
	int len = sizeof(m_ucRx);

	//read UDP socket
	len = 0; //m_p2CANRxSocket->Read((char*)m_ucRx, len);
	
	//cast received data to 2CAN communication structure
	RxCANPacket * pPack = (RxCANPacket * )m_ucRx;

	uint16_t header = pPack->iSig;

	if(len == 0)
	{
		return -1;
	}
	else if(len != sizeof(RxCANPacket))
	{
		return -1;
	}
	else if(header != 0xAAA2)
	{
		return -1;
	}
	else
	{
		//cast received data to the CAN RX communication structure
		RxCANPacket *pRxCANPacket = (RxCANPacket * )m_ucRx;

		if(CheckSum((unsigned short*)m_ucRx,sizeof(RxCANPacket))) 
		{
			//bad checksum
			return -1;
		}
		else
		{
			//copy received CAN frame into caller's parameters
			const CANFrame & front = pRxCANPacket->CANFrames[0];
			
			messageID = ((int32_t)front.arbid_h) << 16 | front.arbid_l; // arbid
			dataSize = front.len_options>>8; // dlc
			for(uint8_t i=0;i<dataSize;++i) // data bytes
				data[i] = front.data[i];
			return 0;
		}
	}
	return -1;
}

void ProcessHeartBeat()
{

  // 2CAN firmware 2.0 and on uses 'robot state' to control LEDs
  //Send the enable datagram to the 2CAN.  This is what controls the LED status.
  EnablePacket enableOut;
  memset((void*)&enableOut,0,sizeof(enableOut));
  enableOut.enableState = 2;//_robotState;
  enableOut.iSig = 0xAAAC;
  enableOut.iByteLen = sizeof(enableOut);
  enableOut.iCRC = 0;
  enableOut.iCRC = CheckSum(enableOut.Words,sizeof(enableOut));
  /*if(m_p2CANTxSocket->Send((char*)&enableOut, sizeof(enableOut)) == 0)
  {
  }*/
	
  //we want to transmit if 50 ms have gone by and there has been no traffic
  //to maintain LED status.

}

int main(int argc, char *argv[]) {
  //CANTxSocket = new TxUDPSocket(m_sIP,1217,0);
  //CANRxSocket = new RxUDPSocket(m_sIP,1218,0);
  
  // Concerned about memory leaks
  
  // Need to figure out how to create sockets and then use them
  return 0;
}