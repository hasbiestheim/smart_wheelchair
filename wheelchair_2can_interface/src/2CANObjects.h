#include <stdint.h>

typedef struct CANFrame // 22 bytes
{
	uint16_t arbid_h;
	uint16_t arbid_l;
	uint16_t len_options;
	
	uint16_t data[8];
}CANFrame;

typedef union TxCANPacket { // 44 bytes
	struct {
		uint16_t iSig;
		uint16_t iByteLen;

		uint16_t iOptions;
		uint16_t iFrameCnt;
		CANFrame CANFrames[1];
	};
	struct {
		uint16_t Words[20];
		uint16_t iCRC;
	};
} TxCANPacket;


typedef union EnablePacket // 24 bytes?
{
	struct
	{
		uint16_t iSig;
		uint16_t iByteLen;

		uint16_t enableState; // upper 8 bits are reserved
		
		uint64_t outputEnables;
		uint16_t sequence;
	};
	struct
	{
		uint16_t Words[ (16)/2 ];
		uint16_t iCRC;
	};
} EnablePacket;

typedef union RxCANPacket { // 44 bytes
	struct {
		uint16_t iSig;
		uint16_t iByteLen;

		uint16_t iOptions;
		uint16_t iFrameCnt;
		CANFrame CANFrames[1];
	};
	struct {
		uint16_t Words[20];
		uint16_t iCRC;
	};
} RxCANPacket;