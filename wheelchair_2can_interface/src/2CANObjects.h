#include <stdint.h>

typedef struct _CANFrame // 22 bytes
{
	uint16_t arbid_h;
	uint16_t arbid_l;
	uint16_t len_options;
	
	uint16_t data[9];
}CANFrame;

typedef union _TxCANPacket { //42 bytes (the Words[20] creates 10 bytes of padding)
	struct {
		uint16_t iSig;
		uint16_t iByteLen;

		uint16_t iOptions;
		uint16_t iFrameCnt;
		
		CANFrame CANFrames[1];
		// 10 bytes of padding?
	};
	struct {
		uint16_t Words[20];
		uint16_t iCRC;
	};
} TxCANPacket;


typedef union _EnablePacket // (18 bytes + 6 worth of padding) = 24 bytes
{
	struct
	{
		uint16_t iSig;
		uint16_t iByteLen;

		uint16_t enableState; // upper 8 bits are reserved
		
		// 1 short of padding here?
		uint64_t outputEnables;
		uint16_t sequence;
		// 2 shorts of padding here?
	};
	struct
	{
		uint16_t Words[ (16)/2 ];
		uint16_t iCRC;
	};
} EnablePacket;

typedef union _RxCANPacket { //42 bytes (the Words[20] creates 10 bytes of padding)
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