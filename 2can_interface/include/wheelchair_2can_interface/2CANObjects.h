#include <stdint.h>

#define v_ctr 0x80
#define w_ctr 0x80
#define speed_def 0xFF

namespace wheelchair_2can_interface {

  typedef struct _CANFrame // 22 bytes -> I expanded data to include 8 of the 10 normal bytes of padding
  {
	  uint16_t arbid_h;
	  uint16_t arbid_l;
	  uint16_t len_options;
	  
	  uint16_t data[16];
  }CANFrame;

  typedef union _TxCANPacket { //42 bytes (the Words[20] creates 2 extra bytes of padding)
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

  typedef union _RxCANPacket { //42 bytes (the Words[20] creates 2 bytes of padding)
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

  // Response structures to send
  /// 36865: a0 00 51 00 27 00 00 00 00 00 00 00 00 00 00 00
  const uint8_t r36865 [8] = {0xa0, 0x00, 0x51, 0x00, 0x27, 0x00, 0x00, 0x00};
  /// 69633: ff 00 80 00 [55] 00 [aa] 00 00 00 00 00 00 00 00 00
  const uint8_t r69633_55 [8] = {0xff, 0x00, 0x80, 0x00, 0x55, 0x00, 0xaa, 0x00};
  const uint8_t r69633_aa [8] = {0xff, 0x00, 0x80, 0x00, 0xaa, 0x00, 0x55, 0x00};
  /// 13701121: a0 00 02 00 03 00 00 00 02 00 00 00 00 00 00 00
  const uint8_t r13701121 [10] = {0xa0, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00, 0x00, 0x02, 0x00};
  /// 3215361: a0 00 [v_ctr] 00 [w_ctr] 00 [spd_cmd] 00 00 00 00 00 00 00 00 00
  const uint8_t r3215361_a0 [8] = {0xa0, 0x00, v_ctr, 0x00, w_ctr, 0x00, speed_def, 0x00};
  /// 3231745: a0 00 01 00 00 00 aa 00 00 00 00 00 00 00 00 00
  const uint8_t r3231745 [8] = {0xa0, 0x00, 0x01, 0x00, 0x00, 0x00, 0xaa, 0x00};
  
  // Packet structes to expect
  /// 32928: 01 00 01 00 02 00 00 00 00 00 00 00 00 00 00 00
  uint8_t e32928 [16] = {0x01, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  /// 69792: ff 00 80 00 aa 00 55 00 00 00 00 00 00 00 00 00
  uint8_t e69792_55 [16] = {0xff, 0x00, 0x80, 0x00, 0x55, 0x00, 0xaa, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t e69792_aa [16] = {0xff, 0x00, 0x80, 0x00, 0xaa, 0x00, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  
  /// 65696: 01 00 01 00 02 00 [00] 00 00 00 00 00 00 00 00 00
  uint8_t e65696_01 [16] = {0x01, 0x00, 0x80, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t e65696_25 [16] = {0x25, 0x00, 0x80, 0x00, 0x64, 0x00, 0xaa, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  /// 13697184: 01 00 v_ctr 00 w_ctr 00 spd_cmd 00 00 00 00 00 00 00 00 00
  uint8_t e13697184 [16] = {0x01, 0x00, v_ctr, 0x00, w_ctr, 0x00, speed_def, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  /// 3211424: 01 00 0a 00 [aa] 00 [55] 00 00 00 00 00 00 00 00 00
  uint8_t e3211424 [16] = {0x01, 0x00, 0x0a, 0x00, 0xaa, 0x00, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  /// 3227808: 01 00 00 00 aa 00 55 00 00 00 00 00 00 00 00 00
  uint8_t e3227808 [16] = {0x01, 0x00, 0x00, 0x00, 0xaa, 0x00, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
};