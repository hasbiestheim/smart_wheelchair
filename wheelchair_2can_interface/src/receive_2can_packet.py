#!/usr/bin/env python

import socket
import struct
import curses
import time

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
rx_can_packet_fmt_string = "<HHHH22s10xH"

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
can_frame_fmt_string = "<HHH16s"

def make_pretty_hex(raw_string):
    return raw_string

def print_arbid_dict(std_scr, can_dict):
    x = 0
    y = 0
    std_scr.addstr(y, x, "%010s: %015s: %50s" % ("Time (ms)", "ARBID", "Data"))
    now = time.time()
    for key in sorted(can_dict):
        hexes =  ['%02x' % ord(c) for c in can_dict[key][0]]
        hex_string = " ".join(hexes)
        y += 1
        old_time = can_dict[key][1]
        time_diff = (now - old_time) * 1000.
        std_scr.addstr(y, x, "%010.2f: %015d: %50s" % (time_diff, key, hex_string))

    std_scr.refresh()

def main(std_scr):

    incomingUDP = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    incomingUDP.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    incomingUDP.bind(('', 1218))

    can_dict = {}

    while True:
        data, addr = incomingUDP.recvfrom(1500)
        num_bytes_received = len(data)
        #struct.unpack(rx_can_packet_fmt_string, 
        if num_bytes_received > 0:
            if num_bytes_received == struct.calcsize(rx_can_packet_fmt_string):
                #SIG_RX_CAN_FRAMES = 0xAAA2
                rx_frame = struct.unpack(rx_can_packet_fmt_string, data)
                if rx_frame[0] == 0xAAA2:
                    #Good packet... passed lots of checks
                    can_frame_bytes = rx_frame[4]
                    can_frame = struct.unpack(can_frame_fmt_string, can_frame_bytes)
                    arbid = (can_frame[0] << 16) + can_frame[1]
                    can_dict[arbid] = (make_pretty_hex(can_frame[3]), time.time())
                else:
                    print "Can Frame Header wrong"
            else:
                print "Wrong Number of bytes received"
        else:
            print "0 bytes received"

        #Print out the arbid dict
        print_arbid_dict(std_scr, can_dict)

    incomingUDP.close()

if __name__ == "__main__":
    curses.wrapper(main)
