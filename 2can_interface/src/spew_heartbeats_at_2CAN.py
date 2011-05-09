#!/usr/bin/env python

import socket
import time
import struct

outgoingUDP = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
address = ("192.168.0.50", 1217)

def calcCRC(iSig, iByteLen, robot_state, i):
        s = iSig + iByteLen + robot_state + i
        return (~s % 2**16 +1)

def makeHeartBeatPacket(i):
    #iSig
    #iByteLen
    #enableState
    #outputEnables
    #sequence
    #iCRC
    heartbeat_format = "<HHHQHH"
    robot_state = 0x0000
    iSig = 0xAAAC
    iByteLen = 0x0012
    crc = calcCRC(iSig, iByteLen, robot_state, i)
    return struct.pack(heartbeat_format, iSig, iByteLen, robot_state, 0, i, crc)


i = 0
while True:
    p = makeHeartBeatPacket(i)
    outgoingUDP.sendto(p, address)
    i += 1
    i = i % 2**16
    #something about 50ms
    time.sleep(0.02)

outgoingUDP.close()
