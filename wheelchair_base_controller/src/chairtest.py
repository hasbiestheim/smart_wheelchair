import serial

port = serial.Serial("/dev/ttyUSB0", 115200)
while(1 > 0):
    cmdVx = 255
    cmdWz = 255
    throttle = 255 
    command_string = "%d %d %d\r" % (cmdVx,cmdWz,throttle)
    port.write(command_string)

port.close()
