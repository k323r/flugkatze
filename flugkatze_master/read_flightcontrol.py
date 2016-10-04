#!/usr/bin/python

import serial, time, struct

# create a serial object and connect it to /dev/ttyUSB0

flugkatze_flightcontrol = serial.Serial();
flugkatze_flightcontrol.port = "/dev/ttyUSB0"
flugkatze_flightcontrol.baudrate = 115200
flugkatze_flightcontrol.bytesize = serial.EIGHTBITS
flugkatze_flightcontrol.parity = serial.PARITY_NONE
flugkatze_flightcontrol.stopbits = serial.STOPBITS_ONE
flugkatze_flightcontrol.timeout = None

try:
    flugkatze_flightcontrol.open()
except Exception, e:
    print "error opening serial port /dev/ttyUSB0: ", str(e)
    exit()

log = open('./log.readFlightcontrol', 'w+')

if flugkatze_flightcontrol.isOpen():
    flugkatze_flightcontrol.flushInput()
    flugkatze_flightcontrol.flushOutput()

'''
struct Flight_data {
    float ax;
    float ay;
    float az;
    
    float temp;
    
    float gx;
    float gy;
    float gz;
    
    int throttle;
    int roll;
    int pitch;
    int yaw;
    
    float roll_setpoint;
    float pitch_setpoint;
    float yaw_setpoint;
    
} flight_data;
'''

FORMAT_STR = "fffffffhhhhfff"

size_struct = struct.calcsize(FORMAT_STR)
print size_struct


    # for binary data (later use)
while True:
    Byte = flugkatze_flightcontrol.read(1)
    if Byte == 'S':
        data = flugkatze_flightcontrol.read(size_struct)
        Byte = flugkatze_flightcontrol.read(1)
        if Byte == 'E':
            msg = str(struct.unpack(FORMAT_STR, data))
            print (msg)
            log.writelines(msg + "\n")
#    while True:
#        msg = flugkatze_flightcontrol.readline()
#        print "flugkatze_flightcontrol: " + msg
#        log.writelines(msg)


 
