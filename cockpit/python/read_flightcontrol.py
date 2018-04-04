#!/usr/bin/python

import serial, time, struct, os

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

if not os.path.isdir("./logs"):
	try:
		os.mkdir("./logs")
	except:
		print "could not create directory 'logs'"
		exit()

## create timestamp
timestring = time.strftime("%Y%m%d-%H%M%S")

# log_name = './logs/' + timestring + '.log'
log_name = './logs/telemetry.log'

log = open(log_name, 'w+')

log.writelines("# ax ay az gx gy gz\n")

if flugkatze_flightcontrol.isOpen():
    flugkatze_flightcontrol.flushInput()
    flugkatze_flightcontrol.flushOutput()

'''
struct Flight_data {
    float ax;
    float ay;
    float az;
    
    float gx;
    float gy;
    float gz;
    
} flight_data;
'''

FORMAT_STR = "ffffff"

size_struct = struct.calcsize(FORMAT_STR)
print size_struct

counter = 0

    # for binary data (later use)
while True:
    Byte = flugkatze_flightcontrol.read(1)
    if Byte == 'S':
        data = flugkatze_flightcontrol.read(size_struct)
        Byte = flugkatze_flightcontrol.read(1)
        if Byte == 'E':
            msg = str(struct.unpack(FORMAT_STR, data)).replace("(", "").replace(")", "")
            counter += 1
            if counter > 3:
                print (msg)
                counter = 0
                log.writelines(msg + "\n")
#    while True:
#        msg = flugkatze_flightcontrol.readline()
#        print "flugkatze_flightcontrol: " + msg
#        log.writelines(msg)


 
