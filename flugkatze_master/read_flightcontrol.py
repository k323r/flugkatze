#!/usr/bin/python

import serial, time

# create a serial object and connect it to /dev/ttyUSB0

flugkatze_flightcontrol = serial.Serial();
flugkatze_flightcontrol.port = "/dev/ttyUSB0"
flugkatze_flightcontrol.baudrate = 9600
flugkatze_flightcontrol.bytesize = serial.EIGHTBITS
flugkatze_flightcontrol.parity = serial.PARITY_NONE
flugkatze_flightcontrol.stopbits = serial.STOPBITS_ONE
flugkatze_flightcontrol.timeout = None


try:
	flugkatze_flightcontrol.open()
except Exception, e:
	print "error opening serial port /dev/ttyUSB0: ", str(e)
	exit()

if flugkatze_flightcontrol.isOpen():
	flugkatze_flightcontrol.flushInput()
	flugkatze_flightcontrol.flushOutput()

	while True:
		msg = flugkatze_flightcontrol.readline()
		print "flugkatze_flightcontrol: " + msg


	    
