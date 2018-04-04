#!/usr/bin/python3

# for reading from serial, non-blocking
from multiprocessing import Process, Queue
import signal       # for capturing ctrl-c otherwise zombies might spawn

import serial       # for serial
import struct       # used to transform the binary data
import os           # for checking paths and the like

# for live plots
from DynamicPlotter import DynamicPlotter

# misc stuff

import random
import time
import math

# function carried out by the worker thread
def readFromSerial(port, queue, format_str, channel):
    counter = 0
    print("starting to read from serial")
    size_struct = struct.calcsize(format_str)
    while True:
        try:
            Byte = port.read(1).decode("utf-8")
        except:
            continue
        if Byte == 'S':
            data = port.read(size_struct)
            try:
                Byte = port.read(1).decode("utf-8")
            except:
                continue
            if Byte == 'E':
                msg = str(struct.unpack(format_str, data)).replace("(", "").replace(")", "").split(",")
                queue.put(msg[channel])
                if counter >= 100:
                    print (msg)
                    counter = 0
                counter += 1



# open serial port

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='read data from a serial connection and plot it')
    parser.add_argument('--port', required=True, dest="port")
    parser.add_argument('--baudrate' required=True, dest="baudrate")
    args = parser.parse_args()

    # create a serial object
    sPort = serial.Serial();
    sPort.port = args.port
    sPort.baudrate = args.baudrate
    sPort.bytesize = serial.EIGHTBITS
    sPort.parity = serial.PARITY_NONE
    sPort.stopbits = serial.STOPBITS_ONE
    sPort.timeout = None

    try:
        sPort.open()
    except Exception as e:
        print("error opening serial port /dev/ttyUSB0: ", str(e))
        exit()
    # flush the port
    if sPort.isOpen():
        sPort.flushInput()
        sPort.flushOutput()

    q = Queue()     # create queue for interprocesscommunication

    sigIntHandler = signal.signal(signal.SIGINT, signal.SIG_IGN)
    p = Process(target=readFromSerial, args=(sPort, q, "ffffff", 0))
    signal.signal(signal.SIGINT, original_sigint_handler)
    p.daemon = True     # in order to kill zombies (actually not even phrasing)
    m = DynamicPlotter(q, sampleinterval=0.005, timewindow=5.)
    try:
        p.start()
        m.run()
    except (KeyboardInterrupt, SystemExit):
        print("exit, bye!")
        p.terminate()       # terminate the thread
        q.close()           # close the queue
        p.join()
    else:
        print("unexpected exit, bye")
        p.terminate()       # terminate the thread
        q.close()           # close the queue
        p.join()


