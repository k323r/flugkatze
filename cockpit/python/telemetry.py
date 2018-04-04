#!/usr/bin/python3

# for reading from serial, non-blocking
from multiprocessing import Process, Queue
import signal       # for capturing ctrl-c otherwise zombies might spawn

import serial       # for serial
import struct       # used to transform the binary data

# for live plots
from DynamicPlotter import DynamicPlotter

# misc stuff
import argparse

# function carried out by the worker thread
def readFromSerial(port, queue, format_str, channel):
    counter = 0
    print("starting to read from serial")
    size_struct = struct.calcsize(format_str)
    while True:
        try:
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
        except KeyboardInterrupt:
            break

# plot runner wrapper
def startPlotter(plot):
    try:
        plot.run()
    except KeyboardInterrupt:
        plot.app.closeAllWindows()
        return
    else:
        return

# open serial port

if __name__ == "__main__":

    # create parser object and parse the command line 
    parser = argparse.ArgumentParser(description='read data from a serial connection and plot it')
    parser.add_argument('--port', required=True, dest="port")
    parser.add_argument('--baudrate', required=True, dest="baudrate")
    parser.add_argument('--channel', required=True, dest="channel", default=0, type=int)
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

    queue = Queue()     # create queue for interprocesscommunication
    dPlot = DynamicPlotter(queue, sampleinterval=0.005, timewindow=1.)  # plotting object

    sigIntHandler = signal.signal(signal.SIGINT, signal.SIG_IGN)
    p_read = Process(target=readFromSerial, args=(sPort, queue, "ffffff", args.channel))
    signal.signal(signal.SIGINT, sigIntHandler)
    p_read.daemon = True     # in order to kill zombies (actually not even phrasing)
    try:
        p_read.start()
        dPlot.run()
    except (KeyboardInterrupt, SystemExit):
        print("keyboard exit, bye!")
        p_read.terminate()       # terminate the serial thread
        queue.close()           # close the queue
        dPlot.close()
    else:
        print("exit, bye")
        p_read.terminate()       # terminate the thread
        queue.close()           # close the queue
        dPlot.close()

