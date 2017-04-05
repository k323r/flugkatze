#!/usr/bin/env python
# -*- coding: utf-8 -*-

#
# Creation:    09.06.2013
# Last Update: 26.04.2015
#
# Copyright (c) 2013-2015 by Georg Kainzbauer <http://www.gtkdb.de>
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#

# import required modules
import time
import subprocess
import RPi.GPIO as GPIO

# define GPIO pin with connected button
GPIOPin = 2

# main function
def main():
  try:
    # use GPIO pin numbering convention
    GPIO.setmode(GPIO.BCM)

    # set up GPIO pin for input
    GPIO.setup(GPIOPin, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)

    # reset value
    value = 0

    while True:
      # increment value if button is pressed
      if GPIO.input(GPIOPin):
        value += 0.5

      else:
        # restart selected if value is larger than 0 and less than 3
        if value > 0 and value < 3:
          subprocess.call(["shutdown", "-r", "now"])
          return 0

        # shutdown selected if value is larger than 3 or equal
        elif value >= 3:
          subprocess.call(["shutdown", "-h", "now"])
          return 0

      # wait 500ms
      time.sleep(0.5)

    return 0

  # reset GPIO settings if execution has been stopped
  except KeyboardInterrupt:
    print("Execution stopped by user")
    GPIO.cleanup()

if __name__ == '__main__':
  # call main function
  main()



