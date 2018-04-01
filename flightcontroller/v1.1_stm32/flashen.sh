#!/bin/bash

# flash me
avrdude -q -V -p atmega328p -C /etc/avrdude/avrdude.conf -D -c arduino -b 57600 -P /dev/ttyUSB0 -U flash:w:build-nano-atmega328/v1.hex:i
