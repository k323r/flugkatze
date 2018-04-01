#!/bin/bash

# TODO: make distinction between stm32 and arduino
# TODO: more parameters

SOURCEFILE=$1

if ! [ -e $SOURCEFILE ]
then
    echo "not a valid file!, exiting"
    exit
fi

USER_LIB_I2CDEVLIB=../../../lib/i2cdevlib/Arduino/I2Cdev
USER_LIB_MPU6050=../../../lib/MPU6050
USER_LIB_BMP180=../../../lib/BMP180
USER_LIB_EXPFILTER=../../../lib/ExponentialFilter2



ARDUINO_BUILDER=/usr/share/arduino/arduino-builder
STLINK=/home/asander/.arduino15/packages/STM32/tools/STM32Tools/1.0.2/tools/linux/stlink_upload
BUILD_PATH=/tmp/arduino_build
CACHE_PATH=/tmp/arduino_cache

if [ -d $BUILD_PATH ]
then
    rm -rf $BUILD_PATH
fi

if [ -d $CACHE_PATH ]
then
    rm -rf $CACHE_PATH
fi

mkdir -p $CACHE_PATH $BUILD_PATH

echo "Building ${SOURCEFILE} in ${BUILD_PATH}"

# dump preferences in a cache folder
$ARDUINO_BUILDER -dump-prefs \
-logger=machine \
-hardware /usr/share/arduino/hardware \
-hardware /home/asander/.arduino15/packages \
-tools /usr/share/arduino/tools-builder \
-tools /home/asander/.arduino15/packages \
-built-in-libraries /usr/share/arduino/libraries \
-libraries /home/asander/Arduino/libraries \
-libraries $USER_LIB_I2CDEVLIB \
-libraries $USER_LIB_EXPFILTER \
-libraries $USER_LIB_MPU6050 \
-fqbn=STM32:stm32:GenF103:pnum=BLUEPILL_F103C8,flash=C8,upload_method=STLinkMethod,xserial=third,opt=osstd \
-ide-version=10805 \
-build-path $BUILD_PATH \
-build-cache $CACHE_PATH \
-prefs=build.warn_data_percentage=75 \
-prefs=runtime.tools.arm-none-eabi-gcc.path=/home/asander/.arduino15/packages/STM32/tools/arm-none-eabi-gcc/6-2017-q2-update \
-prefs=runtime.tools.STM32Tools.path=/home/asander/.arduino15/packages/STM32/tools/STM32Tools/1.0.2 \
-prefs=runtime.tools.CMSIS.path=/home/asander/.arduino15/packages/STM32/tools/CMSIS/5.3.0 \
-verbose \
$SOURCEFILE

echo "dumped prefs... ${CACHE_PATH}"

# compile everything
$ARDUINO_BUILDER -compile \
-logger=machine \
-hardware /usr/share/arduino/hardware \
-hardware /home/asander/.arduino15/packages \
-tools /usr/share/arduino/tools-builder \
-tools /home/asander/.arduino15/packages \
-built-in-libraries /usr/share/arduino/libraries \
-libraries /home/asander/Arduino/libraries \
-libraries $USER_LIB_I2CDEVLIB \
-libraries $USER_LIB_EXPFILTER \
-libraries $USER_LIB_MPU6050 \
-fqbn=STM32:stm32:GenF103:pnum=BLUEPILL_F103C8,flash=C8,upload_method=STLinkMethod,xserial=third,opt=osstd \
-ide-version=10805 \
-build-path $BUILD_PATH \
-build-cache $CACHE_PATH \
-prefs=build.warn_data_percentage=75 \
-prefs=runtime.tools.arm-none-eabi-gcc.path=/home/asander/.arduino15/packages/STM32/tools/arm-none-eabi-gcc/6-2017-q2-update \
-prefs=runtime.tools.STM32Tools.path=/home/asander/.arduino15/packages/STM32/tools/STM32Tools/1.0.2 \
-prefs=runtime.tools.CMSIS.path=/home/asander/.arduino15/packages/STM32/tools/CMSIS/5.3.0 \
-verbose \
$SOURCEFILE

echo "done compiling!"

# flash
$STLINK ttyUSB0 {upload.altID} {upload.usbID} /tmp/${BUILD_PATH}/${SOURCEFILE}.bin

echo "done flashing!"



# flash me
# avrdude -q -V -p atmega328p -C /etc/avrdude/avrdude.conf -D -c arduino -b 57600 -P /dev/ttyUSB0 -U flash:w:build-nano-atmega328/v1.hex:i
