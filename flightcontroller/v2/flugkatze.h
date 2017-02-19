#include <Arduino.h>
#include <Wire.h> //Include the Wire.h library so we can communicate with the gyro.
#include "I2Cdev.h"
#include "MPU6050.h"

#define N_SAMPLES 1000
#define MPUADDR 0x68
#define BAUDRATE 115200
#define SCALE_ACC_2G 8192.0
#define SCALE_GYRO_250 131.0
#define WEIGHT_FILTER_GYRO 40.0
#define RAD2DEG 57.2957786



