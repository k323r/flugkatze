// imma write ma own library cos dis sucks
//#include "I2Cdev.h"
//#include "MPU6050.h"

#ifndef _V2_STM32_H_
#define _V2_STM32_H_
#endif

//#include "ExponentialFilter.h"
#include "Telemetry.h"
#include "IMU.h"

#define BAUDRATE 115200     // Serial speed
#define NSAMPLES 2000       // number of samples for offset
#define W_FILTER_GYRO 40.0  // weight for the exponential filter
#define SCALE_GYRO_250 131
#define SCALE_ACC_2G 16384.0

// data to be sent over serial
//struct Data {
//    float ax;
//    float ay;
//    float az;
//    float roll;
//    float pitch;
//    float yaw;
//    float temp;
//};
//
//struct Offsets {
//    float roll_off;
//    float pitch_off;
//    float yaw_off;
//    float gx_off;
//    float gy_off;
//    float gz_off;
//};


// prototypes
void get_imu(IMU &imu, ExponentialFilter &gF, struct Data *data, struct Offsets *offsets);

