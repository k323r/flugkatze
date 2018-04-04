#ifndef _IMU_H_
#define _IMU_H_
#endif

#pragma once

#include <Wire.h>
#include "ExponentialFilter.h"
#include "Data.h"

#define IMU_DEFAULT_ADDR 0x68

#define IMU_ACC_CONF    0x1C
#define IMU_ACC_X_H     0x3B
#define IMU_ACC_X_L     0x3C
#define IMU_ACC_Y_H     0x3D
#define IMU_ACC_Y_L     0x3E
#define IMU_ACC_Z_H     0x3F
#define IMU_ACC_Z_L     0x40

#define IMU_TEMP_H      0x41
#define IMU_TEMP_L       0x42

#define IMU_GYRO_CONF     0x1B
#define IMU_GYRO_X_H      0x43
#define IMU_GYRO_X_L      0x44
#define IMU_GYRO_Y_H      0x45
#define IMU_GYRO_Y_L      0x46
#define IMU_GYRO_Z_H      0x47
#define IMU_GYRO_Z_L      0x48

class IMU {
    public:
        IMU(float scale_gyro, float scale_acc);
        
        IMU(float scale_gyro, float scale_acc, uint8_t address);

        void initialize(float W_FILTER_GYRO, int NSAMPLES);
        bool testConnection();
        void update(Data* state);
        void setAccRange(uint8_t range);
        uint8_t getAccRange();

        void setGyroRange(uint8_t range);
        uint8_t getGyroRange();

        float getRotationX();
        float getRotationY();
        float getRotationZ();
        float getTemp();
        float getAccelerationX();
        float getAccelerationY();
        float getAccelerationZ();
        
        
    private:
        uint8_t _imu_address;
        float _scale_gyro;
        float _scale_acc;
        
        Data _offsets;

        ExponentialFilter ax;
        ExponentialFilter ay;
        ExponentialFilter az;
        ExponentialFilter roll;
        ExponentialFilter pitch;
        ExponentialFilter yaw;

};
