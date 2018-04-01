#ifndef _IMU_H_
#define _IMU_H_
#endif

#include <Wire.h>
#include "ExponentialFilter.h"
#include "Data.h"

#define IMU_DEFAULT_ADDR 0x68

class IMU {
    public:
        IMU(float scale_gyro, float scale_acc);
        
        IMU(float scale_gyro, float scale_acc, uint8_t address);

        void initialize(float W_FILTER_GYRO);
        void setOffsets(float ax, float ay, float az, float gx, float gy, float gz);
        bool testConnection();

        void update();

        

        float getRotationX();
        float getRotationY();
        float getRotationZ();

        float getAccelerationX();
        float getAccelerationY();
        float getAccelerationZ();
        
        
    private:
        uint8_t _imu_address;
        float _scale_gyro;
        float _scale_acc;
        
        float ax = 0.0;
        float ay = 0.0;
        float az = 0.0;
        float temp = 0.0;
        float roll = 0.0;
        float pitch = 0.0;
        float yaw = 0.0;

        Data offsets;

        ExponentialFilter eFAx;
        ExponentialFilter eFAy;
        ExponentialFilter eFAz;
        ExponentialFilter eFRoll;
        ExponentialFilter eFPitch;
        ExponentialFilter eFYaw;

};
