#include "IMU.h"

IMU::IMU(float scale_gyro, float scale_acc) {
    _imu_address = IMU_DEFAULT_ADDR;
    _scale_gyro = scale_gyro;
    _scale_acc = scale_acc;
    offsets = Data();
    eFAx = ExponentialFilter();
    eFAy = ExponentialFilter();
    eFAz = ExponentialFilter();
    eFRoll = ExponentialFilter();
    eFPitch = ExponentialFilter();
    eFYaw = ExponentialFilter();
}


IMU::IMU(float scale_gyro, float scale_acc, uint8_t address) {
    _imu_address = address;
    _scale_gyro = scale_gyro;
    _scale_acc = scale_acc;
    offsets = Data();
    eFAx = ExponentialFilter();
    eFAy = ExponentialFilter();
    eFAz = ExponentialFilter();
    eFRoll = ExponentialFilter();
    eFPitch = ExponentialFilter();
    eFYaw = ExponentialFilter();
}

void IMU::initialize(float W_FILTER_GYRO) {
    Wire.begin();
    Wire.beginTransmission(_imu_address);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0);     // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
    eFAx.setWeight(W_FILTER_GYRO); eFAx.setCurrent(0);
    eFAy.setWeight(W_FILTER_GYRO); eFAy.setCurrent(0);
    eFAz.setWeight(W_FILTER_GYRO); eFAz.setCurrent(0);
    eFRoll.setWeight(W_FILTER_GYRO); eFRoll.setCurrent(0);
    eFPitch.setWeight(W_FILTER_GYRO); eFPitch.setCurrent(0);
    eFYaw.setWeight(W_FILTER_GYRO); eFYaw.setCurrent(0);
    
}

void IMU::setOffsets(float ax, float ay, float az, float gx, float gy, float gz) {
  offsets.ax = ax;
  offsets.ay = ay;
  offsets.az = az;
  offsets.roll = gx;
  offsets.pitch = gy;
  offsets.yaw = gz;  
}

void IMU::update() {
  Wire.beginTransmission(_imu_address);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(_imu_address,14);  // request a total of 14 registers

  ax = (( Wire.read() << 8 | Wire.read() ) / _scale_acc ) - offsets.ax;// 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  ay = (( Wire.read() << 8 | Wire.read() ) / _scale_acc ) - offsets.ay;  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  az = (( Wire.read() << 8 | Wire.read() ) / _scale_acc ) - offsets.az;  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  temp = Wire.read() << 8 | Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)

  roll = (( Wire.read() << 8 | Wire.read() ) / _scale_gyro ) - offsets.roll;  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  pitch = (( Wire.read() << 8 | Wire.read() ) / _scale_gyro ) - offsets.pitch;  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  yaw = (( Wire.read() << 8 | Wire.read() ) / _scale_gyro ) - offsets.yaw;  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

bool IMU::testConnection() {
  return 0;
}

float IMU::getRotationX() {
  return(ax);
}

float IMU::getRotationY() {
  return(ay);
}

float IMU::getRotationZ() {
  return(az);
}

float IMU::getAccelerationX() {
  return(roll);
}

float IMU::getAccelerationY() {
  return(pitch);
}

float IMU::getAccelerationZ() {
  return(yaw);
}
