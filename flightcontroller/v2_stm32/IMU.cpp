#include "IMU.h"

IMU::IMU(float scale_gyro, float scale_acc) {
    _imu_address = IMU_DEFAULT_ADDR;
    _scale_gyro = scale_gyro;
    _scale_acc = scale_acc;
    _offsets = Data();
    ax = ExponentialFilter();
    ay = ExponentialFilter();
    az = ExponentialFilter();
    roll = ExponentialFilter();
    pitch = ExponentialFilter();
    yaw = ExponentialFilter();
}


IMU::IMU(float scale_gyro, float scale_acc, uint8_t address) {
    _imu_address = address;
    _scale_gyro = scale_gyro;
    _scale_acc = scale_acc;
    _offsets = Data();
    ax = ExponentialFilter();
    ay = ExponentialFilter();
    az = ExponentialFilter();
    roll = ExponentialFilter();
    pitch = ExponentialFilter();
    yaw = ExponentialFilter();
}

void IMU::initialize(float W_FILTER_GYRO, int NSAMPLES) {
    Serial2.println("waking up gyro");
    Wire.begin();
    Wire.beginTransmission(_imu_address);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0);     // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
    Serial2.println("awake and ready");
    IMU::setAccRange(0);
    IMU::setGyroRange(0);
    ax.setWeight(W_FILTER_GYRO); ax.setCurrent(0);
    ay.setWeight(W_FILTER_GYRO); ay.setCurrent(0);
    az.setWeight(W_FILTER_GYRO); az.setCurrent(0);
    roll.setWeight(W_FILTER_GYRO); roll.setCurrent(0);
    pitch.setWeight(W_FILTER_GYRO); pitch.setCurrent(0);
    yaw.setWeight(W_FILTER_GYRO); yaw.setCurrent(0);

    // calculate offsets
    for (int i = 0; i < NSAMPLES; i++) {
        _offsets.roll += IMU::getRotationX();
        _offsets.pitch += IMU::getRotationY();
        _offsets.yaw += IMU::getRotationZ();
    }

    _offsets.roll /= NSAMPLES;
    _offsets.pitch /= NSAMPLES;
    _offsets.yaw /= NSAMPLES;

    Serial2.print("offset roll: "); Serial2.println(_offsets.roll);
    Serial2.print("offset pitch: "); Serial2.println(_offsets.pitch);
    Serial2.print("offset yaw: "); Serial2.println(_offsets.yaw);
    
}

void IMU::update(Data* state) {
  state->ax = ax.filter(IMU::getAccelerationX()) / _scale_acc;// 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  state->ay = ay.filter(IMU::getAccelerationY()) / _scale_acc;  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  state->az = az.filter(IMU::getAccelerationZ()) / _scale_acc;  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  state->temp = IMU::getTemp();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)

  roll.filter(IMU::getRotationX() - _offsets.roll);
  state->roll = roll.getCurrent()/_scale_gyro;  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  
  pitch.filter(IMU::getRotationY()- _offsets.pitch);
  state->pitch = pitch.getCurrent()/_scale_gyro;  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)

  yaw.filter(IMU::getRotationZ()- _offsets.yaw);
  state->yaw = yaw.getCurrent()/_scale_gyro;  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

bool IMU::testConnection() {
  Serial2.println("connection established, duh");
  return 1;
}

void IMU::setGyroRange(uint8_t range) {
  Wire.beginTransmission(_imu_address);
  Wire.write(IMU_GYRO_CONF);
  Wire.write(range);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

uint8_t IMU::getGyroRange() {
  Wire.beginTransmission(_imu_address);
  Wire.write(IMU_GYRO_CONF);
  Wire.endTransmission(false);
  Wire.requestFrom(_imu_address, 1);
  return (Wire.read());
}

void IMU::setAccRange(uint8_t range) {
  Wire.beginTransmission(_imu_address);
  Wire.write(IMU_ACC_CONF);
  Wire.write(range);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

uint8_t IMU::getAccRange() {
  Wire.beginTransmission(_imu_address);
  Wire.write(IMU_ACC_CONF);
  Wire.endTransmission(false);
  Wire.requestFrom(_imu_address, 1);
  return (Wire.read());
}

float IMU::getAccelerationX() {
  Wire.beginTransmission(_imu_address);
  Wire.write(IMU_ACC_X_H);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(_imu_address, 2);
  return ((float) (Wire.read() << 8 | Wire.read()));
}

float IMU::getAccelerationY() {
  Wire.beginTransmission(_imu_address);
  Wire.write(IMU_ACC_Y_H);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(_imu_address, 2);
  return ((float) (Wire.read() << 8 | Wire.read()));
 }

float IMU::getAccelerationZ() {
  Wire.beginTransmission(_imu_address);
  Wire.write(IMU_ACC_Z_H);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(_imu_address, 2);
  return ((float) (Wire.read() << 8 | Wire.read()));
}

float IMU::getTemp() {
  Wire.beginTransmission(_imu_address);
  Wire.write(IMU_TEMP_H);
  Wire.endTransmission(false);
  Wire.requestFrom(_imu_address, 2);
  return (float) (Wire.read() << 8 | Wire.read());
}

float IMU::getRotationX() {
  Wire.beginTransmission(_imu_address);
  Wire.write(IMU_GYRO_X_H);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(_imu_address, 2);
  return ((float) (Wire.read() << 8 | Wire.read()));
}

float IMU::getRotationY() {
  Wire.beginTransmission(_imu_address);
  Wire.write(IMU_GYRO_Y_H);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(_imu_address, 2);
  return ((float) (Wire.read() << 8 | Wire.read()));
}

float IMU::getRotationZ() {
  Wire.beginTransmission(_imu_address);
  Wire.write(IMU_GYRO_Z_H);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(_imu_address, 2);
  return ((float) (Wire.read() << 8 | Wire.read()));
}
