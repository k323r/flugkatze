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

    // let the imu settle
    delay(200);
    Serial2.println("starting offset measurements");
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

    delay(5000);
    
}

uint8_t IMU::update(Data* state) {
  Wire.beginTransmission(_imu_address);
  Wire.write(IMU_ACC_X_H);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
  if (rcode) {
    Serial2.print(F("i2cRead failed: "));
    Serial2.println(rcode);
    return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
  }
  Wire.requestFrom(_imu_address, 14, (uint8_t)true); // Send a repeated start and then release the bus after reading
    
  state->ax = ax.filter((float) IMU::getAddedRegisters()) / _scale_acc;// 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  state->ay = ay.filter((float) IMU::getAddedRegisters()) / _scale_acc;  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  state->az = az.filter((float) IMU::getAddedRegisters()) / _scale_acc;  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  state->temp = IMU::getAddedRegisters();  // missing the conversion function

  state->roll = roll.filter( IMU::getAddedRegisters() - _offsets.roll) / _scale_gyro;  
  state->pitch = pitch.filter( IMU::getAddedRegisters() - _offsets.pitch) / _scale_gyro;
  state->yaw = yaw.filter( IMU::getAddedRegisters() - _offsets.yaw) / _scale_gyro;
}

uint16_t IMU::getAddedRegisters() {
  uint8_t reg1, reg2;
  uint32_t timeOutTimer = 0;
  if (Wire.available())
    reg1 = Wire.read();
  else {
    timeOutTimer = micros();
    while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
    if (Wire.available())
      reg1 = Wire.read();
    else {
      Serial2.println(F("i2cRead timeout"));
      return 0; // This error value is not already taken by endTransmission
    }
  }
  timeOutTimer = 0;
  if (Wire.available())
    reg2 = Wire.read();
  else {
    timeOutTimer = micros();
    while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
    if (Wire.available())
      reg2 = Wire.read();
    else {
      Serial2.println(F("i2cRead timeout"));
      return 0; // This error value is not already taken by endTransmission
    }
  }
  return (reg1 << 8 |reg2);
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
  return (Wire.read() << 8 | Wire.read());
}

float IMU::getAccelerationY() {
  Wire.beginTransmission(_imu_address);
  Wire.write(IMU_ACC_Y_H);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(_imu_address, 2);
  return (Wire.read() << 8 | Wire.read());
 }

float IMU::getAccelerationZ() {
  Wire.beginTransmission(_imu_address);
  Wire.write(IMU_ACC_Z_H);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(_imu_address, 2);
  return (Wire.read() << 8 | Wire.read());
}

float IMU::getTemp() {
  Wire.beginTransmission(_imu_address);
  Wire.write(IMU_TEMP_H);
  Wire.endTransmission(false);
  Wire.requestFrom(_imu_address, 2);
  return (Wire.read() << 8 | Wire.read());
}

float IMU::getRotationX() {
  Wire.beginTransmission(_imu_address);
  Wire.write(IMU_GYRO_X_H);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(_imu_address, 2);
  return (Wire.read() << 8 | Wire.read());
}

float IMU::getRotationY() {
  Wire.beginTransmission(_imu_address);
  Wire.write(IMU_GYRO_Y_H);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(_imu_address, 2);
  return (Wire.read() << 8 | Wire.read());
}

float IMU::getRotationZ() {
  Wire.beginTransmission(_imu_address);
  Wire.write(IMU_GYRO_Z_H);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(_imu_address, 2);
  return (Wire.read() << 8 | Wire.read());
}
