#include "IMU.h"

#define BAUDRATE 115200     // Serial speed
#define NSAMPLES 2000       // number of samples for offset
#define W_FILTER_GYRO 40.0  // weight for the exponential filter
#define SCALE_GYRO_250 131
#define SCALE_ACC_2G 16384.0
// INIT some objects
// IMU

Data offsets;
Data data;

IMU imu(SCALE_GYRO_250, SCALE_ACC_2G, 0x68);

// some global variables
float old_time = 0.0;     // used to store the old cycle time
float current_time = 0.0; // used to store the current cycle time
float deltaT = 0.0;       // time since the last cycle

void setup() {
  Serial2.begin(BAUDRATE);
  Serial2.println("init");

  // init the imu and test the connection
  imu.initialize(W_FILTER_GYRO);
  Serial2.print("testing imu connection...");
  Serial2.println(imu.testConnection() ? "connection established" : "connection failed");
  // imu.setFullScaleAccelRange(0);    // scaling

  for (int i = 0; i < NSAMPLES; i++) {
        imu.update();
        offsets.roll += imu.getRotationX();
        offsets.pitch += imu.getRotationY();
        offsets.yaw += imu.getRotationZ();
    }

  offsets.roll /= NSAMPLES;
  offsets.pitch /= NSAMPLES;
  offsets.yaw /= NSAMPLES;

  imu.setOffsets(offsets.ax, offsets.ay, offsets.az, offsets.roll, offsets.pitch, offsets.yaw);

  
  // put your setup code here, to run once:

}

void loop() {
  deltaT = (( current_time = micros() ) - old_time ) / 1000000.0;
  old_time = current_time;
  imu.update();
  Serial2.print(imu.getAccelerationX()); Serial2.print(" ");
  Serial2.print(imu.getAccelerationX()); Serial2.print(" ");
  Serial2.print(imu.getAccelerationX()); Serial2.print(" ");
  Serial2.print(imu.getRotationX()); Serial2.print(" ");
  Serial2.print(imu.getRotationY()); Serial2.print(" ");
  Serial2.println(imu.getRotationZ());
  delay(250);
  
}

