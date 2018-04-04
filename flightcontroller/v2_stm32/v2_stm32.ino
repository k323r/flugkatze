#include "IMU.h"
#include "Data.h"

#define BAUDRATE 115200     // Serial speed
#define NSAMPLES 1000       // number of samples for offset
#define W_FILTER_GYRO 50.0  // weight for the exponential filter
#define SCALE_GYRO_250 131.0
#define SCALE_ACC_2G 16384.0
// INIT some objects
// IMU

Data state;

IMU imu(SCALE_GYRO_250, SCALE_ACC_2G, 0x68);

// some global variables
float old_time = 0.0;     // used to store the old cycle time
float current_time = 0.0; // used to store the current cycle time
float deltaT = 0.0;       // time since the last cycle

void setup() {
  Serial2.begin(BAUDRATE);
  imu.testConnection();
  imu.initialize(W_FILTER_GYRO, NSAMPLES);
  Serial2.print("size of float: "); Serial2.println(sizeof(float));

  // imu.setFullScaleAccelRange(0);    // scaling
}

void loop() {
  deltaT = (( current_time = micros() ) - old_time ) / 1000000.0;
  old_time = current_time;
  //Serial2.print("current time: "); Serial2.print(current_time); Serial2.print(", deltaT: "); Serial2.println(deltaT); 
  imu.update(&state);
  state.send();
  
  Serial2.print(state.ax); Serial2.print(" ");
  Serial2.print(state.ay); Serial2.print(" ");
  Serial2.print(state.az); Serial2.print(" ");
  Serial2.print(state.roll); Serial2.print(" ");
  Serial2.print(state.pitch); Serial2.print(" ");
  Serial2.println(state.yaw);
  delay(100);
  
}

