#include <Arduino.h>
#include <Wire.h> //Include the Wire.h library so we can communicate with the gyro.
#include "I2Cdev.h"
#include "MPU6050.h"
#include "ExponentialFilter2.h"

#define N_SAMPLES 1000
#define MPUADDR 0x68
#define BAUDRATE 115200
#define SCALE_ACC_2G 8192.0
#define SCALE_GYRO_250 131.0
#define WEIGHT_EFILTER_GYRO 40.0
#define RAD2DEG 57.2957786


// data structures

// structure used globally to save the current values 
// of the  sensors as well as the current delta t
struct Sensors {

    float ax;
    float ay;
    float az;

    float gx;
    float gy;
    float gz;

	double deltaT;
	
};

typedef struct Sensors t_Sensors;


// data structure used to store the offset values of both 
// gyro and accelerometer
struct Sensor_Offsets {

	float ax;
	float ay;
	float az;

	float gx;
	float gy;
	float gz;

};

typedef struct Sensor_Offsets t_Offsets;

// store the current state of the copter in the following data structure
struct State {

	float a_roll_old;
	float a_pitch_old;
	float a_yaw_old;	

	float a_roll;
	float a_pitch;
	float a_yaw;

	float height;	
};

typedef struct State t_State;

