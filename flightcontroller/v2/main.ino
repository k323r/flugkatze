#include "flugkatze.h"
#include "init.h"
#include "control.h"

MPU6050 imu;

/*
	ROLLING AXIS -> X AXIS of the GYRO
	PITCHING AXIS -> Y AXIS of the GYRO
	YAWING AXis -> Z AXIS of the GYRO
*/

// filtering objects
ExponentialFilter<float> filterGX(WEIGHT_EFILTER_GYRO, 0);
ExponentialFilter<float> filterGY(WEIGHT_EFILTER_GYRO, 0);
ExponentialFilter<float> filterGZ(WEIGHT_EFILTER_GYRO, 0);

// used to calculate current delta T
double c_time = 0.0, old_time = 0.0;

t_Sensors sensors;
t_Offsets offsets;
t_State state;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){
	
	init_structs(&sensors, &offsets, &state);
   
	Serial.begin(BAUDRATE);
    Wire.begin();                                                //Start the I2C as master.

	print_start();

    imu.initialize();
    Serial.print("# testing IMU communication... ");
    Serial.println(imu.testConnection() ? "connection established" : "connection failed");

    imu.setFullScaleAccelRange(0);
	calc_offsets(&offsets, &imu);	
	
    Serial.println("# INIT done");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
void loop(){
   	
	// calculate current delta T: 
    sensors.deltaT = ( c_time - old_time ) / 1000000.0;
    old_time = c_time;
    c_time = micros();
	
//	calc_angles(&sensors, &state);	
	update_sensors(&sensors, &offsets, &state, &imu);

	//Serial.print(" current angles: ");
	Serial.print((c_time - sensors.deltaT) / 1000000.0); Serial.print(" ");
	Serial.print(state.a_roll); Serial.print(" ");
	Serial.print(state.a_pitch); Serial.print(" ");
	Serial.print(state.a_yaw); Serial.print(" ");
	Serial.print(sensors.ax); Serial.print(" ");
	Serial.print(sensors.ay); Serial.print(" ");
	Serial.print(sensors.az);
	Serial.println("");

	delay(100);
    
}

