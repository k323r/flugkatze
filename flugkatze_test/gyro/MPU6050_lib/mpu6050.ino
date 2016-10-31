#include <Wire.h> //Include the Wire.h library so we can communicate with the gyro.
#include "I2Cdev.h"
#include "MPU6050.h"

#define SCALE_2G 16384.0
#define SCALE_4G 8192.0
#define SCALE_8G 4096.0
#define SCALE_16G 2048.0

#define SCALE_250RAD 131
#define SCALE_500RAD 65.5
#define SCALE_1000RAD 32.8
#define scale_2000RAD 16.4


struct Raw {
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t gx;
    int16_t gy;
    int16_t gz;
} raw;

struct Flight_data {
    float ax;
    float ay;
    float az;

    float temp;

    float gx;
    float gy;
    float gz;

    int throttle;
    int roll;
    int pitch;
    int yaw;
} flight_data;

MPU6050 imu;

uint8_t scaling_factor = 0;

void scale (struct Flight_data *flight_data, struct Raw *raw, uint8_t scaling_factor);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){
	Wire.begin();
	Serial.begin(115200);

	Serial.println("INIT");
	imu.initialize();

	Serial.println("testing device communication");
	Serial.println(imu.testConnection() ? "connection established" : "connection failure");

	uint8_t accel_setting = imu.getFullScaleAccelRange();
	uint8_t gyro_setting = imu.getFullScaleGyroRange();

	Serial.println("get current accel setting: ");
	//Serial.print(imu.getFullScaleAccelRange());
	Serial.print(accel_setting);
	Serial.print("\n");

	Serial.println("get current gyro setting: ");
	//Serial.print(imu.getFullScaleGyroRange());
	Serial.print(gyro_setting);
	Serial.print("\n");

	Serial.println("offsets: ");

	Serial.print(imu.getXAccelOffset()); Serial.print("\t");
    Serial.print(imu.getYAccelOffset()); Serial.print("\t");
    Serial.print(imu.getZAccelOffset()); Serial.print("\t");
    Serial.print(imu.getXGyroOffset()); Serial.print("\t");
    Serial.print(imu.getYGyroOffset()); Serial.print("\t");
	Serial.println(imu.getZGyroOffset());
	delay(500);

	// set the sensitivity of both acceleration and gyro to something different:
	
	imu.setFullScaleAccelRange(0);
//	accel_setting = imu.getFullScaleAccelRange();

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){

	imu.getMotion6( &raw.ax, 
		&raw.ay, 
		&raw.az, 
		&raw.gx,
		&raw.gy, 
		&raw.gz ); // works. quite amazingly so.

	Serial.print("raw:\t");
	Serial.print(raw.ax); Serial.print("\t");
    Serial.print(raw.ay); Serial.print("\t");
    Serial.print(raw.az); Serial.print("\t");
    Serial.print(raw.gx); Serial.print("\t");
    Serial.print(raw.gy); Serial.print("\t");
	Serial.println(raw.gz);
	delay(500);
/*
	scale(&flight_data, &raw, imu.getFullScaleAccelRange());

	Serial.print("a/g:\t");
	Serial.print(flight_data.ax); Serial.print("\t");
    Serial.print(flight_data.ay); Serial.print("\t");
    Serial.print(flight_data.az); Serial.print("\t");
    Serial.print(flight_data.gx); Serial.print("\t");
    Serial.print(flight_data.gy); Serial.print("\t");
	Serial.println(flight_data.gz);
	delay(500);
*/
	Serial.print(imu.getXAccelOffset()); Serial.print("\t");
    Serial.print(imu.getYAccelOffset()); Serial.print("\t");
    Serial.print(imu.getZAccelOffset()); Serial.print("\t");
    Serial.print(imu.getXGyroOffset()); Serial.print("\t");
    Serial.print(imu.getYGyroOffset()); Serial.print("\t");
	Serial.println(imu.getZGyroOffset());

}

void scale (struct Flight_data *flight_data, struct Raw *raw, uint8_t scaling_factor) {
	switch (scaling_factor) {
		case 0:			// +32767 => +250 deg/sec und +32767 => 2g
			flight_data->ax = (float) (raw->ax / SCALE_2G);
			flight_data->ay = (float) (raw->ay / SCALE_2G);
			flight_data->az = (float) (raw->az / SCALE_2G);
	
			flight_data->gx = (float) (raw->gx / SCALE_250RAD);
			flight_data->gy = (float) (raw->gy / SCALE_250RAD);
			flight_data->gz = (float) (raw->gz / SCALE_250RAD);
			
			break;

		case 1:

			break;

		case 2:
	
			break;

		case 3:

			break;

		default:
			break;		
	}		
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

