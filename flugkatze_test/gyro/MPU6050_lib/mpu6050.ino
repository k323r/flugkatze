#include <Wire.h> //Include the Wire.h library so we can communicate with the gyro.
#include "I2Cdev.h"
#include "MPU6050.h"

struct Flight_data {
    int16_t ax;
    int16_t ay;
    int16_t az;

    float temp;

    int16_t gx;
    int16_t gy;
    int16_t gz;

    int throttle;
    int roll;
    int pitch;
    int yaw;
} flight_data;

MPU6050 imu;

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

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){

	imu.getMotion6(&flight_data.ax, &flight_data.ay, &flight_data.az, &flight_data.gx,&flight_data.gy, &flight_data.gz);

	Serial.print("a/g:\t");
	Serial.print(flight_data.ax); Serial.print("\t");
    Serial.print(flight_data.ay); Serial.print("\t");
    Serial.print(flight_data.az); Serial.print("\t");
    Serial.print(flight_data.gx); Serial.print("\t");
    Serial.print(flight_data.gy); Serial.print("\t");
	Serial.println(flight_data.gz);

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

