#include <Wire.h> //Include the Wire.h library so we can communicate with the gyro.
#include "I2Cdev.h"
#include "MPU6050.h"
#include "RunningAverage.h"

#define SCALE_2G 16384.0

#define SCALE_250RAD 131

MPU6050 imu;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){
	Wire.begin();
	Serial.begin(115200);

	Serial.println("# INIT");
	imu.initialize();

	Serial.println("# testing device communication");
	Serial.println(imu.testConnection() ? "# connection established" : "# connection failure");

	uint8_t accel_setting = imu.getFullScaleAccelRange();
	uint8_t gyro_setting = imu.getFullScaleGyroRange();

	Serial.println("# get current accel setting: ");
	//Serial.print(imu.getFullScaleAccelRange());
	Serial.println(accel_setting);

	Serial.println("# get current gyro setting: ");
	//Serial.print(imu.getFullScaleGyroRange());
	Serial.println(gyro_setting);

	Serial.println("# offsets: ");

	Serial.print(imu.getXAccelOffset()); Serial.print(" ");
    Serial.print(imu.getYAccelOffset()); Serial.print(" ");
    Serial.print(imu.getZAccelOffset()); Serial.print(" ");
    Serial.print(imu.getXGyroOffset()); Serial.print(" ");
    Serial.print(imu.getYGyroOffset()); Serial.print(" ");
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

	Serial.print("raw:");
	Serial.print(raw.ax); Serial.print(" ");
    Serial.print(raw.ay); Serial.print(" ");
    Serial.print(raw.az); Serial.print(" ");
    Serial.print(raw.gx); Serial.print(" ");
    Serial.print(raw.gy); Serial.print(" ");
	Serial.println(raw.gz);

	delay(500);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

