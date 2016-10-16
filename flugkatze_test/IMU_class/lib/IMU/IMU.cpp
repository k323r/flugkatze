#include "Arduino.h"
#include "IMU.h"


// Constructor
IMU::IMU(int IMU_address) {
	_IMU_address = IMU_address;
	_roll_cal = 0;
	_pitch_cal = 0;
	
}

void IMU::init() {
	Wire.begin();
	Wire.beginTransmission(_IMU_address);
	Wire.write(0x6B);
	Wire.write(0);
	Wire.endTransmission(true);
	delay(250);
}

void IMU::calibrate(int n_samples) {
	for (int i = 0; i < n_samples; i++) {
		
	}
}

void IMU::read() {
	Serial.print("hello, world\n");
}
