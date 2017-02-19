class State_c : public ExponentialFilter, public MPU6050 {
	
	private:
		float accOff[3];
		float gyroOff[3];

		float acc[3];
		float gyro[3];

		float rollAngle;
		float pitchAngle;
		float yawAngle;

		double cTime;
		double oldTime;
		double deltaTime;

		ExponentialFilter filterAX;
		ExponentialFilter filterAY;
		ExponentialFilter filterAZ;

		MPU6050 imu;

		void update() {
			gyro[0] = (float) ((imu.getRotationX() - gyroOff[0])/ SCALE_GYRO_250);
			gyro[1] = (float) ((imu.getRotationY() - gyroOff[1])/ SCALE_GYRO_250);
			gyro[2] = (float) ((imu.getRotationZ() - gyroOff[2])/ SCALE_GYRO_250);

			acc[0] = filterAX.filter(imu.getAccelerationX());
			acc[1] = filterAY.filter(imu.getAccelerationY());
			acc[2] = filterAZ.filter(imu.getAccelerationZ());
		}

	public:

		void initFilters(float weightAX, float weightAY, float weightAZ) {
			filterAX.setWeight(weightAX);
			filterAX.setCurrent(0);
			filterAY.setWeight(weightAY);
			filterAY.setCurrent(0);
			filterAZ.setWeight(weightAZ);
			filterAZ.setCurrent(0);
		}

		void initIMU() {
			Serial.print("# testing IMU communication... ");
    		Serial.println(imu.testConnection() ? "connection established" : "connection failed");
    		imu.setFullScaleAccelRange(0);
		}

		void initState() {
			rollAngle = 0;
			pitchAngle = 0;
			yawAngle = 0;
			cTime = 0;
			oldTime = 0;
			deltaTime = 0;

			for (int i = 0; i < N_SAMPLES; i++) {
				gyroOff[0] += (float) imu.getRotationX();
				gyroOff[1] += (float) imu.getRotationY();
				gyroOff[2] += (float) imu.getRotationZ();
			}
			
			gyroOff[0] /= N_SAMPLES;
			gyroOff[1] /= N_SAMPLES;
			gyroOff[2] /= N_SAMPLES;
	
		}

		void updateState() {

			update();

			deltaTime = (cTime - oldTime) / 1000000.0;
			oldTime = cTime;
			cTime = micros();

			float rollAcc = 0.0, pitchAcc = 0.0;

    		rollAcc = atan2(acc[1], acc[2])*RAD2DEG;
    		pitchAcc = atan2(-acc[0], acc[2])*RAD2DEG;
	
			rollAngle = (gyro[0] * deltaTime + rollAngle) * 0.98 + rollAcc * 0.02;
			pitchAngle = (gyro[1] * deltaTime + pitchAngle) * 0.98 + pitchAcc * 0.02;
			yawAngle = gyro[2] * deltaTime + yawAngle;
		}	
		
		void printState() {
			Serial.print((cTime - deltaTime )/ 1000000.0); Serial.print(" ");
			Serial.print(rollAngle); Serial.print(" ");
			Serial.print(pitchAngle); Serial.print(" ");
			Serial.print(yawAngle); Serial.print(" ");
			Serial.print(acc[0]); Serial.print(" ");
			Serial.print(acc[1]); Serial.print(" ");
			Serial.println(acc[2]);		

		}
};
