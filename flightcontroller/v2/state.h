class State_c : public ExponentialFilter, public MPU6050 {
	
	private:
		float accOff[3];
		float gyroOff[3];

		float acc[3];
		float gyro[3];

		char len_s_angles;

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
		
		struct S_Angles {

			float rollAngle;
			float pitchAngle;
			float yawAngle;
		
		} s_angles;

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
			len_s_angles = sizeof(s_angles);
			s_angles.rollAngle = 0;
			s_angles.pitchAngle = 0;
			s_angles.yawAngle = 0;
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
	
			s_angles.rollAngle = (gyro[0] * deltaTime + s_angles.rollAngle) * 0.98 + rollAcc * 0.02;
			s_angles.pitchAngle = (gyro[1] * deltaTime + s_angles.pitchAngle) * 0.98 + pitchAcc * 0.02;
			s_angles.yawAngle = gyro[2] * deltaTime + s_angles.yawAngle;
		}	
		
		void printState() {
			Serial.print((cTime - deltaTime )/ 1000000.0); Serial.print(" ");
			Serial.print(s_angles.rollAngle); Serial.print(" ");
			Serial.print(s_angles.pitchAngle); Serial.print(" ");
			Serial.print(s_angles.yawAngle); Serial.print(" ");
			Serial.print(acc[0]); Serial.print(" ");
			Serial.print(acc[1]); Serial.print(" ");
			Serial.println(acc[2]);		

		}

		void sendState() {
						
			char serialization_buffer[len_s_angles];
			memcpy(&serialization_buffer, &s_angles, len_s_angles);
			Serial.write('S');
			Serial.write((uint8_t *) &serialization_buffer, len_s_angles);
			Serial.write('E');

		}
};
