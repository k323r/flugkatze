class State_c : public ExponentialFilter, public MPU6050 {
	
	private:

		enum Motors_e {
			hold,			// needed for a save state
			stop,			
			start
		} motors;


		float accOff[3];
		float gyroOff[3];

		float acc[3];
		float gyro[3];

		char len_angles_s;

		double cTime;
		double oldTime;
		double deltaTime;

		ExponentialFilter filterAX;
		ExponentialFilter filterAY;
		ExponentialFilter filterAZ;

		MPU6050 imu;

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

		void updateIMU() {
			gyro[0] = (float) ((imu.getRotationX() - gyroOff[0])/ SCALE_GYRO_250);
			gyro[1] = (float) ((imu.getRotationY() - gyroOff[1])/ SCALE_GYRO_250);
			gyro[2] = (float) ((imu.getRotationZ() - gyroOff[2])/ SCALE_GYRO_250);
			acc[0] = filterAX.filter(imu.getAccelerationX());
			acc[1] = filterAY.filter(imu.getAccelerationY());
			acc[2] = filterAZ.filter(imu.getAccelerationZ());
		}

	public:
	
	
		struct Angles_s {

			float rollAngle;
			float pitchAngle;
			float yawAngle;
		
		} angles_s;


		void init() {
			initFilters(10.0, 10.0, 10.0);
			initIMU();
			motors = stop;
			len_angles_s = sizeof(angles_s);
			angles_s.rollAngle = 0;
			angles_s.pitchAngle = 0;
			angles_s.yawAngle = 0;
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

		void update() {

			updateIMU();

			deltaTime = (cTime - oldTime) / 1000000.0;
			oldTime = cTime;
			cTime = micros();

			float rollAcc = 0.0, pitchAcc = 0.0;

    		rollAcc = atan2(acc[1], acc[2])*RAD2DEG;
    		pitchAcc = atan2(-acc[0], acc[2])*RAD2DEG;
	
			angles_s.rollAngle = (gyro[0] * deltaTime + angles_s.rollAngle) * 0.98 + rollAcc * 0.02;
			angles_s.pitchAngle = (gyro[1] * deltaTime + angles_s.pitchAngle) * 0.98 + pitchAcc * 0.02;
			angles_s.yawAngle = gyro[2] * deltaTime + angles_s.yawAngle;
		}	

		void motorsStart() {
			motors = start;
		}

		void motorsStop() {
			motors = stop;
		}

		void motorsHold() {
			motors = hold;
		}
		
		int motorsInitiated() {
			if (motors == hold) {
				return 1;
			} else {
				return 0;
			}
		}
		
		int motorsRunning() {
			if (motors == start) {
				return 1;
			} else {
				return 0;
			}
		}

		
		void print() {
			Serial.print((cTime - deltaTime )/ 1000000.0); Serial.print(" ");
			Serial.print(angles_s.rollAngle); Serial.print(" ");
			Serial.print(angles_s.pitchAngle); Serial.print(" ");
			Serial.print(angles_s.yawAngle); Serial.print(" ");
			Serial.print(acc[0]); Serial.print(" ");
			Serial.print(acc[1]); Serial.print(" ");
			Serial.println(acc[2]);		

		}

		void send() {
						
			char serialization_buffer[len_angles_s];
			memcpy(&serialization_buffer, &angles_s, len_angles_s);
			Serial.write('S');
			Serial.write((uint8_t *) &serialization_buffer, len_angles_s);
			Serial.write('E');

		}
};
