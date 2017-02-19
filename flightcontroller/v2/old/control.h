void calc_angles(t_Sensors *sensors, t_State *state) {
//	Serial.print("* delta T: ");
//	Serial.println(sensors->deltaT);

	float roll_acc = 0;
	float pitch_acc = 0;

	state->a_roll_old = state->a_roll;
	state->a_pitch_old = state->a_pitch;
	state->a_yaw_old = state->a_yaw;

	roll_acc = atan2(sensors->ay, sensors->az)*RAD2DEG;
	pitch_acc = atan2(-sensors->ax, sensors->az)*RAD2DEG;

	state->a_roll = (sensors->gx * sensors->deltaT + state->a_roll_old) * 0.98 + roll_acc * 0.02;
	state->a_pitch = (sensors->gy * sensors->deltaT + state->a_pitch_old) * 0.98 + pitch_acc * 0.02;	
	state->a_yaw = sensors->gz * sensors->deltaT + state->a_yaw_old;		
	
}

void update_sensors(t_Sensors *sensors, t_Offsets *offsets, t_State *state, MPU6050 *imu) {
	sensors->gx = ((float)imu->getRotationX() - offsets->gx) / SCALE_GYRO_250;
	sensors->gy = ((float)imu->getRotationY() - offsets->gy) / SCALE_GYRO_250;
	sensors->gz = ((float)imu->getRotationZ() - offsets->gz) / SCALE_GYRO_250;
	
	sensors->ax = imu->getAccelerationX();
	sensors->ay = imu->getAccelerationY();
	sensors->az = imu->getAccelerationZ();

	// sensors->ax = ((float)imu->getAccelerationX() - offsets->ax) / SCALE_ACC_2G;
	// sensors->ay = ((float)imu->getAccelerationY() - offsets->ay) / SCALE_ACC_2G;
	// sensors->az = ((float)imu->getAccelerationX() - offsets->az) / SCALE_ACC_2G;
	
	calc_angles(sensors, state);	
	
}


