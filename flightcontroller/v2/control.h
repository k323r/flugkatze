void calc_angles(t_Sensors *sensors, t_State *state) {
//	Serial.print("* delta T: ");
//	Serial.println(sensors->deltaT);

	state->a_roll_old = state->a_roll;
	state->a_pitch_old = state->a_pitch;
	state->a_yaw_old = state->a_yaw;

	state->a_roll = sensors->gx * sensors->deltaT + state->a_roll_old;		
	state->a_pitch = sensors->gy * sensors->deltaT + state->a_pitch_old;		
	state->a_yaw = sensors->gz * sensors->deltaT + state->a_yaw_old;		
	
}

void update_sensors(t_Sensors *sensors, t_Offsets *offsets, t_State *state, MPU6050 *imu) {
	sensors->gx = (imu->getRotationX() - offsets->gx) / SCALE_GYRO_250;
	sensors->gy = (imu->getRotationY() - offsets->gy) / SCALE_GYRO_250;
	sensors->gz = (imu->getRotationZ() - offsets->gz) / SCALE_GYRO_250;
	
	calc_angles(sensors, state);	
	
}


