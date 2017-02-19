void print_start () {
	Serial.println("##############################################");
	Serial.println("### THIS IS FLUGATZE FLIGHTCONTROLLER V2.0 ###");
	Serial.println("##############################################");
	Serial.println("");

    Serial.println("### starting INIT ###");
}

void init_structs (t_Sensors *sensors, t_Offsets *sensor_offsets, t_State *state ) {
	sensors->ax = 0.0;
	sensors->ay = 0.0;
	sensors->az = 0.0;
	sensors->gx = 0.0;
	sensors->gy = 0.0;
	sensors->gz = 0.0;
	sensors->deltaT = 0.0;
	
	sensor_offsets->ax = 0.0;	
	sensor_offsets->ay = 0.0;	
	sensor_offsets->az = 0.0;	
	sensor_offsets->gx = 0.0;	
	sensor_offsets->gy = 0.0;	
	sensor_offsets->gz = 0.0;

	state->a_roll_old = 0.0;
	state->a_pitch_old = 0.0;
	state->a_yaw_old = 0.0;
	state->a_roll = 0.0;
	state->a_pitch = 0.0;
	state->a_yaw = 0.0;
	state->height = 0.0;
}

void calc_offsets (t_Offsets *offsets, MPU6050 *imu) {
	for (int i = 0; i < N_SAMPLES; i++) {
	
		offsets->ax += (float) imu->getAccelerationX();
		offsets->ay += (float) imu->getAccelerationY();
		offsets->az += (float) imu->getAccelerationZ();
				
		offsets->gx += (float) imu->getRotationX();
		offsets->gy += (float) imu->getRotationY();
		offsets->gz += (float) imu->getRotationZ();	
	}

	offsets->ax /= N_SAMPLES;
	offsets->ay /= N_SAMPLES;
	offsets->az /= N_SAMPLES;

	offsets->gx /= N_SAMPLES;
	offsets->gy /= N_SAMPLES;
	offsets->gz /= N_SAMPLES;

	Serial.print("# offsets ROTATION: ");
	Serial.print(offsets->gx);	Serial.print(" ");
	Serial.print(offsets->gy);	Serial.print(" ");
	Serial.println(offsets->gz);
	Serial.print("# offsets ACCELERATION: ");
	Serial.print(offsets->ax);	Serial.print(" ");
	Serial.print(offsets->ay);	Serial.print(" ");
	Serial.println(offsets->az);

	Serial.println("# done calculating offsets");

}
