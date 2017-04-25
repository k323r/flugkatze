class Receiver_c : Channel_c{
	
	private:
	
		const static float _offset = 1500.0;
		const static float _conversion = 0.01;

	public:
		
		Channel_c Yaw;
		Channel_c Throttle;
		Channel_c Pitch;
		Channel_c Roll;		
	
		void init() {
			Yaw.initChannel();
			Throttle.initChannel();
			Pitch.initChannel();
			Roll.initChannel();
		}

		void print() {
			Serial.print("Yaw: "); Serial.print(Yaw.getInput());
			Serial.print(" Throttle: "); Serial.print(Throttle.getInput());
			Serial.print(" Roll: "); Serial.print(Roll.getInput());
			Serial.print(" Pitch: "); Serial.println(Pitch.getInput());
			
		}

		float yawAngle () {
			return (Yaw.getInput() - _offset) * _conversion;
		}

		float pitchAngle () {
			return (Pitch.getInput() - _offset) * _conversion;
		}

		float rollAngle () {
			return (Roll.getInput() - _offset) * _conversion;
		}

		void printAngles() {
			Serial.print("Yaw: "); Serial.print(yawAngle());
			Serial.print(" Roll: "); Serial.print(rollAngle());
			Serial.print(" Pitch: "); Serial.println(pitchAngle());
			
		}

};
