class Receiver_c : Channel_c{
	
	private:

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
};
