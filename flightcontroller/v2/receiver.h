class Receiver_c : Channel_c{
	
	private:

	public:
		
		Channel_c Yaw;
		Channel_c Throttle;
		Channel_c Pitch;
		Channel_c Roll;		
	
		void initReceiver() {
			Yaw.initChannel();
			Throttle.initChannel();
			Pitch.initChannel();
			Roll.initChannel();
		}
};
