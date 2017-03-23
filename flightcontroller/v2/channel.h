class Channel_c{
	
	private:
		
		byte _state;
		unsigned long _timer;
		unsigned int _input;
		int _counter;	
	
	public:


		void initChannel() {
			_state = 0;
			_timer = 0;
			_input = 0;
			_counter = 0;		
		}

		byte getState () {
			return(_state);
		}

		void setState (byte state) {
			_state = state;
		}
	
		unsigned long getTimer () {
			return (_timer);
		}

		void setTimer(unsigned long c_time) {
			_timer = c_time;
		}

		unsigned long getInput () {
			return (_input);
		}
	
		void setInput(unsigned int input) {
			_input = input;
		}
		
};
