class PID_c{
	
	private:

		float _error;		// error between the setpoint and the current state
		float _error_mem;	// used to calculate the difference
		float _integrative_mem;	// used to integrate the error
		float _current;	

		float _P;			// the acutal weights
		float _I;
		float _D;
	
		float _max;			// max input/output aussumed to be symmetrical!
	
	public:

		void init (float P, float I, float D, float max) {
			_P = P;
			_I = I;
			_D = D;
			_max = max;
		}

		void setP (float P) {
			_P = P;
		}

		void setI (float I) {
			_I = I;
		}

		void setD (float D) {
			_D = D;
		}
	
		float getCurrent() {
			return _current;
		}

		void setMax(float max) {
			_max = max;
		}

		float calc (float input, float setpoint) {
			_error = input - setpoint;
			_integrative_mem += _I * _error;
			
			_current = 
				( _P * _error )
				+
				_integrative_mem
				+
				( _D * ( _error - _error_mem) );

			if (_current > _max) { 
				_current = _max;
			} else if (_current < _max * -1) {
				_current = -1 * _max;
			}

			_error_mem = _error;

			return _current;
		}

		void print () {
			Serial.print("current: "); Serial.println(_current);
		}
};
