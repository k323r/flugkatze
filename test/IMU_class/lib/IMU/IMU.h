#ifndef PID_h
#define PID_h

#include "Arduino.h"

class PID {


	public:
		PID(float p, float i, float d, float max);
		void calc(float setpoint, float input);


	private:
		float _p, _i, _d, _max;

};

#endif
