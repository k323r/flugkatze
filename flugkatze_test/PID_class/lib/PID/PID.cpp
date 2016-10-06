#include "Arduino.h"
#include "PID.h"

PID::PID(float p, float i, float d, float max) {
	_p = p;
	_i = i;
	_d = d;	

	_max = max;
}

void PID::calc() {
	Serial.print("hello, world\n");
}
