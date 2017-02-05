#include "PID.h"

PID pid(1.0, 1.0, 1.0);

void setup () {
	Serial.begin(115200);
	Serial.print("starting execution\n");
}

void loop() {
	
	pid.calc();
	delay(3000);
}
