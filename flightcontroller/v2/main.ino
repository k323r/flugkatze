#include "flugkatze.h"
#include "filter.h"
#include "state.h"
#include "channel.h"
#include "receiver.h"

MPU6050 imu;
State_c State;
Receiver_c Receiver;

unsigned long c_time;

/*
	ROLLING AXIS -> X AXIS of the GYRO
	PITCHING AXIS -> Y AXIS of the GYRO
	YAWING AXis -> Z AXIS of the GYRO
*/


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){


	Serial.begin(BAUDRATE);
    Wire.begin();                                                //Start the I2C as master.

	// SET PINS
	DDRD |= B11110000;
	DDRB |= B00010000;

	// INIT STATE, FILTERS AND IMU
	State.initState();
	State.initIMU();
	State.initFilters(10, 10, 10);

	Receiver.initReceiver();

	// ATTACH INTERRUPTs FOR RX
	PCICR |= (1 << PCIE0);  
    PCMSK0 |= (1 << PCINT0);
    PCMSK0 |= (1 << PCINT1);
    PCMSK0 |= (1 << PCINT2);
    PCMSK0 |= (1 << PCINT3);

		
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
void loop(){
   	
	State.updateState();
	State.printState();

	//delay(100);
    
}

ISR(PCINT0_vect) {

	c_time = micros();
	
	if (PINB & B00000001) {
		
		if (Receiver.Yaw.getState() == 0) {
			Receiver.Yaw.setState(1);
			Receiver.Yaw.setTimer(c_time);
		}

	} else if (Receiver.Yaw.getState() == 1) {
		
		Receiver.Yaw.setState(0);
		Receiver.Yaw.setInput(c_time - Receiver.Yaw.getTimer());

	}
	
	if (PINB & B00000010) {
		
		if (Receiver.Throttle.getState() == 0) {
			Receiver.Throttle.setState(1);
			Receiver.Throttle.setTimer(c_time);
		}

	} else if (Receiver.Throttle.getState() == 1) {
		
		Receiver.Throttle.setState(0);
		Receiver.Throttle.setInput(c_time - Receiver.Throttle.getTimer());

	}

	if (PINB & B00000100) {
		
		if (Receiver.Roll.getState() == 0) {
			Receiver.Roll.setState(1);
			Receiver.Roll.setTimer(c_time);
		}

	} else if (Receiver.Roll.getState() == 1) {
		
		Receiver.Roll.setState(0);
		Receiver.Roll.setInput(c_time - Receiver.Roll.getTimer());

	}

	if (PINB & B00001000) {
		
		if (Receiver.Pitch.getState() == 0) {
			Receiver.Pitch.setState(1);
			Receiver.Pitch.setTimer(c_time);
		}

	} else if (Receiver.Pitch.getState() == 1) {
		
		Receiver.Pitch.setState(0);
		Receiver.Pitch.setInput(c_time - Receiver.Pitch.getTimer());

	}
}
