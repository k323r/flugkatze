#include "flugkatze.h"
#include "filter.h"
#include "state.h"
#include "channel.h"
#include "receiver.h"
#include "PID.h"

MPU6050 imu;
State_c State;
Receiver_c Receiver;
PID_c Roll;
PID_c Pitch;
PID_c Yaw; 

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
	DDRD |= B11110000;			/// set port 4 5 6 7 as outputs
	DDRB |= B00010000;			//TODO port 12 *and* 13 as output? -> is a '1' missing? -> DDRB |= B00110000

	// INIT STATE, FILTERS AND IMU
	State.init();
	Receiver.init();
	PIDTest.init(1.0, 0, 0, 5);

	// ATTACH INTERRUPTs FOR RX
	PCICR |= (1 << PCIE0);  
    PCMSK0 |= (1 << PCINT0);
    PCMSK0 |= (1 << PCINT1);
    PCMSK0 |= (1 << PCINT2);
    PCMSK0 |= (1 << PCINT3);

	// wait on user interaction & turn of ESC beeping!
	//Serial.println("init done..");	
	while (Receiver.Yaw.getInput() > 1040 || Receiver.Throttle.getInput() > 1200) {
		PORTD |= B11110000;
		delayMicroseconds(1000);
		PORTD |= B00001111;
		delay(3);
	//	Serial.println("waiting on user");
	}

	State.motorsStop();

//	Serial.println("starting\n");
		
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
void loop(){
  
	c_time = micros(); 
	State.update();
	

	// TODO Add PID controllers

	//out = PIDTest.calc(sine, sine2);

	//Serial.write((uint8_t *) &sine, sizeof(sine));
	//Serial.write((uint8_t *) &out, sizeof(out));
	//Serial.write('\n');
	
	//Serial.print("input : "); Serial.print(sine);
	//Serial.print(" output : "); Serial.println(out);
	//Serial.print(" sin(c_time2) : "); Serial.print(sine2);
	//Serial.print(" PID output : "); Serial.print(PIDTest.getCurrent());
	//Serial.print(" sum : "); Serial.println(sine + PIDTest.getCurrent());

	
	
	// TODO Init sequence needs to be redone in case the TX is offline when the copter fires up
	// Left stick in lower left corner in order to initate starting the motors
	if (Receiver.Throttle.getInput() < 1050 && Receiver.Yaw.getInput() < 1090) {
		State.motorsHold();
		Serial.println("Motors: HOLD");
	}

	if (State.motorsInitiated() && Receiver.Throttle.getInput() < 1050 && Receiver.Yaw.getInput() > 1450) {
		State.motorsStart();
		Serial.println("Motors: START");
		
		// PID controllers need to be resettet here!
		// Rollcontroll.reset();
		// Pitchcontroll.reset();
		// Yawcontroll.reset();
	}

	if (State.motorsRunning() && Receiver.Throttle.getInput() < 1050 && Receiver.Yaw.getInput() > 1800) {
		State.motorsStop();
		Serial.println("Motors: STOPING");
	}
	
	
	if (State.motorsRunning()) {
		Serial.println("Motors: RUNNING");
	}

	delay(100);
    
}


/// INTERRUPT ROUTINE
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
		
		if (Receiver.Pitch.getState() == 0) {
			Receiver.Pitch.setState(1);
			Receiver.Pitch.setTimer(c_time);
		}

	} else if (Receiver.Pitch.getState() == 1) {
		
		Receiver.Pitch.setState(0);
		Receiver.Pitch.setInput(c_time - Receiver.Pitch.getTimer());

	}

	if (PINB & B00001000) {
		
		if (Receiver.Roll.getState() == 0) {
			Receiver.Roll.setState(1);
			Receiver.Roll.setTimer(c_time);
		}

	} else if (Receiver.Roll.getState() == 1) {
		
		Receiver.Roll.setState(0);
		Receiver.Roll.setInput(c_time - Receiver.Roll.getTimer());

	}
}
