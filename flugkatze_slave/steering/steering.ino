#include <Wire.h> //Include the Wire.h library so we can communicate with the gyro.

#define THROTTLE_THRESHOLD 1100
#define THROTTLE_MAX 1500
#define NUM_GYRO_SAMPLES 1000
#define SIGMA_GYRO_FACTOR 3

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring Variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
int esc_1, esc_2, esc_3, esc_4;
int throttle, battery_voltage;
float throttle_rel = 0.0;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
int cal_int, start;
unsigned long loop_timer;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
double gyro_roll_sigma, gyro_pitch_sigma, gyro_yaw_sigma;
double gyro_roll_threshold, gyro_pitch_threshold, gyro_yaw_threshold;
byte highByte, lowByte;
byte print_counter = 0;

float gyro_roll_input, gyro_pitch_input, gyro_yaw_input;

void gyro_signals(){
  Wire.beginTransmission(105);                                 //Start communication with the gyro (adress 1101001)
  Wire.write(168);                                             //Start reading @ register 28h and auto increment with every read
  Wire.endTransmission();                                      //End the transmission
  Wire.requestFrom(105, 6);                                    //Request 6 bytes from the gyro
  while(Wire.available() < 6);                                 //Wait until the 6 bytes are received
  lowByte = Wire.read();                                       //First received byte is the low part of the angular data
  highByte = Wire.read();                                      //Second received byte is the high part of the angular data
  gyro_roll = ((highByte<<8)|lowByte);                         //Multiply highByte by 256 (shift left by 8) and ad lowByte
/*
  if(cal_int == NUM_GYRO_SAMPLES) {
    gyro_roll -= gyro_roll_cal;               //Only compensate after the calibration
    if (abs(gyro_roll) < gyro_roll_threshold) {
      gyro_roll = 0;
    }
  }
*/
  lowByte = Wire.read();                                       //First received byte is the low part of the angular data
  highByte = Wire.read();                                      //Second received byte is the high part of the angular data
  gyro_pitch = ((highByte<<8)|lowByte);                        //Multiply highByte by 256 (shift left by 8) and ad lowByte
/*  gyro_pitch *= -1;                                            //Invert axis
  if(cal_int == NUM_GYRO_SAMPLES)gyro_pitch -= gyro_pitch_cal;             //Only compensate after the calibration
*/
  lowByte = Wire.read();                                       //First received byte is the low part of the angular data
  highByte = Wire.read();                                      //Second received byte is the high part of the angular data
  gyro_yaw = ((highByte<<8)|lowByte);                          //Multiply highByte by 256 (shift left by 8) and ad lowByte
/*
  gyro_yaw *= -1;                                              //Invert axis
  if(cal_int == NUM_GYRO_SAMPLES)gyro_yaw -= gyro_yaw_cal;                 //Only compensate after the calibration
*/
  if(cal_int == NUM_GYRO_SAMPLES) {
    gyro_roll -= gyro_roll_cal;               //Only compensate after the calibration
    if (abs(gyro_roll) < gyro_roll_threshold) {
      gyro_roll = 0;
    }

    gyro_pitch -= gyro_pitch_cal;
    if (abs(gyro_pitch) < gyro_pitch_threshold) {
      gyro_pitch = 0;
    }

    gyro_yaw -= gyro_yaw_cal;
    if (abs(gyro_yaw) < gyro_yaw_threshold) {
      gyro_yaw = 0;
    }
  }


}

void print_signals(){
  Serial.print(gyro_roll_input);
  Serial.print(" ");
  Serial.print(gyro_pitch_input);
  Serial.print(" ");
  Serial.print(gyro_pitch_input);
  Serial.print(" ");
  Serial.print(receiver_input_channel_1);
  Serial.print(" ");
  Serial.print(receiver_input_channel_2);
  Serial.print(" ");
  Serial.print(receiver_input_channel_3);
  Serial.print("  ");
  Serial.println(receiver_input_channel_4);
//  Serial.print("\n");
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){

  Serial.begin(38400);
  Serial.print("init\n");

  Wire.begin();                                                //Start the I2C as master.

  DDRD |= B11110000;                                           //Configure digital poort 4, 5, 6 and 7 as output.
  DDRB |= B00010000;                                           //Configure digital poort 12 and 13 as output.
  //Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs.

  //Use the led on the Arduino for startup indication
  digitalWrite(12,HIGH);                                       //Turn on the warning led.
  delay(250);                                                 //Wait 2 second befor continuing.

  Wire.beginTransmission(105);                                 //Start communication with the gyro (adress 1101001)
  Wire.write(0x20);                                            //We want to write to register 1 (20 hex)
  Wire.write(0x0F);                                            //Set the register bits as 00001111 (Turn on the gyro and enable all axis)
  Wire.endTransmission();                                      //End the transmission with the gyro

  Wire.beginTransmission(105);                                 //Start communication with the gyro (adress 1101001)
  Wire.write(0x23);                                            //We want to write to register 4 (23 hex)
  Wire.write(0x80);                                            // set 250 dps
//  Wire.write(0x90);                                            //Set the register bits as 10010000 (Block Data Update active & 500dps full scale)
  Wire.endTransmission();                                      //End the transmission with the gyro

  delay(250);                                                  //Give the gyro time to start.

  Serial.print("starting gyro calibration\n");

  //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
  for (cal_int = 0; cal_int < NUM_GYRO_SAMPLES ; cal_int ++){              //Take 2000 readings for calibration.
    if(cal_int % 15 == 0)digitalWrite(12, !digitalRead(12));   //Change the led status to indicate calibration.
    gyro_signals();                                           //Read the gyro output.
    gyro_roll_cal += gyro_roll;                                //Ad roll value to gyro_roll_cal.
    gyro_pitch_cal += gyro_pitch;                              //Ad pitch value to gyro_pitch_cal.
    gyro_yaw_cal += gyro_yaw;

    // calculate the standard deviation as well
    gyro_roll_sigma += gyro_roll * gyro_roll;
    gyro_pitch_sigma += gyro_pitch * gyro_pitch;
    gyro_yaw_sigma += gyro_yaw * gyro_yaw;
                        //Ad yaw value to gyro_yaw_cal.
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while calibrating the gyro.
    PORTD |= B11110000;                                        //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                   //Wait 1000us.
    PORTD &= B00001111;                                        //Set digital poort 4, 5, 6 and 7 low.
    delay(3);                                                  //Wait 3 milliseconds before the next loop.
  }
  //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
  gyro_roll_cal /= NUM_GYRO_SAMPLES;                                       //Divide the roll total by 2000.
  gyro_pitch_cal /= NUM_GYRO_SAMPLES;                                      //Divide the pitch total by 2000.
  gyro_yaw_cal /= NUM_GYRO_SAMPLES;

  gyro_roll_threshold = sqrt((double(gyro_roll_sigma) / NUM_GYRO_SAMPLES) - (gyro_roll_cal * gyro_roll_cal)) * SIGMA_GYRO_FACTOR;
  gyro_pitch_threshold = sqrt((double(gyro_pitch_sigma) / NUM_GYRO_SAMPLES) - (gyro_pitch_cal * gyro_pitch_cal)) * SIGMA_GYRO_FACTOR;
  gyro_yaw_threshold = sqrt((double(gyro_yaw_sigma) / NUM_GYRO_SAMPLES) - (gyro_yaw_cal * gyro_yaw_cal)) * SIGMA_GYRO_FACTOR;


  Serial.print("done calibrating gyro\n");

  PCICR |= (1 << PCIE0);                                       //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                     //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);                                     //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);                                     //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT3);                                     //Set PCINT3 (digital input 11)to trigger an interrupt on state change.

  Serial.print("waiting on user input\n");

  //Wait until the receiver is active and the throtle is set to the lower position.
  while(receiver_input_channel_3 < 990 || receiver_input_channel_3 > 1020 || receiver_input_channel_4 < 1400){
    start ++;                                                  //While waiting increment start whith every loop.
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while waiting for the receiver inputs.
    PORTD |= B11110000;                                        //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                   //Wait 1000us.
    PORTD &= B00001111;                                        //Set digital poort 4, 5, 6 and 7 low.
    delay(3);                                                  //Wait 3 milliseconds before the next loop.
    if(start == 125){                                          //Every 125 loops (500ms).
      digitalWrite(12, !digitalRead(12));                      //Change the led status.
      start = 0;                                               //Start again at 0.
    }
  }
  start = 0;                                                   //Set start back to 0.

  //When everything is done, turn off the led.
  digitalWrite(12,LOW);                                        //Turn off the warning led.
  Serial.print("init done\n");
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){
  //Let's get the current gyro data and scale it to degrees per second for the pid calculations.
  gyro_signals();
  gyro_roll_input = (gyro_roll_input * 0.8) + ((gyro_roll / 57.14286) * 0.2);            //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.8) + ((gyro_pitch / 57.14286) * 0.2);         //Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.8) + ((gyro_yaw / 57.14286) * 0.2);               //Gyro pid input is deg/sec.

  print_signals();
    //For starting the motors: throttle low and yaw left (step 1).
  if(receiver_input_channel_3 < 1050 && receiver_input_channel_4 < 1090) {
    start = 1;
  }
  //When yaw stick is back in the center position start the motors (step 2).
  if(start == 1 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1450){
    start = 2;
  }

  //Stopping the motors: throttle low and yaw right.
  if(start == 2 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1900) {
    start = 0;
  }

  throttle = receiver_input_channel_3;                                      //We need the throttle signal as a base signal.

  if (start == 2){                                                          //The motors are started.
    if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.
    //esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
    //esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
    //esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    //esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

    if (esc_1 < THROTTLE_THRESHOLD) esc_1 = THROTTLE_THRESHOLD;                                         //Keep the motors running.
    if (esc_2 < THROTTLE_THRESHOLD) esc_2 = THROTTLE_THRESHOLD;                                         //Keep the motors running.
    if (esc_3 < THROTTLE_THRESHOLD) esc_3 = THROTTLE_THRESHOLD;                                         //Keep the motors running.
    if (esc_4 < THROTTLE_THRESHOLD) esc_4 = THROTTLE_THRESHOLD;                                         //Keep the motors running.

    if(esc_1 > THROTTLE_MAX)esc_1 = THROTTLE_MAX;                                           //Limit the esc-1 pulse to 2000us.
    if(esc_2 > THROTTLE_MAX)esc_2 = THROTTLE_MAX;                                           //Limit the esc-2 pulse to 2000us.
    if(esc_3 > THROTTLE_MAX)esc_3 = THROTTLE_MAX;                                           //Limit the esc-3 pulse to 2000us.
    if(esc_4 > THROTTLE_MAX)esc_4 = THROTTLE_MAX;                                           //Limit the esc-4 pulse to 2000us.
  }

  else{
    esc_1 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-1.
    esc_2 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-2.
    esc_3 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-3.
    esc_4 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-4.
  }

  //All the information for controlling the motor's is available.
  //The refresh rate is 250Hz. That means the esc's need there pulse every 4ms.
  while(micros() - loop_timer < 4000);                                      //We wait until 4000us are passed.
  loop_timer = micros();                                                    //Set the timer for the next loop.

  PORTD |= B11110000;                                                       //Set digital outputs 4,5,6 and 7 high.
  timer_channel_1 = esc_1 + loop_timer;                                     //Calculate the time of the faling edge of the esc-1 pulse.
  timer_channel_2 = esc_2 + loop_timer;                                     //Calculate the time of the faling edge of the esc-2 pulse.
  timer_channel_3 = esc_3 + loop_timer;                                     //Calculate the time of the faling edge of the esc-3 pulse.
  timer_channel_4 = esc_4 + loop_timer;                                     //Calculate the time of the faling edge of the esc-4 pulse.

  while(PORTD >= 16){                                                       //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros();                                              //Read the current time.
    if(timer_channel_1 <= esc_loop_timer)PORTD &= B11101111;                //Set digital output 4 to low if the time is expired.
    if(timer_channel_2 <= esc_loop_timer)PORTD &= B11011111;                //Set digital output 5 to low if the time is expired.
    if(timer_channel_3 <= esc_loop_timer)PORTD &= B10111111;                //Set digital output 6 to low if the time is expired.
    if(timer_channel_4 <= esc_loop_timer)PORTD &= B01111111;                //Set digital output 7 to low if the time is expired.
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This routine is called every time input 8, 9, 10 or 11 changed state
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(PCINT0_vect){
  current_time = micros();
  //Channel 1=========================================
  if(PINB & B00000001){                                        //Is input 8 high?
    if(last_channel_1 == 0){                                   //Input 8 changed from 0 to 1
      last_channel_1 = 1;                                      //Remember current input state
      timer_1 = current_time;                                  //Set timer_1 to current_time
    }
  }
  else if(last_channel_1 == 1){                                //Input 8 is not high and changed from 1 to 0
    last_channel_1 = 0;                                        //Remember current input state
    receiver_input_channel_1 = current_time - timer_1;         //Channel 1 is current_time - timer_1
  }
  //Channel 2=========================================
  if(PINB & B00000010 ){                                       //Is input 9 high?
    if(last_channel_2 == 0){                                   //Input 9 changed from 0 to 1
      last_channel_2 = 1;                                      //Remember current input state
      timer_2 = current_time;                                  //Set timer_2 to current_time
    }
  }
  else if(last_channel_2 == 1){                                //Input 9 is not high and changed from 1 to 0
    last_channel_2 = 0;                                        //Remember current input state
    receiver_input_channel_2 = current_time - timer_2;         //Channel 2 is current_time - timer_2
  }
  //Channel 3=========================================
  if(PINB & B00000100 ){                                       //Is input 10 high?
    if(last_channel_3 == 0){                                   //Input 10 changed from 0 to 1
      last_channel_3 = 1;                                      //Remember current input state
      timer_3 = current_time;                                  //Set timer_3 to current_time
    }
  }
  else if(last_channel_3 == 1){                                //Input 10 is not high and changed from 1 to 0
    last_channel_3 = 0;                                        //Remember current input state
    receiver_input_channel_3 = current_time - timer_3;         //Channel 3 is current_time - timer_3

  }
  //Channel 4=========================================
  if(PINB & B00001000 ){                                       //Is input 11 high?
    if(last_channel_4 == 0){                                   //Input 11 changed from 0 to 1
      last_channel_4 = 1;                                      //Remember current input state
      timer_4 = current_time;                                  //Set timer_4 to current_time
    }
  }
  else if(last_channel_4 == 1){                                //Input 11 is not high and changed from 1 to 0
    last_channel_4 = 0;                                        //Remember current input state
    receiver_input_channel_4 = current_time - timer_4;         //Channel 4 is current_time - timer_4
  }
}
