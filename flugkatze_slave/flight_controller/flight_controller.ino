#include <Wire.h> //Include the Wire.h library so we can communicate with the gyro.

#define THROTTLE_THRESHOLD 1150
#define THROTTLE_MAX 1400
#define N_SAMPLES 1000
#define MPUADDR 0x68
#define BAUDRATE 115200

// PROTOTYPES
void imu ();
void calculate_pid();


/*
### channel:

LEFT SIDE TX
                          ^
                          |  channel 2
                          |  THROTTLE
    channel 1      <----- o ----->
    YAW                   |
                          |
                          v

RIGHT SIDE TX
                          ^
                          |  channel 3
                          |  PITCH
    channel 4      <----- o ----->
    ROLL                  |
                          |
*/

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 1.0;               //Gain setting for the roll P-controller (1.3)
float pid_i_gain_roll = 0.0;              //Gain setting for the roll I-controller (0.3)
float pid_d_gain_roll = 0.0;                //Gain setting for the roll D-controller (15)
int pid_max_roll = 100;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 1.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.0;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 100;                     //Maximum output of the PID-controller (+/-)

float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// variables to handle the receiver
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
int esc_1, esc_2, esc_3, esc_4;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MISC variables to handle the gyro and acc inputs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float throttle_rel = 0.0;
int cal_int, start;
int throttle, battery_voltage;
unsigned long loop_timer;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
byte highByte, lowByte;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// The following data structure is used to store all essential data and send it to the rapsberry pi
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct Flight_data {
    float ax;
    float ay;
    float az;

    float temp;

    float gx;
    float gy;
    float gz;

    int throttle;
    int roll;
    int pitch;
    int yaw;
} flight_data;

char len_struct = sizeof(flight_data);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){

    Serial.begin(BAUDRATE);
    Serial.print("init\n");

    Serial.print("length of struct: ");
    Serial.print(len_struct,DEC);
    Serial.print("\n");

    Wire.begin();                                                //Start the I2C as master.

    DDRD |= B11110000;                                           //Configure digital poort 4, 5, 6 and 7 as output.
    DDRB |= B00010000;                                           //Configure digital poort 12 and 13 as output.
    //Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs.

    //Use the led on the Arduino for startup indication
    digitalWrite(12,HIGH);                                       //Turn on the warning led.
    delay(250);                                                 //Wait 2 second befor continuing.

    // wake up the imu
    Wire.begin();
    Wire.beginTransmission(MPUADDR);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0);     // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
    delay(250);                                                  //Give the gyro time to start.

    Serial.print("starting gyro calibration\n");

    //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
    for (cal_int = 0; cal_int < N_SAMPLES ; cal_int ++){              //Take 2000 readings for calibration.
        if(cal_int % 15 == 0) {
            digitalWrite(12, !digitalRead(12));   //Change the led status to indicate calibration.
        }
        imu();                                           //Read the gyro output.
        gyro_roll_cal += flight_data.gx;                                //Ad roll value to gyro_roll_cal.
        gyro_pitch_cal += flight_data.gy;                              //Ad pitch value to gyro_pitch_cal.
        gyro_yaw_cal += flight_data.gz;                                  //Ad yaw value to gyro_yaw_cal.
        //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while calibrating the gyro.
        PORTD |= B11110000;                                        //Set digital poort 4, 5, 6 and 7 high.
        delayMicroseconds(1000);                                   //Wait 1000us.
        PORTD &= B00001111;                                        //Set digital poort 4, 5, 6 and 7 low.
        delay(3);                                                  //Wait 3 milliseconds before the next loop.
    }

    //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
    gyro_roll_cal /= N_SAMPLES;                                       //Divide the roll total by 2000.
    gyro_pitch_cal /= N_SAMPLES;                                      //Divide the pitch total by 2000.
    gyro_yaw_cal /= N_SAMPLES;                                        //Divide the yaw total by 2000.

    Serial.print("roll offset: ");
    Serial.print(gyro_roll_cal);

    Serial.print("done calibrating gyro\n");

    PCICR |= (1 << PCIE0);                                       //Set PCIE0 to enable PCMSK0 scan.
    PCMSK0 |= (1 << PCINT0);                                     //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
    PCMSK0 |= (1 << PCINT1);                                     //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
    PCMSK0 |= (1 << PCINT2);                                     //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
    PCMSK0 |= (1 << PCINT3);                                     //Set PCINT3 (digital input 11)to trigger an interrupt on state change.

    Serial.print("waiting on user input\n");

    //Wait until the receiver is active and the throtle is set to the lower position.
    while(receiver_input_channel_2 < 900 || receiver_input_channel_2 > 1040 || receiver_input_channel_1 < 1400){
      start ++;
                                                 //While waiting increment start whith every loop.
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
    imu();

    flight_data.throttle = receiver_input_channel_2;
    flight_data.roll = receiver_input_channel_4;
    flight_data.pitch = receiver_input_channel_3;
    flight_data.yaw = receiver_input_channel_1;

    // send telemetry (stored in the flight_data struct)
    char len_struct = sizeof(flight_data);
    char aux[len_struct];                 // auxiliary buffer used to serialize the struct -> put init before loop!!!!!!!
    memcpy(&aux, &flight_data, len_struct);   // copy the struct into the new buffer
    Serial.write('S');                    // starting byte to ensure data integrity
    Serial.write((uint8_t *) &aux, len_struct);  // send the actual data
    Serial.write('E');                    // end byte to ensure data integrity

    // gyro_roll_input = (gyro_roll_input * 0.8) + ((flight_data.gx / 57.14286) * 0.2);            //Gyro pid input is deg/sec.
    // gyro_pitch_input = (gyro_pitch_input * 0.8) + ((flight_data.gy / 57.14286) * 0.2);         //Gyro pid input is deg/sec.
    // gyro_yaw_input = (gyro_yaw_input * 0.8) + ((flight_data.gz / 57.14286) * 0.2);               //Gyro pid input is deg/sec.

	gyro_roll_input = 0.0;           //Gyro pid input is deg/sec.
    gyro_pitch_input = 0.0;         //Gyro pid input is deg/sec.
    gyro_yaw_input = 0.0;               //Gyro pid input is deg/sec.


    //print_signals();

    //For starting the motors: throttle low and yaw left (step 1).
    if(receiver_input_channel_2 < 1050 && receiver_input_channel_1 < 1090) {
        start = 1;
        Serial.print(" stage 1 ");
    }

    //When yaw stick is back in the center position start the motors (step 2).
    if(start == 1 && receiver_input_channel_2 < 1050 && receiver_input_channel_1 > 1450){
        start = 2;
        Serial.print(" stage 2 ");
        //Reset the pid controllers for a bumpless start.
        pid_i_mem_roll = 0;
        pid_last_roll_d_error = 0;
        pid_i_mem_pitch = 0;
        pid_last_pitch_d_error = 0;
        pid_i_mem_yaw = 0;
        pid_last_yaw_d_error = 0;
    }

    //Stopping the motors: throttle low and yaw right.
    if (start == 2 && receiver_input_channel_2 < 1050 && receiver_input_channel_1 > 1800) {
        start = 0;
        Serial.print("stage 0 ");
    }

    //The PID set point in degrees per second is determined by the roll receiver input.
    //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
    pid_roll_setpoint = 0;

    //We need a little dead band of 16us for better results.
    if (receiver_input_channel_4 > 1508) {
        pid_roll_setpoint = (receiver_input_channel_4 - 1508)/3.0;
    } else if(receiver_input_channel_4 < 1492) {
        pid_roll_setpoint = (receiver_input_channel_4 - 1492)/3.0;
    }


    //  if(receiver_input_channel_4 > 1508) pid_roll_setpoint = (receiver_input_channel_4 - 1508)/3.0;
    //  else if(receiver_input_channel_4 < 1492) pid_roll_setpoint = (receiver_input_channel_4 - 1492)/3.0;

    //The PID set point in degrees per second is determined by the pitch receiver input.
    //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
    pid_pitch_setpoint = 0;
    //We need a little dead band of 16us for better results.
    if (receiver_input_channel_3 > 1508) {
        pid_pitch_setpoint = (receiver_input_channel_3 - 1508)/3.0;
    } else if (receiver_input_channel_3 < 1492) {
        pid_pitch_setpoint = (receiver_input_channel_3 - 1492)/3.0;
    }

    //The PID set point in degrees per second is determined by the yaw receiver input.
    //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
    pid_yaw_setpoint = 0;
    //We need a little dead band of 16us for better results.
    if(receiver_input_channel_1 > 1050){ //Do not yaw when turning off the motors.
        if(receiver_input_channel_1 > 1508) {
            pid_yaw_setpoint = (receiver_input_channel_1 - 1508)/3.0;
        } else if(receiver_input_channel_1 < 1492) {
            pid_yaw_setpoint = (receiver_input_channel_1 - 1492)/3.0;
        }
    }
    //PID inputs are known. So we can calculate the pid output.
    calculate_pid();

    //The battery voltage is needed for compensation.
    //A complementary filter is used to reduce noise.
    //0.09853 = 0.08 * 1.2317.
    // battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;

    //Turn on the led if battery voltage is to low.
    //if(battery_voltage < 1050 && battery_voltage > 600)digitalWrite(12, HIGH);

    throttle = receiver_input_channel_2;                                      //We need the throttle signal as a base signal.


//  print_signals();
//  throttle_rel = (throttle - 1000) / 100.0;
// print_signals();

//  Serial.print("throttle relative: "); Serial.print(throttle_rel);


    if (start == 2){                                                          //The motors are started.
        if (throttle > THROTTLE_MAX) {
            throttle = THROTTLE_MAX;                                   //We need some room to keep full control at full throttle.
        }

        // switched the signes brfore pid_output_roll!
        //    esc_1 = throttle - pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
        //    esc_2 = throttle + pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
        //    esc_3 = throttle + pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
        //    esc_4 = throttle - pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)
        // old version

        esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
        esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
        esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
        esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

        //    if (battery_voltage < 1240 && battery_voltage > 800){                   //Is the battery connected?
        //      esc_1 += esc_1 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-1 pulse for voltage drop.
        //      esc_2 += esc_2 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-2 pulse for voltage drop.
        //      esc_3 += esc_3 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-3 pulse for voltage drop.
        //      esc_4 += esc_4 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-4 pulse for voltage drop.
        //    }

        if (esc_1 < THROTTLE_THRESHOLD) {
            esc_1 = THROTTLE_THRESHOLD;                                         //Keep the motors running.
        }

        if (esc_2 < THROTTLE_THRESHOLD) {
            esc_2 = THROTTLE_THRESHOLD;                                         //Keep the motors running.
        }

        if (esc_3 < THROTTLE_THRESHOLD) {
            esc_3 = THROTTLE_THRESHOLD;                                         //Keep the motors running.
        }

        if (esc_4 < THROTTLE_THRESHOLD) {
            esc_4 = THROTTLE_THRESHOLD;                                         //Keep the motors running.
        }

        if(esc_1 > THROTTLE_MAX) {
            esc_1 = THROTTLE_MAX;                                           //Limit the esc-1 pulse to 2000us.
        }

        if(esc_2 > THROTTLE_MAX) {
            esc_2 = THROTTLE_MAX;                                           //Limit the esc-2 pulse to 2000us.
        }

        if(esc_3 > THROTTLE_MAX) {
            esc_3 = THROTTLE_MAX;                                           //Limit the esc-3 pulse to 2000us.
        }

        if(esc_4 > THROTTLE_MAX) {
            esc_4 = THROTTLE_MAX;                                           //Limit the esc-4 pulse to 2000us.
        }
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

        if(timer_channel_1 <= esc_loop_timer) {
            PORTD &= B11101111;                //Set digital output 4 to low if the time is expired.
        }

        if(timer_channel_2 <= esc_loop_timer) {
            PORTD &= B11011111;                //Set digital output 5 to low if the time is expired.
        }

        if(timer_channel_3 <= esc_loop_timer) {
            PORTD &= B10111111;                //Set digital output 6 to low if the time is expired.
        }

        if(timer_channel_4 <= esc_loop_timer) {
            PORTD &= B01111111;                //Set digital output 7 to low if the time is expired.
        }
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
    } else if(last_channel_1 == 1){                                //Input 8 is not high and changed from 1 to 0
        last_channel_1 = 0;                                        //Remember current input state
        receiver_input_channel_1 = current_time - timer_1;         //Channel 1 is current_time - timer_1
    }

    //Channel 2=========================================
    if(PINB & B00000010 ){                                       //Is input 9 high?
        if(last_channel_2 == 0){                                   //Input 9 changed from 0 to 1
            last_channel_2 = 1;                                      //Remember current input state
            timer_2 = current_time;                                  //Set timer_2 to current_time
        }
    } else if(last_channel_2 == 1){                                //Input 9 is not high and changed from 1 to 0
        last_channel_2 = 0;                                        //Remember current input state
        receiver_input_channel_2 = current_time - timer_2;         //Channel 2 is current_time - timer_2
    }

    //Channel 3=========================================
    if(PINB & B00000100 ){                                       //Is input 10 high?
        if(last_channel_3 == 0){                                   //Input 10 changed from 0 to 1
            last_channel_3 = 1;                                      //Remember current input state
            timer_3 = current_time;                                  //Set timer_3 to current_time
        }
    } else if(last_channel_3 == 1){                                //Input 10 is not high and changed from 1 to 0
        last_channel_3 = 0;                                        //Remember current input state
        receiver_input_channel_3 = current_time - timer_3;         //Channel 3 is current_time - timer_3
    }

    //Channel 4=========================================
    if(PINB & B00001000 ){                                       //Is input 11 high?
        if(last_channel_4 == 0){                                   //Input 11 changed from 0 to 1
            last_channel_4 = 1;                                      //Remember current input state
            timer_4 = current_time;                                  //Set timer_4 to current_time
        }
    } else if(last_channel_4 == 1){                                //Input 11 is not high and changed from 1 to 0
        last_channel_4 = 0;                                        //Remember current input state
        receiver_input_channel_4 = current_time - timer_4;         //Channel 4 is current_time - timer_4
    }
}



void imu () {
    Wire.beginTransmission(MPUADDR);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPUADDR, 14, true);  // request a total of 14 registers
    flight_data.ax = (Wire.read() << 8) | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    flight_data.ay = (Wire.read() << 8) | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    flight_data.az = (Wire.read() << 8) | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

    flight_data.temp = (Wire.read() << 8) | Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)

    flight_data.gx = (Wire.read() << 8) | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    flight_data.gy = (Wire.read() << 8) | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    flight_data.gz = (Wire.read() << 8) | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    if (cal_int == N_SAMPLES) {
        flight_data.gx -= gyro_roll_cal;
        flight_data.gy -= gyro_pitch_cal;
        flight_data.gz -= gyro_yaw_cal;
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for calculating pid outputs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void calculate_pid(){

///////////////////////////////////////////////////////
///////// ROLL calculations
///////////////////////////////////////////////////////
    pid_error_temp = gyro_roll_input - pid_roll_setpoint;
    pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;

    if(pid_i_mem_roll > pid_max_roll) {
        pid_i_mem_roll = pid_max_roll;
    } else if(pid_i_mem_roll < (pid_max_roll * -1)) {
        pid_i_mem_roll = pid_max_roll * -1;
    }

    pid_output_roll = (( pid_p_gain_roll * pid_error_temp ) + pid_i_mem_roll + ( pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error))) * -1; // maybe *-1 ?
    // old version
    if(pid_output_roll > pid_max_roll) {
        pid_output_roll = pid_max_roll;
    } else if(pid_output_roll < (pid_max_roll * -1)) {
        pid_output_roll = pid_max_roll * -1;
    }

    pid_last_roll_d_error = pid_error_temp;

///////////////////////////////////////////////////////
///////// PITCH calculations
///////////////////////////////////////////////////////
    pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
    pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
    if(pid_i_mem_pitch > pid_max_pitch) {
        pid_i_mem_pitch = pid_max_pitch;
    } else if(pid_i_mem_pitch < (pid_max_pitch * -1)) {
        pid_i_mem_pitch = pid_max_pitch * -1;
    }

    pid_output_pitch = (( pid_p_gain_pitch * pid_error_temp )  + pid_i_mem_pitch + ( pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error)));
    if(pid_output_pitch > pid_max_pitch) {
        pid_output_pitch = pid_max_pitch;
    } else if(pid_output_pitch < (pid_max_pitch * -1)) {
        pid_output_pitch = pid_max_pitch * -1;
    }

    pid_last_pitch_d_error = pid_error_temp;

///////////////////////////////////////////////////////
///////// YAW calculations
///////////////////////////////////////////////////////
    pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
    pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
    if(pid_i_mem_yaw > pid_max_yaw) {
        pid_i_mem_yaw = pid_max_yaw;
    } else if(pid_i_mem_yaw < pid_max_yaw * -1) {
        pid_i_mem_yaw = pid_max_yaw * -1;
    }

    pid_output_yaw = (( pid_p_gain_yaw * pid_error_temp ) + pid_i_mem_yaw + ( pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error)));
    if(pid_output_yaw > pid_max_yaw) {
        pid_output_yaw = pid_max_yaw;
    } else if(pid_output_yaw < pid_max_yaw * -1) {
        pid_output_yaw = pid_max_yaw * -1;
    }

    pid_last_yaw_d_error = pid_error_temp;
}
