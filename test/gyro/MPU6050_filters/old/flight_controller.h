#include <Wire.h> //Include the Wire.h library so we can communicate with the gyro.

#define THROTTLE_THRESHOLD 1150
#define THROTTLE_MAX 1400
#define N_SAMPLES 1000
#define MPUADDR 0x68
#define BAUDRATE 115200

/*
### channel:

LEFT SIDE TRCV
                          ^
                          |  channel 2
                          |  THROTTLE
    channel 1      <----- o ----->
    YAW                   |
                          |
                          v

RIGHT SIDE TRCV
                          ^
                          |  channel 3
                          |  PITCH
    channel 4      <----- o ----->
    ROLL                  |
                          |
                          v
*/


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 1.2;               //Gain setting for the roll P-controller (1.3)
float pid_i_gain_roll = 0.05;              //Gain setting for the roll I-controller (0.3)
float pid_d_gain_roll = 5;                //Gain setting for the roll D-controller (15)
int pid_max_roll = 100;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 1.5;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 5;                //Gain setting for the pitch D-controller.
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

// prototypes from PID.c
void calculate_pid();
