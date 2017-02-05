//Declaring Variables
byte last_channel_1;
int receiver_input_channel_1;
int counter_channel_1, start;
unsigned long timer_channel_1, esc_timer, esc_loop_timer;
unsigned long zero_timer, timer_1, current_time;

//Setup routine
void setup(){
  
  Serial.begin(9600);
  
  DDRD |= B11110000;                                 //Configure digital poort 4, 5, 6 and 7 as output
  DDRB |= B00010000;                                 //Configure digital poort 12 as output
  //Arduino Uno pins default to inputs, so they don't need to be explicitly declared as inputs
  
  PCICR |= (1 << PCIE0);                             // set PCIE0 to enable PCMSK0 scanz
  PCMSK0 |= (1 << PCINT0);                           // set PCINT0 (digital input 8) to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT1);                           // set PCINT1 (digital input 9)to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT2);                           // set PCINT2 (digital input 10)to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT3);                           // set PCINT3 (digital input 11)to trigger an interrupt on state change
  
  Serial.print("starting init\n");
  
  ///Wait until the receiver is active and the throtle is set to the lower position.
  while(receiver_input_channel_1 < 990 || receiver_input_channel_1 > 1080){
    Serial.print("waiting for the receiver init\n");
    start ++;                                        //While waiting increment start whith every loop.
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while waiting for the receiver inputs.
    PORTD |= B11110000;                              //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                         //Wait 1000us (We can use delayMicroseconds because the receiver interrupt routine is not used).
    PORTD &= B00001111;                              //Set digital poort 4, 5, 6 and 7 low.
    
    //PORTD |= B00010000;                              //Set digital poort 4, 5, 6 and 7 high.
    //delayMicroseconds(1000);                         //Wait 1000us (We can use delayMicroseconds because the receiver interrupt routine is not used).
    //PORTD &= B11101111;                              //Set digital poort 4, 5, 6 and 7 low.
    
    delay(3);                                        //Wait 3 milliseconds before the next loop.
    if(start == 125){                                //Every 125 loops (500ms).
      digitalWrite(12, !digitalRead(12));            //Change the led status.
      start = 0;                                     //Start again at 0.
    }
  }
  start = 0;
  digitalWrite(12, LOW);                             //Turn off the led.
  zero_timer = micros();                             //Set the zero_timer for the first loop.
  
  Serial.print("init finished\n");
}

//Main program loop
void loop(){
  while(zero_timer + 4000 > micros());                       //Start the pulse after 4000 micro seconds.
  zero_timer = micros();                                     //Reset the zero timer.
  PORTD |= B11110000;

  timer_channel_1 = receiver_input_channel_1 + zero_timer;   //Calculate the time when digital port 8 is set low.
  
  while(PORTD >= 16){     //Execute the loop until digital port 8 til 11 is low.
    esc_loop_timer = micros();            //Check the current time.
    if(timer_channel_1 <= esc_loop_timer) {
      PORTD &= B00001111; //When the delay time is expired, digital port 8 is set low.
    }
  }
}

//This routine is called every time input 8, 9, 10 or 11 changed state
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
}

