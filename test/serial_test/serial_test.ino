struct test {
    float a;
} blubb;

float old_time = 0.0, current_time = 0.0, deltaT = 0.0;


void my_func (struct test* _test);

void setup(){

  Serial.begin(9600);
  Serial.print("init\n");

}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){

deltaT = current_time - old_time;
old_time = current_time;
current_time = micros();

deltaT = deltaT / 1000000;

my_func(&blubb);

    Serial.print("current delta T: ");
    Serial.println(deltaT);
    Serial.println(blubb.a);
    Serial.println("\n");
	delay(250);
}

void my_func (struct test* _test) {
    _test->a = deltaT;
}
