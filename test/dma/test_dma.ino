class test_dma {

	private:
		float private_data;	

	public:
		float public_data;

		void init() {
			public_data = 27;
			private_data = 28;	
		}

		float * return_adress_public () {
			return &public_data;
		}
};

test_dma bla;

//Setup routine
void setup(){

	Serial.begin(9600);

	bla.init();

	Serial.print("init done\n");
}

//Main program loop
void loop(){

	Serial.print("address: "); Serial.print(bla.return_adress_public());
	delay(1000);

}


