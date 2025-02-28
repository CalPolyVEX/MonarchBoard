

//Arduino Tone library uses TC5
//Encoder SPI uses SERCOM4
//VEX Brain communication uses SERCOM5
//LED pattern delays use TC3

int num_bytes_received;  //number of bytes received from the VEX brain so far
int num_error_counter = 0;  //number of data reception errors

void manage_led_commands(uint8_t val){
  if (val == 255){
    // do nothing. default value
  }
}

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(13, OUTPUT); //set LED2 pin to output
  pinMode(12, OUTPUT); //set LED1 pin to output
  //digitalWrite(12, HIGH);
  pinMode(5, OUTPUT);  //set SERCOM1.0 to output
  Serial.begin(115200);
  //init_vex_brain_serial();
  //led_setup();
  //init_encoders();
  init_led_timer();
  //setup_otos();
  //digitalWrite(12, HIGH);
  //initialize_lox();
  setup_VL53L0X();
  //set PA19 (SERCOM1.3) to input with pullup
  PORT->Group[0].PINCFG[19].reg |= PORT_PINCFG_INEN | PORT_PINCFG_PULLEN;
  REG_PORT_OUTSET0 = PORT_PA19;
  delay(1);

  if ((REG_PORT_IN0 & (1 << 19)) == 0) {  //play Mario theme if pins shorted on boot
    play_mario_theme2();
  }

  PORT->Group[1].DIRSET.reg |= PORT_PB01; //set PB01 (LEDSTRIP3) to output
  //PORT->Group[1].DIRSET.reg |= PORT_PB03;
  //digitalWrite(13, LOW);
  led_setup();
  
  //led_show();

  //offset_otos_with_lidar(true, 10, 3, 4, 50);
  
  

}

// the loop function runs over and over again forever
void loop() {  
  unsigned char buf[5];
  int toggle = 0;
  int toggle2 = 1, toggle3 = 1;
  //uint8_t fake_buf[20];

  test_leds();
  //led_show();
  
}


