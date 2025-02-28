//In the FastLED library, updated src/platforms/arm/d51/fastpin_arm_d51.h to map the digital pins to the ledstrip pins
//using this line in the Feather M4 Express config, replace:
//
//  _FL_DEFPIN( 0, 17, 1); _FL_DEFPIN( 1, 16, 1);
//
//  with:
//
//  _FL_DEFPIN( 0, 3, 1); _FL_DEFPIN( 1, 2, 1);

#include <FastLED.h>
#define NUM_LEDS 10
#define BRIGHTNESS 200
CRGB leds[NUM_LEDS];


uint32_t green_array[NUM_LEDS];
uint32_t red_array[NUM_LEDS];
uint32_t blue_array[NUM_LEDS];
uint32_t sparkles[NUM_LEDS];

void initialize_color_array_states(){
  for (int i = 0; i < NUM_LEDS; i++){
    if (i % 3 == 1){
      red_array[i] = 0xff0000;
      green_array[i] = 0x00ff00;
      blue_array[i] = 0x0000ff;
    }else if(i % 3 == 2){
      red_array[i] = 0xee1100;
      green_array[i] == 0x11ee00;
      blue_array[i] = 0x0011ee;
    } else {
      red_array[i] = 0xee0011;
      green_array[i] = 0x00ee11;
      blue_array[i] = 0x1100ee;
    }
  }
  

}

void led_setup() {
  //initialize_color_array_states();
  FastLED.addLeds<WS2812B, 4, GRB>(leds, NUM_LEDS);  //add 10 LEDs to pin 0 (LEDSTRIP 1)
  FastLED.setBrightness(BRIGHTNESS);
}

void test_leds() {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Blue;
    FastLED.show();
    delay(30);
  }
  /*
  digitalWrite(12, HIGH);
  delay(1000);
  digitalWrite(12, LOW);
  delay(1000);
  */
}

void led_show() {
  int delay_time = 30;
  
  for (int i=0; i<10; i++) {
    leds[i] = CRGB::Red; FastLED.show(); delay(delay_time);
    leds[i] = CRGB::Black; FastLED.show(); delay(delay_time);
  }
  
  for (int i=8; i>0; i--) {
    leds[i] = CRGB::Red; FastLED.show(); delay(delay_time);
    leds[i] = CRGB::Black; FastLED.show(); delay(delay_time);
  }

  for (int i=0; i<10; i++) {
    leds[i] = CRGB::Green; FastLED.show(); delay(delay_time);
    leds[i] = CRGB::Black; FastLED.show(); delay(delay_time);
  }
  
  for (int i=8; i>0; i--) {
    leds[i] = CRGB::Green; FastLED.show(); delay(delay_time);
    leds[i] = CRGB::Black; FastLED.show(); delay(delay_time);
  }

  for (int i=0; i<10; i++) {
    leds[i] = CRGB::Blue; FastLED.show(); delay(delay_time);
    leds[i] = CRGB::Black; FastLED.show(); delay(delay_time);
  }
  
  for (int i=8; i>0; i--) {
    leds[i] = CRGB::Blue; FastLED.show(); delay(delay_time);
    leds[i] = CRGB::Black; FastLED.show(); delay(delay_time);
  }

  for (int i=0; i<10; i++) {
    leds[i] = CRGB::Yellow; FastLED.show(); delay(delay_time);
    leds[i] = CRGB::Black; FastLED.show(); delay(delay_time);
  }
  
  for (int i=8; i>0; i--) {
    leds[i] = CRGB::Yellow; FastLED.show(); delay(delay_time);
    leds[i] = CRGB::Black; FastLED.show(); delay(delay_time);
  }

     for (int i=0; i<10; i++) {
    leds[i] = CRGB::White; FastLED.show(); delay(delay_time);
    leds[i] = CRGB::Black; FastLED.show(); delay(delay_time);
  }
  
  for (int i=8; i>0; i--) {
    leds[i] = CRGB::White; FastLED.show(); delay(delay_time);
    leds[i] = CRGB::Black; FastLED.show(); delay(delay_time);
  }
}

void led_blip() {
  int delay_time = 40;
  
  for (int i=0; i<1; i++) {
    leds[i] = CRGB::Red; FastLED.show(); delay(delay_time);
    leds[i] = CRGB::Black; FastLED.show(); delay(delay_time);
  }
}

void led_on(int num) {
  if (num == 1) {
    digitalWrite(12, HIGH);    // turn LED1 on 
  } else if (num == 2) {
    digitalWrite(13, HIGH);    // turn LED2 on 
  }
}

void led_off(int num) {
  if (num == 1) {
    digitalWrite(12, LOW);    // turn LED1 off
  } else if (num == 2) {
    digitalWrite(13, LOW);    // turn LED2 off
  }
}

void manage_led_states(bool x){
  if (x == false) {
    led_on(2);
    x = true;
  } else {
    led_off(2);
    x = false;
  }
}

void init_led_timer() {
  // Enable the clock for TC3
  MCLK->APBBMASK.reg |= MCLK_APBBMASK_TC3;  // Enable TC3 peripheral bus
  GCLK->PCHCTRL[TC3_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN;  // Use GCLK0

  // Wait for synchronization
  while (GCLK->PCHCTRL[TC3_GCLK_ID].reg & GCLK_PCHCTRL_CHEN) {}

  // Reset TC3
  TC3->COUNT32.CTRLA.reg = TC_CTRLA_SWRST;
  while (TC3->COUNT32.CTRLA.bit.SWRST) {}  // Wait for reset to complete

  // Set the counter mode to 16-bit (can also be 8-bit or 32-bit)
  TC3->COUNT32.CTRLA.reg |= TC_CTRLA_MODE_COUNT32;  // 16-bit counter
  TC3->COUNT32.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;  // Set prescaler to 256

  // Set the compare match value
  TC3->COUNT32.CC[0].reg = 10 * (120000000 / 1024) / 1000;  // Set to 1ms intervals at 48MHz clock
  while (TC3->COUNT32.SYNCBUSY.bit.CC0) {}  // Wait for synchronization

  // Enable TC3 interrupt for compare match (OVF interrupt)
  TC3->COUNT32.INTENSET.reg = TC_INTENSET_MC0;  // Enable interrupt on compare match 0
  NVIC_EnableIRQ(TC3_IRQn);  // Enable TC3 interrupt in NVIC

  // Enable the TC3 module
  TC3->COUNT32.CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC3->COUNT32.SYNCBUSY.bit.ENABLE) {}  // Wait for synchronization
}

// Interrupt Service Routine (ISR) for TC3
void TC3_Handler(void) {
  static boolean x = false;
  
  if (TC3->COUNT32.INTFLAG.bit.MC0) {
    // Clear the interrupt flag
    TC3->COUNT32.INTFLAG.reg = TC_INTFLAG_MC0;

    // Do something - for example, toggle an LED or set a flag
    manage_led_states(x);

    float counts_per_ms = (120000000/1024.0) / 1000.0;
    TC3->COUNT32.CC[0].reg += 500 * counts_per_ms;  // Set to 1ms intervals at 120MHz clock
    //while (TC3->COUNT32.SYNCBUSY.bit.CC0) {}
  }
}

// ChatGPT methods

#include "sam.h" // Ensure this points to the right MCU header

#define LED_PIN PORT_PB01 // Adjust this to match your pin mapping

void delay_us(uint32_t us) {
  for (volatile uint32_t i = 0; i < (us * (SystemCoreClock / 3000000)); i++);
}

void send_bit(int bit) {
  if (bit) { // '1' bit
    PORT->Group[1].OUTSET.reg = LED_PIN; // Set high
    delay_us(0.8); // Hold high for 800 ns
    PORT->Group[1].OUTCLR.reg = LED_PIN; // Set low
    delay_us(0.45); // Hold low for 450 ns
  } else { // '0' bit
    PORT->Group[1].OUTSET.reg = LED_PIN; // Set high
    delay_us(0.4); // Hold high for 400 ns
    PORT->Group[1].OUTCLR.reg = LED_PIN; // Set low
    delay_us(0.85); // Hold low for 850 ns
  }
}

void send_byte(uint8_t byte) {
  for (int i = 7; i >= 0; i--) {
    send_bit((byte >> i) & 1); // Send each bit, MSB first
  }
}

void send_color(uint8_t red, uint8_t green, uint8_t blue) {
  send_byte(green); // WS2812 expects GRB format
  send_byte(red);
  send_byte(blue);
  delay_us(50); // Latch data (minimum 50 µs reset time)
}

// Send color to only the first LED out of the strip of 144
void send_color_to_one_led(uint8_t red, uint8_t green, uint8_t blue) {
  // Send color data for the first LED
  send_color(red, green, blue);

  // Send 'off' (0, 0, 0) for the rest of the LEDs
  for (int i = 1; i < 144; i++) {
    send_color(0, 0, 0);
  }

  // Ensure data is latched
  delay_us(50); // Minimum 50 µs reset time
}

void led_setup_ChatGPT() {
  PORT->Group[1].DIRSET.reg |= LED_PIN; // Set PB01 as output
}

int main_led_ChatGPT() {
  led_setup();
  
  while (1) {
    send_color_to_one_led(255, 0, 0); // Red for the first LED only
    delay_us(500000);                 // 500ms delay
    
    send_color_to_one_led(0, 255, 0); // Green for the first LED only
    delay_us(500000);
    
    send_color_to_one_led(0, 0, 255); // Blue for the first LED only
    delay_us(500000);
  }
}
