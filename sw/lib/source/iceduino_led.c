#include "neorv32_iceduino.h"

//set led port (0-7)
void iceduino_led_set(int value) {
  ICEDUINO_LED = (uint8_t) value;
}

//clear output
void iceduino_led_clr() {  
  ICEDUINO_LED = 0x00000000;
}

//set led pin 
void iceduino_led_set_pin(int pin) {
  uint8_t mask = (uint8_t)(1 << (pin & 0x7));
  ICEDUINO_LED = ICEDUINO_LED | mask;
}

//clear led pin
void iceduino_led_clr_pin(int pin) {
  uint8_t mask = (uint8_t)(1 << (pin & 0x7));
  ICEDUINO_LED = ICEDUINO_LED & ~mask;
}


