#include "neorv32_olimex.h"

//set led pin 
void olimex_led_set_pin(int pin) {
  if(pin == 0) OLIMEX_LED = OLIMEX_LED | 0x1;
  if(pin == 1) OLIMEX_LED = OLIMEX_LED | 0x2;
}

//clear led pin
void olimex_led_clr_pin(int pin) {
  if(pin == 0) OLIMEX_LED = OLIMEX_LED & 0xFE; //11111110
  if(pin == 1) OLIMEX_LED = OLIMEX_LED & 0xFD; //11111101
}


