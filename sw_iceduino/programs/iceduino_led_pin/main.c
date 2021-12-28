#include <neorv32_iceduino.h>


int main() {
	
  while (1) {
	iceduino_led_clr();
    iceduino_led_set_pin(0);
    iceduino_led_set_pin(1);
    iceduino_led_set_pin(2);
    iceduino_led_set_pin(3);
    iceduino_led_set_pin(4);
    iceduino_led_set_pin(5);
    iceduino_led_set_pin(6);
    iceduino_led_set_pin(7);    
  }

  return 0;
}
