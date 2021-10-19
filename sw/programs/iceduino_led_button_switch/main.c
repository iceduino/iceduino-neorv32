#include <neorv32_iceduino.h>


int main() {

  iceduino_led_clr();
  
  while (1) {
    // Switch 0 Led 0  
    if(iceduino_switch_get(0)){
        iceduino_led_set_pin(0);
    } else {
        iceduino_led_clr_pin(0);
    }
    // Switch 1 Led 1  
    if(iceduino_switch_get(1)){
        iceduino_led_set_pin(1);
    } else {
        iceduino_led_clr_pin(1);
    }
    // Button 0 Led 2  
    if(iceduino_button_get(0)){
        iceduino_led_set_pin(2);
    } else {
        iceduino_led_clr_pin(2);
    }
    // Button 1 Led 3  
    if(iceduino_button_get(1)){
        iceduino_led_set_pin(3);
    } else {
        iceduino_led_clr_pin(3);
    }
    // Button 2 Led 4  
    if(iceduino_button_get(2)){
        iceduino_led_set_pin(4);
    } else {
        iceduino_led_clr_pin(4);
    }
    // Button 3 Led 5  
    if(iceduino_button_get(3)){
        iceduino_led_set_pin(5);
    } else {
        iceduino_led_clr_pin(5);
    }
    // Button 4 Led 6  
    if(iceduino_button_get(4)){
        iceduino_led_set_pin(6);
    } else {
        iceduino_led_clr_pin(6);
    }
    iceduino_led_set_pin(7);
  }

  return 0;
}


