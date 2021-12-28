#include <neorv32_olimex.h>


int main() {

  olimex_led_clr_pin(0);
  olimex_led_clr_pin(1);

  while (1) {
      if(olimex_button_get(0)) {
         olimex_led_set_pin(0);  
      } else {
         olimex_led_clr_pin(0);
      }
      if(olimex_button_get(1)) {
         olimex_led_set_pin(1);  
      } else {
         olimex_led_clr_pin(1);
      }
  }

  return 0;
}


