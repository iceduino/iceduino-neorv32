#include <neorv32_olimex.h>


int main() {

  olimex_led_clr_pin(0);
  olimex_led_clr_pin(1);
  
  while (1) {
      olimex_led_clr_pin(0);   
      olimex_led_clr_pin(1); 
      neorv32_cpu_delay_ms(100);
      olimex_led_set_pin(0);  
      olimex_led_set_pin(1);  
      neorv32_cpu_delay_ms(100);
  }

  return 0;
}


