#include <neorv32_iceduino.h>


int main() {

  iceduino_led_clr();

  int cnt = 0;

  while (1) {
    iceduino_led_set(cnt & 0xFF);
    //neorv32_cpu_delay_ms(100);
    cnt++;
  }

  return 0;
}


