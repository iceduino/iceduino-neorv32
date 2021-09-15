#include "neorv32.h"
#include "iceduino_gpio.h"

//set all pins of output 0-31
void iceduino_gpio_set(int value) {
  ICEDUINO_GPIO_OUTPUT = (uint32_t) value;
}

//clear output
void iceduino_gpio_clr() {  
  ICEDUINO_GPIO_OUTPUT = 0x00000000;
}


