#include "neorv32_iceduino.h"

void iceduino_gpio_set_pin(int pin) {
  uint8_t mask = (uint8_t)(1 << (pin & 0x7));
  ICEDUINO_ARD_GPIO_OUTPUT = ICEDUINO_ARD_GPIO_OUTPUT | mask;
}

uint8_t iceduino_gpio_get_pin(int pin) {
  uint8_t mask = (uint8_t)(1 << (number & 0x7));
  return ICEDUINO_ARD_GPIO_INPUT & mask; 
}

void iceduino_gpio_clr_pin(int pin){
  uint8_t mask = (uint8_t)(1 << (pin & 0x7));
  ICEDUINO_ARD_GPIO_OUTPUT = ICEDUINO_ARD_GPIO_OUTPUT & ~mask;
}


