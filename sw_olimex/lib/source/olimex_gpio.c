#include "neorv32_olimex.h"

void olimex_gpio_set_pin(int pin) {
  uint32_t mask = (uint32_t)(1 << (pin & 0x7));
  OLIMEX_GPIO_OUTPUT = OLIMEX_GPIO_OUTPUT | mask;
}

uint8_t olimex_gpio_get_pin(int pin) {
  uint32_t mask = (uint32_t)(1 << (pin & 0x7));
  return OLIMEX_GPIO_INPUT & mask; 
}

void olimex_gpio_clr_pin(int pin){
  uint32_t mask = (uint32_t)(1 << (pin & 0x7));
  OLIMEX_GPIO_OUTPUT = OLIMEX_GPIO_OUTPUT & ~mask;
}


