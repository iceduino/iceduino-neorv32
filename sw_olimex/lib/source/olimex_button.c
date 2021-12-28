#include "neorv32_olimex.h"

uint8_t olimex_button_get(int number) {
    uint8_t mask = (uint8_t)(1 << (number & 0xf));
    return OLIMEX_BUTTON & mask;  
}


