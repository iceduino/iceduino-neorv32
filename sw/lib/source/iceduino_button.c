#include "neorv32_iceduino.h"

uint8_t iceduino_button_get(int number) {
    uint8_t mask = (uint8_t)(1 << (number & 0x4));
    return ICEDUINO_BUTTON & mask;  
}


