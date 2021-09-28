#include "neorv32_iceduino.h"

uint8_t iceduino_switch_get(int number) {
    uint8_t mask = (uint8_t)(1 << (number & 0x2));
    return ICEDUINO_SWITCH & mask;  
}


