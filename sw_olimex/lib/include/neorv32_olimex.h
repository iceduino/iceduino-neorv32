#ifndef olimex_neorv32_h
#define olimex_neorv32_h
#include "neorv32.h"


//#io devices#
#include "olimex_led.h"
#include "olimex_button.h"
#include "olimex_gpio.h"

//#memory mapped io#
//LED base address
#define OLIMEX_LED_BASE (0xF0000000UL) 
//LED output port 8-bit (r/w) */
#define OLIMEX_LED (*((volatile uint8_t*) (OLIMEX_LED_BASE)))

//BUTTON base address
#define OLIMEX_BUTTON_BASE (0xF0000010UL) 
//BUTTON input port 8-bit (r) */
#define OLIMEX_BUTTON (*((const volatile uint8_t*) (OLIMEX_BUTTON_BASE)))

//GPIO base address
#define OLIMEX_GPIO_BASE (0xF0000048UL)
//GPIO output port 32-bit (r/w) */
#define OLIMEX_GPIO_OUTPUT (*((volatile uint32_t*) (OLIMEX_GPIO_BASE)))
//GPIO input port 32-bit (r) */
#define OLIMEX_GPIO_INPUT (*((const volatile uint32_t*) (OLIMEX_GPIO_BASE + 8)))


#endif // olimex_neorv32_h
