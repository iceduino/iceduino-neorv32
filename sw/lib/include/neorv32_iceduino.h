#ifndef iceduino_neorv32_h
#define iceduino_neorv32_h
#include "neorv32.h"


//#io devices#
#include "iceduino_led.h"
#include "iceduino_switch.h"
#include "iceduino_button.h"
#include "iceduino_pmod.h"
#include "iceduino_gpio.h"
#include "iceduino_uart.h"
#include "iceduino_spi.h"
#include "iceduino_i2c.h"
#include "iceduino_adc.h"

//#memory mapped io#
//LED base address
#define ICEDUINO_LED_BASE (0xFFFF8000UL) 
//LED output port 8-bit (r/w) */
#define ICEDUINO_LED (*((volatile uint8_t*) (ICEDUINO_GPIO_BASE)))

//SWITCH base address
#define ICEDUINO_SWITCH_BASE (0xFFFF8008UL) 
//SWITCH input port 8-bit (r) */
#define ICEDUINO_SWITCH (*((const volatile uint8_t*) (ICEDUINO_SWITCH_BASE)))

//BUTTON base address
#define ICEDUINO_BUTTON_BASE (0xFFFF8010UL) 
//BUTTON input port 8-bit (r) */
#define ICEDUINO_BUTTON (*((const volatile uint8_t*) (ICEDUINO_BUTTON_BASE)))

//PMOD1 address
#define ICEDUINO_PMOD1_BASE (0xFFFF8018UL)
//PMOD1 input port 8-bit (r) */
#define ICEDUINO_PMOD1_INPUT (*((const volatile uint8_t*) (ICEDUINO_PMOD1_BASE)))
//PMOD1 output port 8-bit (r/w) */
#define ICEDUINO_PMOD1_OUTPUT (*((volatile uint8_t*) (ICEDUINO_PMOD1_BASE + 8)))

//PMOD2 address
#define ICEDUINO_PMOD2_BASE (0xFFFF8028UL)
//PMOD2 input port 8-bit (r) */
#define ICEDUINO_PMOD2_INPUT (*((const volatile uint8_t*) (ICEDUINO_PMOD2_BASE)))
//PMOD2 output port 8-bit (r/w) */
#define ICEDUINO_PMOD2_OUTPUT (*((volatile uint8_t*) (ICEDUINO_PMOD2_BASE + 8)))

//PMOD3 address
#define ICEDUINO_PMOD3_BASE (0xFFFF8038UL)
//PMOD3 input port 8-bit (r) */
#define ICEDUINO_PMOD3_INPUT (*((const volatile uint8_t*) (ICEDUINO_PMOD3_BASE)))
//PMOD3 output port 8-bit (r/w) */
#define ICEDUINO_PMOD3_OUTPUT (*((volatile uint8_t*) (ICEDUINO_PMOD3_BASE + 8)))

//ARDUINO GPIO address
#define ICEDUINO_ARD_GPIO_BASE (0xFFFF8048UL)
//PMOD3 input port 8-bit (r) */
#define ICEDUINO_ARD_GPIO_INPUT (*((const volatile uint8_t*) (ICEDUINO_ARD_GPIO_BASE)))
//PMOD3 output port 8-bit (r/w) */
#define ICEDUINO_ARD_GPIO_OUTPUT (*((volatile uint8_t*) (ICEDUINO_ARD_GPIO_BASE + 8)))

//ARDUINO UART address
#define ICEDUINO_ARD_UART_BASE (0xFFFF8058UL) 
//ARDUINO UART port 8-bit (r/w) */
#define ICEDUINO_ARD_UART (*((volatile uint8_t*) (ICEDUINO_ARD_UART_BASE)))

//ARDUINO SPI address
#define ICEDUINO_ARD_SPI_BASE (0xFFFF8060UL) 
//ARDUINO SPI port 8-bit (r/w) */
#define ICEDUINO_ARD_SPI (*((volatile uint8_t*) (ICEDUINO_ARD_SPI_BASE)))

//ARDUINO I2C address
#define ICEDUINO_ARD_I2C_BASE (0xFFFF8068UL) 
//ARDUINO I2C port 8-bit (r/w) */
#define ICEDUINO_ARD_I2C (*((volatile uint8_t*) (ICEDUINO_ARD_I2C_BASE)))

//ARDUINO ADC address
#define ICEDUINO_ADC_BASE (0xFFFF8070UL) 
//ARDUINO ADC input port 8-bit (r/w) */
#define ICEDUINO_ADC (*((volatile uint8_t*) (ICEDUINO_ADC_BASE)))

#endif // iceduino_neorv32_h
