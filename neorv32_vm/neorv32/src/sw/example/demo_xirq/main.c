// #################################################################################################
// # << NEORV32 - External Interrupt Controller (XIRQ) Demo Program >>                             #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2021, Stephan Nolting. All rights reserved.                                     #
// #                                                                                               #
// # Redistribution and use in source and binary forms, with or without modification, are          #
// # permitted provided that the following conditions are met:                                     #
// #                                                                                               #
// # 1. Redistributions of source code must retain the above copyright notice, this list of        #
// #    conditions and the following disclaimer.                                                   #
// #                                                                                               #
// # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
// #    conditions and the following disclaimer in the documentation and/or other materials        #
// #    provided with the distribution.                                                            #
// #                                                                                               #
// # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
// #    endorse or promote products derived from this software without specific prior written      #
// #    permission.                                                                                #
// #                                                                                               #
// # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
// # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
// # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
// # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
// # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
// # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
// # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
// # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
// # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
// # ********************************************************************************************* #
// # The NEORV32 Processor - https://github.com/stnolting/neorv32              (c) Stephan Nolting #
// #################################################################################################


/**********************************************************************//**
 * @file demo_xirq/main.c
 * @author Stephan Nolting
 * @brief External interrupt controller (XIRQ) demo program (using hardware-assisted prioritization).
 **************************************************************************/

#include <neorv32.h>


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200
/**@}*/

// prototypes
void xirq_handler_ch0(void);
void xirq_handler_ch1(void);
void xirq_handler_ch2(void);
void xirq_handler_ch3(void);


/**********************************************************************//**
 * Main function
 *
 * @note This program requires the WDT and the UART to be synthesized.
 *
 * @return 0 if execution was successful
 **************************************************************************/
int main() {

  // initialize the neorv32 runtime environment
  // this will take care of handling all CPU traps (interrupts and exceptions)
  neorv32_rte_setup();

  // setup UART0 at default baud rate, no parity bits, ho hw flow control
  neorv32_uart0_setup(BAUD_RATE, PARITY_NONE, FLOW_CONTROL_NONE);

  // check if XIRQ unit is implemented at all
  if (neorv32_xirq_available() == 0) {
    neorv32_uart0_printf("XIRQ not synthesized!\n");
    return 1;
  }


  // intro
  neorv32_uart0_printf("External interrupt controller (XIRQ) demo program\n\n");

  int err_cnt = 0;


  // initialize XIRQ controller
  // this will disable all XIRQ channels and will also clear any pending external interrupts
  // (details: this will register the XIRQ's second-level interrupt handler in the NEORV32 RTE)
  err_cnt = neorv32_xirq_setup();

  // check if setup went fine
  if (err_cnt) {
    neorv32_uart0_printf("Error during XIRQ setup!\n");
    return 1;
  }


  // install handler functions for XIRQ channel 0,1,2,3. note that these functions are "normal" functions!
  // (details: these are "third-level" interrupt handler)
  err_cnt = 0;
  err_cnt += neorv32_xirq_install(0, xirq_handler_ch0); // handler function for channel 0
  err_cnt += neorv32_xirq_install(1, xirq_handler_ch1); // handler function for channel 1
  err_cnt += neorv32_xirq_install(2, xirq_handler_ch2); // handler function for channel 2
  err_cnt += neorv32_xirq_install(3, xirq_handler_ch3); // handler function for channel 3

  // check if installation went fine
  if (err_cnt) {
    neorv32_uart0_printf("Error during XIRQ install!\n");
    return 1;
  }


  // allow XIRQ to trigger CPU interrupt
  neorv32_xirq_global_enable();


  // enable global interrupts
  neorv32_cpu_eint();


  // now we are ready to got!
  // the code below assumes the XIRQ inputs are connected to the processor's GPIO output port
  // so we can trigger the IRQs from software; if you have connected the XIRQs to buttons you
  // can remove the code below (note the trigger configuration using the XIRQ generics!)
  {
    // trigger XIRQs 3:0 at once
    // assumes xirq_i <= gpio.output(31:0)

    // due to the prioritization this will execute
    // -> xirq_handler_ch0
    // -> xirq_handler_ch1
    // -> xirq_handler_ch2
    // -> xirq_handler_ch3
    neorv32_gpio_port_set(0xF); // set output pins 3:0 -> trigger XIRQ 3:0
    neorv32_gpio_port_set(0x0);
  }


  // --- wait for interrupts ---
  // All incoming XIRQ interrupt requests are "prioritized" in this example. The XIRQ FIRQ handler
  // reads the ID of the interrupt with the highest priority from the XIRQ controller ("source" register) and calls the according
  // handler function.
  // Non-prioritized handling of interrupts (or custom prioritization) can be implemented by manually reading the
  // XIRQ controller's "pending" register. It is up to the software to define which pending IRQ should be served.
  while(1);


  // just as an example: to disable certain XIRQ interrupt channels, we can
  // un-install the according handler. this will also clear a pending interrupt for that channel
  neorv32_xirq_uninstall(0); // disable XIRQ channel 0 and remove associated handler
  neorv32_xirq_uninstall(1); // disable XIRQ channel 1 and remove associated handler
  neorv32_xirq_uninstall(2); // disable XIRQ channel 2 and remove associated handler
  neorv32_xirq_uninstall(3); // disable XIRQ channel 3 and remove associated handler


  return 0;
}


/**********************************************************************//**
 * XIRQ handler channel 0.
 *
 * @warning This function has to be of type "void xyz(void)" and must not use any interrupt attributes!
 **************************************************************************/
void xirq_handler_ch0(void) {

  neorv32_uart0_printf("XIRQ interrupt from channel %i\n", 0);
}

/**********************************************************************//**
 * XIRQ handler channel 1.
 *
 * @warning This function has to be of type "void xyz(void)" and must not use any interrupt attributes!
 **************************************************************************/
void xirq_handler_ch1(void) {

  neorv32_uart0_printf("XIRQ interrupt from channel %i\n", 1);
}

/**********************************************************************//**
 * XIRQ handler channel 2.
 *
 * @warning This function has to be of type "void xyz(void)" and must not use any interrupt attributes!
 **************************************************************************/
void xirq_handler_ch2(void) {

  neorv32_uart0_printf("XIRQ interrupt from channel %i\n", 2);
}

/**********************************************************************//**
 * XIRQ handler channel 3.
 *
 * @warning This function has to be of type "void xyz(void)" and must not use any interrupt attributes!
 **************************************************************************/
void xirq_handler_ch3(void) {

  neorv32_uart0_printf("XIRQ interrupt from channel %i\n", 3);
}
