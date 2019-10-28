#ifndef __STM32_P103_H
#define __STM32_P103_H

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "misc.h"

/* Initialize the LED (the board only has one). */
void init_led(void);

/* Initialize the button (the board only has one). */
void init_button(void);

/* Configures the RS232 serial port using the following settings:
 *   9600 Baud
 *   8 bits + 1 stop bit
 *   No parity bit
 *   No hardware flow control
 * Note that the USART2 is not enabled in this routine.  It is left disabled in
 * case any additional configuration is needed.
 */
void init_rs232(void);

void enable_rs232_interrupts(void);

void enable_rs232(void);

#endif /* __STM32_P103_H */
