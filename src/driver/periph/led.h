#ifndef __LED_H
#define __LED_H

#include "stm32f4xx.h"

#define led_toggle(led)  GPIO_ToggleBits(led)
/* 
 * <For QCopter Flight Control Board>
 * Because there are three push-up register in the circuit for the LED
 * The GPIO high/low operation to the LED will be opposite
 */
#define led_off(led)  GPIO_SetBits(led)
#define led_on(led)  GPIO_ResetBits(led)

#define LED1 GPIOC, GPIO_Pin_15
#define LED2 GPIOC, GPIO_Pin_14
#define LED3 GPIOC, GPIO_Pin_13

/* Debug port, for work frequency measurement  */
#define debug_port_off(port) GPIO_ResetBits(port)
#define debug_port_on(port) GPIO_SetBits(port)

#define DEBUG_PORT GPIOB, GPIO_Pin_6

void led_init();
void debug_port_init();

#endif
