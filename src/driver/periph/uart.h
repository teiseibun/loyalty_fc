#ifndef __USART_H
#define __USART_H

#include <stdint.h>

void uart3_init(int baudrate);
void uart3_putc(uint8_t data);
void uart3_puts(uint8_t *str);
uint8_t uart3_getc();
int printf(const char *format, ...);

#endif
