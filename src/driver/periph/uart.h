#ifndef __USART_H__
#define __USART_H__

#include <stdint.h>
#include <stdbool.h>

void uart3_init(int baudrate);
bool uart3_tx_busy();
void uart3_puts(uint8_t *data, int size);
uint8_t uart3_getc();
int printf(const char *format, ...);

#endif
