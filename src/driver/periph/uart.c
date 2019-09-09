#include <unistd.h>
#include <stdarg.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "stm32f4xx_conf.h"

void uart3_init(int baudrate)
{
	/* RCC initialization */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

	/* GPIO initialization */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

	GPIO_InitTypeDef GPIO_InitStruct = {
		.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11,
		.GPIO_Mode = GPIO_Mode_AF,
		.GPIO_Speed = GPIO_Speed_50MHz,
		.GPIO_OType = GPIO_OType_PP,
		.GPIO_PuPd = GPIO_PuPd_UP
	};
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USART initialization */
	USART_InitTypeDef USART_InitStruct = {
		.USART_BaudRate = baudrate,
		.USART_Mode = USART_Mode_Rx | USART_Mode_Tx,
		.USART_WordLength = USART_WordLength_8b,
		.USART_StopBits = USART_StopBits_1,
		.USART_Parity = USART_Parity_No
	};
	USART_Init(USART3, &USART_InitStruct);

	USART_Cmd(USART3, ENABLE);

	USART_ClearFlag(USART3, USART_FLAG_TC);

	/* DMA initialization */
	DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4);
}

bool uart3_tx_busy()
{
	if(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
		return true;
	else
		return false; 
}

void uart3_puts(uint8_t *data, int size)
{
	DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4);

	/* Setup the DMA */
	DMA_InitTypeDef DMA_InitStructure = {
		.DMA_BufferSize = (uint32_t)size,
		.DMA_FIFOMode = DMA_FIFOMode_Disable,
		.DMA_FIFOThreshold = DMA_FIFOThreshold_Full,
		.DMA_MemoryBurst = DMA_MemoryBurst_Single,
		.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte,
		.DMA_MemoryInc = DMA_MemoryInc_Enable,
		.DMA_Mode = DMA_Mode_Normal,
		.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR),
		.DMA_PeripheralBurst = DMA_PeripheralBurst_Single,
		.DMA_PeripheralInc = DMA_PeripheralInc_Disable,
		.DMA_Priority = DMA_Priority_Medium,
		.DMA_Channel = DMA_Channel_7,
		.DMA_DIR = DMA_DIR_MemoryToPeripheral,
		.DMA_Memory0BaseAddr = (uint32_t)data
        };
	DMA_Init(DMA1_Stream4, &DMA_InitStructure);

	/* Enable DMA to sent the data */
	DMA_Cmd(DMA1_Stream4, ENABLE);
	USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
}

uint8_t uart3_getc()
{
	while(USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == SET);
	return USART_ReceiveData(USART3);
}

int printf(const char *format, ...)
{
	while(uart3_tx_busy() == true);

	char str[128];
	va_list para;
	va_start(para, format);
	int curr_pos = 0;
	char ch[] = {'0', '\0'};
	char integer[11];
	str[0] = '\0';

	while (format[curr_pos] != '\0') {
		if (format[curr_pos] != '%') {
			ch[0] = format[curr_pos];
			strcat(str, ch);

		} else {
			switch (format[++curr_pos]) {
			case 's':
				strcat(str, va_arg(para, char *));
				break;

			case 'c':
				ch[0] = (char)va_arg(para, int);
				strcat(str, ch);
				break;

			case 'i':
			case 'f':
				strcat(str, ftoa(va_arg(para, double)));
				break;

			case 'd':
				strcat(str, itoa(va_arg(para, int), integer));
				break;

			case 'u':
				strcat(str, itoa(va_arg(para, unsigned), integer));
				break;

			default:
				break;
			}
		}

		curr_pos++;
	}

	va_end(para);
	uart3_puts((uint8_t *)str, strlen(str));
	return 1;
}
