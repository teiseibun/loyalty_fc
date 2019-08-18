#ifndef __SPI_H
#define __SPI_H

void spi1_init();

uint8_t spi_read_write(SPI_TypeDef *spi_channel, uint8_t data);

#endif
