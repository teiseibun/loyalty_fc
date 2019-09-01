#include "stm32f4xx_conf.h"
#include "delay.h"
#include "mpu9250.h"

uint8_t mpu9250_read_byte(uint8_t address)
{
	uint8_t read;

	mpu9250_chip_select();

	spi_read_write(SPI1, address | 0x80);
	read = spi_read_write(SPI1, 0xff);

	mpu9250_chip_deselect();

	return read;
}

void mpu9250_write_byte(uint8_t address, uint8_t data)
{
	mpu9250_chip_select();

	spi_read_write(SPI1, address);
	spi_read_write(SPI1, data);

	mpu9250_chip_deselect();
}

uint8_t mpu9250_read_who_am_i()
{
	uint8_t id;
	id = mpu9250_read_byte(MPU9250_WHO_AM_I);

	return id;
}

void mpu9250_reset()
{
        mpu9250_write_byte(MPU9250_PWR_MGMT_1, 0x80);
	delay_ms(500);
}

int mpu9250_init()
{
	//read mpu9250 device id
	if(mpu9250_read_who_am_i() != 0x71) return 1;

	//reset mpu9250
	mpu9250_reset();

	delay_ms(5);

        return 0;
}
