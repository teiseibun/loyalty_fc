#include "uart.h"
#include "i2c.h"
#include "spi.h"
#include "led.h"
#include "timer.h"

#include "mpu6050.h"

#include "delay.h"


int main()
{
	led_init();
	i2c1_init();

	delay_ms(5);

	while(mpu6050_init());

	led_on(LED1);

	while(1);

	return 0;
}
