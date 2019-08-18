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

	led_on(LED1);

	while(1);

	return 0;
}
