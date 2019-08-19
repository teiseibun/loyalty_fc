#include "uart.h"
#include "i2c.h"
#include "spi.h"
#include "led.h"
#include "timer.h"

#include "mpu6050.h"

#include "delay.h"

volatile int cnt = 500;

void SysTick_Handler()
{
	led_toggle(LED1);

	if(cnt != 0) {
		cnt--;
	} else {
		cnt = 500;
		printf("hello\n\r");
	}
}

int main()
{
	led_init();
	i2c1_init();
	uart3_init(115200);

	delay_ms(5);

	while(mpu6050_init());

	SysTick_Config(SystemCoreClock / 500); //500Hz flight controller main loop

	while(1);

	return 0;


}
