#include <stdbool.h>

#include "uart.h"
#include "i2c.h"
#include "spi.h"
#include "led.h"
#include "timer.h"
#include "mpu6050.h"
#include "delay.h"
#include "link.h"

volatile int cnt = 100;

void SysTick_Handler()
{
	led_toggle(LED1);
}

void TIM2_IRQHandler()
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) {
		//toggle in 1Hz
		if(cnt == 0) {
			led_toggle(LED2);
			cnt = 50;

			telemetry_loop();
		} else {
			cnt--;
		}

		TIM_ClearITPendingBit(TIM2, TIM_FLAG_Update);
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
	timer2_init(); //100Hz telemetry loop

	while(1);

	return 0;


}
