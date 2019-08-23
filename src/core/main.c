#include <stdbool.h>

#include "uart.h"
#include "i2c.h"
#include "spi.h"
#include "led.h"
#include "timer.h"
#include "mpu6050.h"
#include "delay.h"
#include "link.h"
#include "ahrs.h"

volatile int cnt = 100;

void SysTick_Handler()
{
	led_toggle(LED1);

	ahrs_ekf_loop();


#if 0   //some unit tests...
	if(uart3_tx_busy() == true) return;

	attitude_t euler;
	quat_t q;

	euler.roll = deg_to_rad(30.0);
	euler.pitch =deg_to_rad(40.0);
	euler.yaw = deg_to_rad(50);

	euler_to_quat(&euler, &q);
	quat_normalize(&q);
	quat_to_euler(&q, &euler);

	printf("%f, %f, %f\n\r", rad_to_deg(euler.roll), rad_to_deg(euler.pitch), rad_to_deg(euler.yaw));
#endif
}

void TIM2_IRQHandler()
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) {
		//toggle in 1Hz
		if(cnt == 0) {
			led_toggle(LED2);
			cnt = 50;
		} else {
			cnt--;
		}

		telemetry_loop();

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

	ahrs_ekf_init();

	SysTick_Config(SystemCoreClock / 500); //500Hz flight controller main loop
	timer2_init(); //100Hz telemetry loop

	while(1);

	return 0;


}
