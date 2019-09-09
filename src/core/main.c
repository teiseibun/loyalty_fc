#include <stdbool.h>

#include "uart.h"
#include "i2c.h"
#include "spi.h"
#include "led.h"
#include "timer.h"
#include "pwm.h"
#include "pwm_capture.h"
#include "rc_receiver.h"
#include "radio_control.h"
#include "mpu9250.h"
#include "motor.h"
#include "delay.h"
#include "link.h"
#include "ahrs.h"

volatile int cnt = 100;

radio_control_t rc;

void SysTick_Handler()
{
	led_on(LED1);

	radio_control_update(&rc);
	ahrs_loop();

	led_off(LED1);
}

void TIM1_UP_TIM10_IRQHandler()
{
	if(TIM_GetITStatus(TIM1, TIM_IT_Update) == SET) {
		//toggle in 1Hz
		if(cnt == 0) {
			led_toggle(LED2);
			cnt = 50;
		} else {
			cnt--;
		}

		telemetry_loop();

		//debug_print_radio_control_expect_value(&rc);
		//debug_print_radio_control_value();

		TIM_ClearITPendingBit(TIM1, TIM_FLAG_Update);
	}
}

int main()
{
        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	led_init();
	spi1_init();
	uart3_init(115200);
	delay_ms(5);
	pwm_timer4_init();
	pwm_timer5_init();
	pwm_capture_timer2_init();
	pwm_capture_timer3_init();
	pwm_capture_timer8_init();
	motor_init();

	while(mpu9250_init());

	ahrs_init();

	SysTick_Config(SystemCoreClock / 500); //500Hz flight controller main loop
	timer1_init(); //100Hz telemetry loop

	while(1);

	return 0;


}
