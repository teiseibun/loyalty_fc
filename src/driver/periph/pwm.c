#include "stm32f4xx.h"

void pwm_timer4_init()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4); //PWM pin 1
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4); //PWM pin 2

	GPIO_InitTypeDef GPIO_InitStruct = {
		.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7,
		.GPIO_Mode = GPIO_Mode_AF,
		.GPIO_Speed = GPIO_Speed_100MHz,
		.GPIO_OType = GPIO_OType_PP,
		.GPIO_PuPd = GPIO_PuPd_DOWN
	};

	GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* Timer period = 0.002, frequency = 500hz */
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct = {
		.TIM_Period = 2500 - 1,
		.TIM_Prescaler = 84 - 1,
		.TIM_ClockDivision = TIM_CKD_DIV1,
		.TIM_CounterMode = TIM_CounterMode_Up
	};

	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);

	/* PWM mode 1 */
	TIM_OCInitTypeDef TIM_OCInitStruct = {
		.TIM_OCMode = TIM_OCMode_PWM1,
		.TIM_OutputState = TIM_OutputState_Enable,
		.TIM_Pulse = 0,
	};

	TIM_OC1Init(TIM4, &TIM_OCInitStruct);
	TIM_OC2Init(TIM4, &TIM_OCInitStruct);

	TIM_Cmd(TIM4, ENABLE);
}

void pwm_timer5_init()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5); //PWM pin 3
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5); //PWM pin 4

	GPIO_InitTypeDef GPIO_InitStruct = {
		.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1,
		.GPIO_Mode = GPIO_Mode_AF,
		.GPIO_Speed = GPIO_Speed_100MHz,
		.GPIO_OType = GPIO_OType_PP,
		.GPIO_PuPd = GPIO_PuPd_DOWN
	};

	GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* Timer period = 0.002, frequency = 500hz */
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct = {
		.TIM_Period = 2500 - 1,
		.TIM_Prescaler = 84 - 1,
		.TIM_ClockDivision = TIM_CKD_DIV1,
		.TIM_CounterMode = TIM_CounterMode_Up
	};

	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStruct);

	/* PWM mode 1 */
	TIM_OCInitTypeDef TIM_OCInitStruct = {
		.TIM_OCMode = TIM_OCMode_PWM1,
		.TIM_OutputState = TIM_OutputState_Enable,
		.TIM_Pulse = 0,
	};

	TIM_OC1Init(TIM5, &TIM_OCInitStruct);
	TIM_OC2Init(TIM5, &TIM_OCInitStruct);

	TIM_Cmd(TIM5, ENABLE);
}
