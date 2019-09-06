#include "stm32f4xx_conf.h"
#include "timer.h"

void timer1_init()
{
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

        /* 168Mhz / (8400 * 200) = 100hz */
        TIM_TimeBaseInitTypeDef TimeBaseInitStruct = {
                .TIM_Period = 8400 - 1,
                .TIM_Prescaler = 200 - 1,
                .TIM_CounterMode = TIM_CounterMode_Up
        };
        TIM_TimeBaseInit(TIM1, &TimeBaseInitStruct);

        NVIC_InitTypeDef NVIC_InitStruct = {
                .NVIC_IRQChannel = TIM1_UP_TIM10_IRQn,
                .NVIC_IRQChannelPreemptionPriority = 5,
                .NVIC_IRQChannelCmd = ENABLE
        };
        NVIC_Init(&NVIC_InitStruct);

        TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
        TIM_Cmd(TIM1, ENABLE);
}
