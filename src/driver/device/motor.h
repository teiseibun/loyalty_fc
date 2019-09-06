#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx.h"

#define MOTOR_PULSE_MAX 2375
#define MOTOR_PULSE_MIN 800

#define MOTOR1 &TIM4->CCR1
#define MOTOR2 &TIM4->CCR2
#define MOTOR3 &TIM5->CCR1
#define MOTOR4 &TIM5->CCR2

void set_motor_pwm_pulse(volatile uint32_t *motor, uint16_t pulse);
void motor_init();

void esc_calibrate();

#endif
