#include <stdint.h>

#include "motor.h"

#include "delay.h"

void set_motor_pwm_pulse(volatile uint32_t *motor, uint16_t pulse)
{
	if(pulse < MOTOR_PULSE_MIN)
		*motor = MOTOR_PULSE_MIN;
	else if(pulse > MOTOR_PULSE_MAX)
		*motor = MOTOR_PULSE_MAX;
	else *motor = pulse;
}

void motor_init()
{
	set_motor_pwm_pulse(MOTOR1, MOTOR_PULSE_MIN);
	set_motor_pwm_pulse(MOTOR2, MOTOR_PULSE_MIN);
	set_motor_pwm_pulse(MOTOR3, MOTOR_PULSE_MIN);
	set_motor_pwm_pulse(MOTOR4, MOTOR_PULSE_MIN);

	delay_ms(8000);
}

void esc_calibrate()
{
	set_motor_pwm_pulse(MOTOR1, MOTOR_PULSE_MAX);
	set_motor_pwm_pulse(MOTOR2, MOTOR_PULSE_MAX);
	set_motor_pwm_pulse(MOTOR3, MOTOR_PULSE_MAX);
	set_motor_pwm_pulse(MOTOR4, MOTOR_PULSE_MAX);

	delay_ms(2000);

	set_motor_pwm_pulse(MOTOR1, MOTOR_PULSE_MIN);
	set_motor_pwm_pulse(MOTOR2, MOTOR_PULSE_MIN);
	set_motor_pwm_pulse(MOTOR3, MOTOR_PULSE_MIN);
	set_motor_pwm_pulse(MOTOR4, MOTOR_PULSE_MIN);

	while(1);
}
