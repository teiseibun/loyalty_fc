#include "bound.h"
#include "controller.h"
#include "motor.h"

void attitude_pd_control(pid_control_t *pid, float ahrs_attitude,
			 float setpoint_attitude, float angular_velocity)
{
	//error = reference (setpoint) - measurement
	pid->error_current = setpoint_attitude - ahrs_attitude;

	pid->error_derivative = angular_velocity;

	pid->output = pid->kp * pid->error_current + pid->kd * pid->error_derivative;

	if(pid->output > pid->output_max) pid->output = pid->output_max;
	if(pid->output < pid->output_min) pid->output = pid->output_min;
}

void yaw_rate_p_control(pid_control_t *pid, float setpoint_yaw_rate, float angular_velocity)
{
	pid->error_current = setpoint_yaw_rate - angular_velocity;

	bound_float(&pid->output, pid->output_max, pid->output_min);
}

void motor_control(volatile float throttle_percentage, uint16_t roll_ctrl_precentage,
		   uint16_t pitch_ctrl_precentage, float yaw_ctrl_precentage)
{
	uint16_t percentage_to_pwm = 0.01 * (MOTOR_PULSE_MAX - MOTOR_PULSE_MIN);

	volatile uint16_t power_basis = throttle_percentage * percentage_to_pwm + MOTOR_PULSE_MIN;
	uint16_t roll_pwm = roll_ctrl_precentage * percentage_to_pwm;
	uint16_t pitch_pwm = pitch_ctrl_precentage * percentage_to_pwm;
	uint16_t yaw_pwm = yaw_ctrl_precentage * percentage_to_pwm;

	uint16_t motor1, motor2, motor3, motor4;

	motor1 = power_basis + roll_pwm + pitch_pwm + yaw_pwm;
	motor2 = power_basis - roll_pwm + pitch_pwm - yaw_pwm;
	motor3 = power_basis - roll_pwm - pitch_pwm + yaw_pwm;
	motor4 = power_basis + roll_pwm - pitch_pwm - yaw_pwm;

	bound_uint16(&motor1, MOTOR_PULSE_MAX, MOTOR_PULSE_MIN);
	bound_uint16(&motor2, MOTOR_PULSE_MAX, MOTOR_PULSE_MIN);
	bound_uint16(&motor3, MOTOR_PULSE_MAX, MOTOR_PULSE_MIN);
	bound_uint16(&motor4, MOTOR_PULSE_MAX, MOTOR_PULSE_MIN);

	set_motor_pwm_pulse(MOTOR1, motor1);
	set_motor_pwm_pulse(MOTOR2, motor2);
	set_motor_pwm_pulse(MOTOR3, motor3);
	set_motor_pwm_pulse(MOTOR4, motor4);
}
