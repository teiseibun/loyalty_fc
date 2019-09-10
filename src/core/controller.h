#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

typedef struct {
	float kp;
	float ki;
	float kd;
	float error_current;
	float error_last;
	float error_integral;
	float error_derivative;
	float output;
	float output_max;
	float output_min;
} pid_control_t;

void attitude_pd_control(pid_control_t *pid, float ahrs_attitude,
			 float setpoint_attitude, float angular_velocity);
void yaw_rate_p_control(pid_control_t *pid, float setpoint_yaw_rate,
			float angular_velocity);

void motor_control(volatile float throttle_percentage, uint16_t roll_ctrl_precentage,
		   uint16_t pitch_ctrl_precentage, float yaw_ctrl_precentage);

#endif
