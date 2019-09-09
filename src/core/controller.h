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

#endif
