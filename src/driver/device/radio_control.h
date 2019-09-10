#ifndef __RADIO_CONTROL_H
#define __RADIO_CONTROL_H

#define RC_ROLL_ANGLE_MAX 35.0
#define RC_ROLL_ANGLE_MIN -35.0

#define RC_PITCH_ANGLE_MAX 35.0
#define RC_PITCH_ANGLE_MIN -35.0

#define RC_YAW_RATE_MAX 35.0
#define RC_YAW_RATE_MIN -35.0

#define RC_THROTTLE_SCALE_MAX 100.0
#define RC_THROTTLE_SCALE_MIN 0.0

typedef enum {ENGINE_OFF, ENGINE_ON} RC_Safety; 

typedef struct {
	RC_Safety safety_status;
	float roll_angle;
	float pitch_angle;
	float yaw_rate;
	float throttle_scale;
} radio_control_t;

void radio_control_update(radio_control_t *radio_control_data);
int radio_control_safety_check(radio_control_t *radio_control_data);

void debug_print_radio_control_expect_value(radio_control_t *radio_control_data);

#endif
