#include "uart.h"
#include "rc_receiver.h"
#include "radio_control.h"
#include "delay.h"
#include "bound.h"

void radio_control_update(radio_control_t *radio_control_data)
{
	/* The RC value have to bigger then 50 persent of the scale */
	if((get_radio_control_value(SAFETY_CHANNEL) - RC_ROLL_MIN) > ((RC_ROLL_MAX - RC_ROLL_MIN) / 2))
		radio_control_data->safety_status = ENGINE_ON;
	else
		radio_control_data->safety_status = ENGINE_OFF;

	/* RC expect value = rc_scale * scale_size - offset */
	//RC expect roll angle range: -45 ~ 45 (degree), offset = -45
	radio_control_data->roll_angle =
		(float)(get_radio_control_value(ROLL_CHANNEL) - RC_ROLL_MIN) / (RC_ROLL_MAX - RC_ROLL_MIN) *
			(RC_ROLL_ANGLE_MAX - RC_ROLL_ANGLE_MIN) -
			(0 - RC_ROLL_ANGLE_MIN);
	bound_float(&radio_control_data->roll_angle, RC_ROLL_ANGLE_MAX, RC_ROLL_ANGLE_MIN);

	//RC expect pitch angle range: -45 ~ 45 (degree), offset = -45
	radio_control_data->pitch_angle =
		(float)(get_radio_control_value(PITCH_CHANNEL) - RC_PITCH_MIN) / (RC_PITCH_MAX - RC_PITCH_MIN) *
			(RC_PITCH_ANGLE_MAX - RC_PITCH_ANGLE_MIN) -
			(0 - RC_PITCH_ANGLE_MIN);
	bound_float(&radio_control_data->pitch_angle, RC_PITCH_ANGLE_MAX, RC_PITCH_ANGLE_MIN);

	//RC expect roll angle range: -45 ~ 45 (degree), offset = -45
	radio_control_data->yaw_rate =
		(float)(get_radio_control_value(YAW_CHANNEL) - RC_YAW_MIN) / (RC_YAW_MAX - RC_YAW_MIN) *
			(RC_YAW_RATE_MAX - RC_YAW_RATE_MIN) -
			(0 - RC_YAW_RATE_MIN);
	bound_float(&radio_control_data->yaw_rate, RC_YAW_RATE_MAX, RC_YAW_RATE_MIN);

	//RC expect throttle scale range: 0 ~ 90 (degree), no offset
	radio_control_data->throttle_scale =
		(float)(get_radio_control_value(THROTTLE_CHANNEL) - RC_THROTTLE_MIN) / (RC_THROTTLE_MAX - RC_THROTTLE_MIN) *
			(RC_THROTTLE_SCALE_MAX - RC_THROTTLE_SCALE_MIN);
	bound_float(&radio_control_data->throttle_scale, RC_THROTTLE_SCALE_MAX, RC_THROTTLE_SCALE_MIN);
} 

int radio_control_safety_check(radio_control_t *radio_control_data)
{
	//Not pass if engine is on
	if(radio_control_data->safety_status != ENGINE_OFF) return 1;
	//No pass if throttle scale is over 10 persent
	else if(radio_control_data->throttle_scale > 10.0) return 1;

	/* Not pass if no RC value */
	if(get_radio_control_value(ROLL_CHANNEL) == 0) return 1;
	if(get_radio_control_value(PITCH_CHANNEL) == 0) return 1;
	if(get_radio_control_value(YAW_CHANNEL) == 0) return 1;
	if(get_radio_control_value(THROTTLE_CHANNEL) == 0) return 1;
	if(get_radio_control_value(SAFETY_CHANNEL) == 0) return 1;

	return 0; //Pass
}

#if 0
void debug_print_radio_control_expect_value(radio_control_t *radio_control_data)
{
	if(radio_control_data->safety_status == ENGINE_ON) {
		debug_print("Safety status: Engine ON\n\r");
	} else {
		debug_print("Safety status: Engine OFF\n\r");
	}


	debug_print("Expect roll: %f\n\r", radio_control_data->roll_angle);
	debug_print("Expect pitch: %f\n\r", radio_control_data->pitch_angle);
	debug_print("Expect yaw rate: %f\n\r", radio_control_data->yaw_rate);
	debug_print("Expect throttle scale: %f\n\r", radio_control_data->throttle_scale);


	delay_ms(50);
	debug_print("\x1b[H\x1b[2J");
}
#endif
