#ifndef __RC_RECEIVER_H
#define __RC_RECEIVER_H

/* Radio control value range definitions,
   you should calibrate these by yourself! */
#define RC_ROLL_MAX 27019
#define RC_ROLL_MIN 15525

#define RC_PITCH_MAX 27019
#define RC_PITCH_MIN 15538

#define RC_YAW_MAX 27019
#define RC_YAW_MIN 15525

#define RC_THROTTLE_MAX 27019
#define RC_THROTTLE_MIN 15538

#define RC_SAFETY_MAX 28979
#define RC_SAFETY_MIN 13578

typedef enum {FALL, RISE} PWM_State;

enum {
	ROLL_CHANNEL,
	PITCH_CHANNEL,
	YAW_CHANNEL,
	THROTTLE_CHANNEL,
	SAFETY_CHANNEL,
	RESERVE_CHANNEL_1
} RC_Channel;

int get_radio_control_value(int channel);

void debug_print_radio_control_value();

#endif
