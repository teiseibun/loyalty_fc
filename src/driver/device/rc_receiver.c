#include "stm32f4xx.h"

#include "uart.h"

#include "rc_receiver.h"

#include "delay.h"

#define RC_CHANNEL_CNT 6

static int rise_value[RC_CHANNEL_CNT] = {0};
static int fall_value[RC_CHANNEL_CNT] = {0};
static int rc_value[RC_CHANNEL_CNT] = {0}; /* Should initialized with RC minimized value */
static PWM_State capture_state[RC_CHANNEL_CNT] = {RISE};

void TIM2_IRQHandler()
{
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICStructInit(&TIM_ICInitStructure); //Initialize the structure with default setting

	if(TIM_GetITStatus(TIM2, TIM_IT_CC3) == SET) {
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);

		TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;

		if(capture_state[ROLL_CHANNEL] == RISE) {
			//Rised edge capture
			rise_value[ROLL_CHANNEL] = TIM_GetCapture3(TIM2);

			//Change to measure the falled edge
			capture_state[ROLL_CHANNEL] = FALL;
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
		} else {
			//Falled edge capture
			fall_value[ROLL_CHANNEL] = TIM_GetCapture3(TIM2);

			/* Calculate the RC high pulse value */
			if(fall_value[ROLL_CHANNEL] > rise_value[ROLL_CHANNEL])
				rc_value[ROLL_CHANNEL] = fall_value[ROLL_CHANNEL] - rise_value[ROLL_CHANNEL];
			else if(fall_value[ROLL_CHANNEL] == 0 && rise_value[ROLL_CHANNEL] == 0) /* No signal */
				rc_value[ROLL_CHANNEL] = 0;
			else /* falled edge measured value is overflow */
				rc_value[ROLL_CHANNEL] = (fall_value[ROLL_CHANNEL] + 0xFFFF) - rise_value[ROLL_CHANNEL];

			//Change to measure the rised edge
			capture_state[ROLL_CHANNEL] = RISE;
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
		}

		TIM_ICInit(TIM2, &TIM_ICInitStructure);
	}

	if(TIM_GetITStatus(TIM2, TIM_IT_CC4) == SET) {
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);

		TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;

		if(capture_state[PITCH_CHANNEL] == RISE) {
			//Rised edge capture
			rise_value[PITCH_CHANNEL] = TIM_GetCapture4(TIM2);

			//Change to measure the falled edge
			capture_state[PITCH_CHANNEL] = FALL;
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
		} else {
			//Falled edge capture
			fall_value[PITCH_CHANNEL] = TIM_GetCapture4(TIM2);

			/* Calculate the RC high pulse value */
			if(fall_value[PITCH_CHANNEL] > rise_value[PITCH_CHANNEL])
				rc_value[PITCH_CHANNEL] = fall_value[PITCH_CHANNEL] - rise_value[PITCH_CHANNEL];
			else if(fall_value[PITCH_CHANNEL] == 0 && rise_value[PITCH_CHANNEL] == 0) /* No signal */
				rc_value[PITCH_CHANNEL] = 0;
			else /* falled edge measured value is overflow */
				rc_value[PITCH_CHANNEL] = (fall_value[PITCH_CHANNEL] + 0xFFFF) - rise_value[PITCH_CHANNEL];

			//Change to measure the rised edge
			capture_state[PITCH_CHANNEL] = RISE;
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
		}

		TIM_ICInit(TIM2, &TIM_ICInitStructure);
	}
}

void TIM3_IRQHandler()
{
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICStructInit(&TIM_ICInitStructure); //Initialize the structure with default setting

	if(TIM_GetITStatus(TIM3, TIM_IT_CC3) == SET) {
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);

		TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;

		if(capture_state[YAW_CHANNEL] == RISE) {
			//Rised edge capture
			rise_value[YAW_CHANNEL] = TIM_GetCapture3(TIM3);

			//Change to measure the falled edge
			capture_state[YAW_CHANNEL] = FALL;
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
		} else {
			//Falled edge capture
			fall_value[YAW_CHANNEL] = TIM_GetCapture3(TIM3);

			/* Calculate the RC high pulse value */
			if(fall_value[YAW_CHANNEL] > rise_value[YAW_CHANNEL])
				rc_value[YAW_CHANNEL] = fall_value[YAW_CHANNEL] - rise_value[YAW_CHANNEL];
			else if(fall_value[YAW_CHANNEL] == 0 && rise_value[YAW_CHANNEL] == 0)
				rc_value[YAW_CHANNEL] = 0; //No signal
			else /* falled edge measured value is overflow */
				rc_value[YAW_CHANNEL] = (fall_value[YAW_CHANNEL] + 0xFFFF) - rise_value[YAW_CHANNEL];

			//Change to measure the rised edge
			capture_state[YAW_CHANNEL] = RISE;
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
		}

		TIM_ICInit(TIM3, &TIM_ICInitStructure);
	}

	if(TIM_GetITStatus(TIM3, TIM_IT_CC4) == SET) {
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);

		TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;

		if(capture_state[THROTTLE_CHANNEL] == RISE) {
			//Rised edge capture
			rise_value[THROTTLE_CHANNEL] = TIM_GetCapture4(TIM3);

			//Change to measure the falled edge
			capture_state[THROTTLE_CHANNEL] = FALL;
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
		} else {
			//Falled edge capture
			fall_value[THROTTLE_CHANNEL] = TIM_GetCapture4(TIM3);

			/* Calculate the RC high pulse value */
			if(fall_value[THROTTLE_CHANNEL] > rise_value[THROTTLE_CHANNEL])
				rc_value[THROTTLE_CHANNEL] = fall_value[THROTTLE_CHANNEL] - rise_value[THROTTLE_CHANNEL];
			else if(fall_value[THROTTLE_CHANNEL] == 0 && rise_value[THROTTLE_CHANNEL] == 0)
				rc_value[THROTTLE_CHANNEL] = 0; //No signal
			else /* falled edge measured value is overflow */
				rc_value[THROTTLE_CHANNEL] = (fall_value[THROTTLE_CHANNEL] + 0xFFFF) - rise_value[THROTTLE_CHANNEL];

			//Change to measure the rised edge
			capture_state[THROTTLE_CHANNEL] = RISE;
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
		}

		TIM_ICInit(TIM3, &TIM_ICInitStructure);
	}
}

void TIM8_CC_IRQHandler()
{
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICStructInit(&TIM_ICInitStructure); //Initialize the structure with default setting

	if(TIM_GetITStatus(TIM8, TIM_IT_CC1) == SET) {
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC1);

		TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;

		if(capture_state[SAFETY_CHANNEL] == RISE) {
			//Rised edge capture
			rise_value[SAFETY_CHANNEL] = TIM_GetCapture1(TIM8);

			//Change to measure the falled edge
			capture_state[SAFETY_CHANNEL] = FALL;
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
		} else {
			//Falled edge capture
			fall_value[SAFETY_CHANNEL] = TIM_GetCapture1(TIM8);

			/* Calculate the RC high pulse value */
			if(fall_value[SAFETY_CHANNEL] > rise_value[SAFETY_CHANNEL])
				rc_value[SAFETY_CHANNEL] = fall_value[SAFETY_CHANNEL] - rise_value[SAFETY_CHANNEL];
			else if(fall_value[SAFETY_CHANNEL] == 0 && rise_value[SAFETY_CHANNEL] == 0)
				rc_value[SAFETY_CHANNEL] = 0; //No signal
			else /* falled edge measured value is overflow */
				rc_value[SAFETY_CHANNEL] = (fall_value[SAFETY_CHANNEL] + 0xFFFF) - rise_value[SAFETY_CHANNEL];

			//Change to measure the rised edge
			capture_state[SAFETY_CHANNEL] = RISE;
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
		}

		TIM_ICInit(TIM8, &TIM_ICInitStructure);
	}

	if(TIM_GetITStatus(TIM8, TIM_IT_CC2) == SET) {
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC2);

		TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;

		if(capture_state[RESERVE_CHANNEL_1] == RISE) {
			//Rised edge capture
			rise_value[RESERVE_CHANNEL_1] = TIM_GetCapture2(TIM8);

			//Change to measure the falled edge
			capture_state[RESERVE_CHANNEL_1] = FALL;
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
		} else {
			//Falled edge capture
			fall_value[RESERVE_CHANNEL_1] = TIM_GetCapture2(TIM8);

			/* Calculate the RC high pulse value */
			if(fall_value[RESERVE_CHANNEL_1] > rise_value[RESERVE_CHANNEL_1])
				rc_value[RESERVE_CHANNEL_1] = fall_value[RESERVE_CHANNEL_1] - rise_value[RESERVE_CHANNEL_1];
			else if(fall_value[RESERVE_CHANNEL_1] == 0 && rise_value[RESERVE_CHANNEL_1] == 0)
				rc_value[RESERVE_CHANNEL_1] = 0; //No signal
			else /* falled edge measured value is overflow */
				rc_value[RESERVE_CHANNEL_1] = (fall_value[RESERVE_CHANNEL_1] + 0xFFFF) - rise_value[RESERVE_CHANNEL_1];

			//Change to measure the rised edge
			capture_state[RESERVE_CHANNEL_1] = RISE;
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
		}

		TIM_ICInit(TIM8, &TIM_ICInitStructure);
	}
}

int get_radio_control_value(int channel)
{
	return rc_value[channel];
}

void debug_print_radio_control_value()
{
	printf("rc ch1 (roll): %d\n\r", rc_value[ROLL_CHANNEL]);
	printf("rc ch2 (pitch): %d\n\r", rc_value[PITCH_CHANNEL]);
	printf("rc ch3 (throttle): %d\n\r", rc_value[THROTTLE_CHANNEL]);
	printf("rc ch4 (yaw): %d\n\r", rc_value[YAW_CHANNEL]);
	printf("rc ch5 (safety): %d\n\r", rc_value[SAFETY_CHANNEL]);
	printf("rc reserve: %d", rc_value[RESERVE_CHANNEL_1]);

	delay_ms(100);
	printf("\x1b[H\x1b[2J");
	delay_ms(1);
}
