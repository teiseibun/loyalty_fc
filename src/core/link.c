#include <stdint.h>
#include <string.h>

#include "uart.h"
#include "link.h"
#include "vector.h"
#include "matrix.h"
#include "ahrs.h"

extern imu_t imu;
extern ahrs_t ahrs;

extern float _mat_(P)[4 * 4];
extern float _mat_(K)[4 * 4];

int pack_float(float *data_float, uint8_t *byte_to_sent)
{
	memcpy(byte_to_sent, (uint8_t *)data_float, sizeof(float));
	return sizeof(float);
}

int pack_vector3d(vector3d_f_t *data_vector3d, uint8_t *byte_to_sent)
{
	memcpy(byte_to_sent, (uint8_t *)data_vector3d, sizeof(vector3d_f_t));
	return sizeof(vector3d_f_t);
}

int pack_attitude(attitude_t *data_attitude, uint8_t *byte_to_sent)
{
	memcpy(byte_to_sent, (uint8_t *)data_attitude, sizeof(attitude_t));
	return sizeof(attitude_t);
}

static uint8_t generate_checksum_byte(uint8_t *payload, int payload_count)
{
	uint8_t result = 0;

	int i;
	for(i = 0; i < payload_count; i++)
		result ^= payload[i];

	return result;
}

static void send_onboard_data(uint8_t *payload, int payload_count)
{
        uint8_t checksum;

        checksum = generate_checksum_byte(payload + 3, payload_count - 3);

	payload[0] = '@';
	payload[1] = payload_count - 3;

	payload[payload_count] = checksum;
	payload_count++;

        uart3_puts(payload, payload_count);
}

void send_imu_message(void)
{
	uint8_t payload[512] = {0}; //~64 float
	int payload_size = 3; //reserved for header message

	payload[2] = MESSAGE_ID_IMU;

	payload_size += pack_vector3d(&imu.raw_accel, payload + payload_size);
	payload_size += pack_vector3d(&imu.filtered_accel, payload + payload_size);
	payload_size += pack_vector3d(&imu.raw_gyro, payload + payload_size);
	payload_size += pack_vector3d(&imu.filtered_gyro, payload + payload_size);

	send_onboard_data(payload, payload_size);
}

void send_attitude_message(void)
{
	uint8_t payload[512] = {0}; //~64 float
	int payload_size = 3; //reserved for header message
	payload[2] = MESSAGE_ID_ATTITUDE;

	payload_size += pack_attitude(&ahrs.attitude, payload + payload_size);

	send_onboard_data(payload, payload_size);
}

void send_attitude_imu_message(void)
{
	uint8_t payload[512] = {0}; //~64 float
	int payload_size = 3; //reserved for header message

	payload[2] = MESSAGE_ID_ATTITUDE_IMU;

	payload_size += pack_attitude(&ahrs.attitude, payload + payload_size);
	payload_size += pack_vector3d(&imu.filtered_accel, payload + payload_size);
	payload_size += pack_vector3d(&imu.filtered_gyro, payload + payload_size);

	send_onboard_data(payload, payload_size);
}

void send_ekf_message(void)
{
	uint8_t payload[512] = {0}; //~64 float
	int payload_size = 3; //reserved for header message

	payload[2] = MESSAGE_ID_EKF;

	payload_size += pack_float(&_mat_(P)[0], payload + payload_size);
	payload_size += pack_float(&_mat_(P)[5], payload + payload_size);
	payload_size += pack_float(&_mat_(P)[10], payload + payload_size);
	payload_size += pack_float(&_mat_(P)[15], payload + payload_size);
	payload_size += pack_float(&_mat_(K)[0], payload + payload_size);
	payload_size += pack_float(&_mat_(K)[5], payload + payload_size);
	payload_size += pack_float(&_mat_(K)[10], payload + payload_size);
	payload_size += pack_float(&_mat_(K)[15], payload + payload_size);

	send_onboard_data(payload, payload_size);
}

void telemetry_loop()
{
	if(uart3_tx_busy() == true)
		return;

	//send_imu_message();
	//send_attitude_message();
	//send_attitude_imu_message();
	send_ekf_message();
}
