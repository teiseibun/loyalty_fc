#include <stdint.h>
#include <string.h>

#include "uart.h"
#include "link.h"
#include "vector.h"
#include "ahrs.h"

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

        checksum = generate_checksum_byte(payload + 2, payload_count - 2);

	payload[0] = '@';
	payload[1] = payload_count - 2;

	payload[payload_count] = checksum;
	payload_count++;

        uart3_puts(payload, payload_count);
}

void telemetry_loop()
{
	if(uart3_tx_busy() == true)
		return;

	imu_t imu;
	ahrs_t ahrs;
	float alpha_roll, alpha_pitch;

	imu.raw_accel.x = 0.0f;
	imu.raw_accel.y = 0.1f;
	imu.raw_accel.z = 0.2f;
	imu.filtered_accel.x = 0.3f;
	imu.filtered_accel.y = 0.4f;
	imu.filtered_accel.z = 0.5f;
	imu.raw_gyro.x = 0.6f;
	imu.raw_gyro.y = 0.7f;
	imu.raw_gyro.z = 0.8f;
	imu.filtered_gyro.x = 0.9f;
	imu.filtered_gyro.y = 1.0f;
	imu.filtered_gyro.z = 1.1f;
	ahrs.accel.roll = 1.2f;
	ahrs.accel.pitch = 1.3f;
	ahrs.gyro.roll = 1.4f;
	ahrs.gyro.pitch = 1.5f;
	ahrs.gyro.yaw = 1.6f;
	ahrs.mag.yaw = 1.7f;
	alpha_roll = 1.8f;
	alpha_pitch = 1.9f;

	uint8_t payload[512] = {0}; //~64 float
	int payload_size = 2; //reserved for header message
	
	payload_size += pack_vector3d(&imu.raw_accel, payload + payload_size);
	payload_size += pack_vector3d(&imu.filtered_accel, payload + payload_size);
	payload_size += pack_vector3d(&imu.raw_gyro, payload + payload_size);
	payload_size += pack_vector3d(&imu.filtered_gyro, payload + payload_size);
	payload_size += pack_float(&ahrs.accel.roll, payload + payload_size);
	payload_size += pack_float(&ahrs.accel.pitch, payload + payload_size);
	payload_size += pack_float(&ahrs.mag.yaw, payload + payload_size);
	payload_size += pack_attitude(&ahrs.gyro, payload + payload_size);
	payload_size += pack_float(&alpha_roll, payload + payload_size);
	payload_size += pack_float(&alpha_pitch, payload + payload_size);

	send_onboard_data(payload, payload_size);
}
