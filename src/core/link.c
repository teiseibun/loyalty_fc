#include <stdint.h>
#include <string.h>

#include "uart.h"
#include "link.h"

void insert_new_graph(package_t *package, char *data_name, float *data_float, int data_cnt)
{
	//start byte
	if(package->payload_count == 0) {
		package->payload_count++;
		package->payload[0] = '@';
	}

	memcpy(&package->payload[package->payload_count], data_float, sizeof(float) * data_cnt);
	package->payload_count += sizeof(float) * data_cnt;

	//payload
	package->payload[package->payload_count] = '+';
	package->payload_count++;
}

static uint8_t generate_checksum(uint8_t *payload, int payload_count)
{
	uint8_t checksum_result = 0;

	int i;
	for(i = 0; i < payload_count; i++)
		checksum_result ^= payload[i];

	return checksum_result;
}

void send_onboard_data(package_t *package)
{
	//checksum
	package->payload_count--; //replace last byte '+'
	package->payload[package->payload_count] =
		generate_checksum(package->payload, package->payload_count);

	uart3_puts(package->payload, package->payload_count);
}
