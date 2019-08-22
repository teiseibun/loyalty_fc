#ifndef __LINK__ 
#define __LINK__

#include <stdint.h>

enum {
	MESSAGE_ID_IMU = 0,
	MESSAGE_ID_ATTITUDE = 1
} MESSAGE_ID;

typedef struct {
	uint8_t *payload;
	int payload_count;
} package_t;

void telemetry_loop();

#endif
