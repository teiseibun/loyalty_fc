#ifndef __LINK__ 
#define __LINK__

#include <stdint.h>

typedef struct {
	uint8_t *payload;
	int payload_count;
} package_t;

void telemetry_loop();

#endif
