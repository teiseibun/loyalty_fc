#ifndef __LINK__ 
#define __LINK__

#include <stdint.h>

typedef struct {
	int graph_count;
	uint8_t *payload;
	int payload_count;
} package_t;

void insert_new_graph(package_t *package, char *data_name, float *data_float, int data_cnt);
void send_onboard_data(package_t *package);

#endif
