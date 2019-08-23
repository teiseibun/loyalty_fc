#include <stdbool.h>

#include "arm_math.h"
#include "uart.h"

volatile arm_status mat_op_status = 0;

void print_matrix(float *mat_arr, int r, int c)
{
	int i, j;
	for(i = 0; i < r; i++) {
		for(j = 0; j < c; j++) {
			while(uart3_tx_busy() == true);
			printf("%f, ", mat_arr[i * c + j]);
		}
		while(uart3_tx_busy() == true);
		printf("\n\r");
	}
}
