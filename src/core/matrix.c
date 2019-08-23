#include <stdbool.h>

#include "arm_math.h"
#include "matrix.h"
#include "uart.h"
#include "led.h"

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

void matrix_unit_test(void)
{
	//matrix unit tests
	MAT_ALLOC(A, 3, 3) = {1, 2, 3, 0, 0, 0, 2, 2, 3};
	MAT_INIT(A, 3, 3);
	MAT_ALLOC(A_trans, 3, 3);
	MAT_INIT(A_trans, 3, 3);
	MAT_ALLOC(A_inv, 3, 3);
	MAT_INIT(A_inv, 3, 3);
	MAT_ALLOC(B, 3, 3) = {1, 0, 0, 1, 1, 1, 0, 0, 0};
	MAT_INIT(B, 3, 3);
	MAT_ALLOC(C, 3, 3) = {0};
	MAT_INIT(C, 3, 3);

#if 1
	MAT_ADD(&A, &B, &C);
	print_matrix(C_arr, 3, 3);
	MAT_SUB(&A, &B, &C);
	print_matrix(C_arr, 3, 3);
	MAT_MULT(&A, &B, &C);
	print_matrix(C_arr, 3, 3);
	MAT_TRANS(&A, &A_trans);
	print_matrix(A_trans_arr, 3, 3);
	MAT_INV(&A, &A_inv);
	print_matrix(A_inv_arr, 3, 3);
#endif
}
