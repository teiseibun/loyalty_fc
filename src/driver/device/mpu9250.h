#ifndef __MPU9250_H__
#define __MPU9250_H__

#include "stm32f4xx_conf.h"
#include "spi.h"
#include "vector.h"

#define mpu9250_chip_select() GPIO_ResetBits(GPIOA, GPIO_Pin_4)
#define mpu9250_chip_deselect() GPIO_SetBits(GPIOA, GPIO_Pin_4)

#define MPU9250_SMPLRT_DIV 0x19
#define MPU9250_CONFIG 0x1A
#define MPU9250_GYRO_CONFIG 0x1B
#define MPU9250_ACCEL_CONFIG 0x1C
#define MPU9250_MOT_THR 0x1F
#define MPU9250_FIFO_EN 0x23
#define MPU9250_INT_ENABLE 0x38
#define MPU9250_INT_STATUS 0x3A
#define MPU9250_ACCEL_XOUT_H 0x3B
#define MPU9250_ACCEL_XOUT_L 0x3C
#define MPU9250_ACCEL_YOUT_H 0x3D
#define MPU9250_ACCEL_YOUT_L 0x3E
#define MPU9250_ACCEL_ZOUT_H 0x3F
#define MPU9250_ACCEL_ZOUT_L 0x40
#define MPU9250_TEMP_OUT_H 0x41
#define MPU9250_TEMP_OUT_L 0x42
#define MPU9250_GYRO_XOUT_H 0x43
#define MPU9250_GYRO_XOUT_L 0x44
#define MPU9250_GYRO_YOUT_H 0x45
#define MPU9250_GYRO_YOUT_L 0x46
#define MPU9250_GYRO_ZOUT_H 0x47
#define MPU9250_GYRO_ZOUT_L 0x48
#define MPU9250_USER_CTRL 0x6A
#define MPU9250_PWR_MGMT_1 0x6B
#define MPU9250_PWR_MGMT_2 0x6C
#define MPU9250_FIFO_COUNTH 0x72
#define MPU9250_FIFO_COUNTL 0x73
#define MPU9250_FIFO_R_W 0x74
#define MPU9250_WHO_AM_I 0x75

#define MPU9250A_2g 0.0000610352f 
#define MPU9250A_4g 0.0001220703f
#define MPU9250A_8g 0.0002441406f
#define MPU9250A_16g 0.0004882813f

#define MPU9250G_250dps 0.007633587786f
#define MPU9250G_500dps 0.015267175572f
#define MPU9250G_1000dps 0.030487804878f
#define MPU9250G_2000dps 0.060975609756f

#define MPU9250T_85degC 0.00294f

int mpu9250_init();

void mpu9250_read_unscaled_data(vector3d_16_t *accel_unscaled_data, vector3d_16_t *gyro_unscaled_data);

void mpu9250_fix_bias(vector3d_16_t *accel_unscaled_data,
	vector3d_16_t *gyro_unscaled_data);

void mpu9250_accel_convert_to_scale(vector3d_16_t *accel_unscaled_data,
	vector3d_f_t *accel_scaled_data);
void mpu9250_gyro_convert_to_scale(vector3d_16_t *gyro_unscaled_data,
	vector3d_f_t *gyro_scaled_data);

#endif
