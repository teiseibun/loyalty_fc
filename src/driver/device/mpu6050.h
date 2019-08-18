#ifndef __MPU6050_H
#define __MPU6050_H

#include "vector_space.h"

/* MPU6050 register address */
#define MPU6050_DEVICE_ADDRESS 0x68 << 1

#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C

#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40

#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_GYRO_XOUT_L 0x44
#define MPU6050_GYRO_YOUT_H 0x45
#define MPU6050_GYRO_YOUT_L 0x46
#define MPU6050_GYRO_ZOUT_H 0x47
#define MPU6050_GYRO_ZOUT_L 0x48

/* MPU6050 scale value */
#define MPU6050A_2g 0.0000610352f
#define MPU6050A_4g 0.0001220703f
#define MPU6050A_8g 0.0002441406f
#define MPU6050A_16g 0.0004882813f

#define MPU6050G_250dps 0.007633587786f
#define MPU6050G_500dps 0.015267175572f
#define MPU6050G_1000dps 0.030487804878f
#define MPU6050G_2000dps 0.060975609756f

uint8_t mpu6050_read_who_am_i();

int mpu6050_init();

void mpu6050_read_unscaled_data(vector3d_16_t *accel_unscaled_data, vector3d_16_t *gyro_unscaled_data);

void mpu6050_fix_bias(vector3d_16_t *accel_unscaled_data,
	vector3d_16_t *gyro_unscaled_data);

void mpu6050_accel_convert_to_scale(vector3d_16_t *accel_unscaled_data,
	vector3d_f_t *accel_scaled_data);
void mpu6050_gyro_convert_to_scale(vector3d_16_t *gyro_unscaled_data,
	vector3d_f_t *gyro_scaled_data);

#endif
