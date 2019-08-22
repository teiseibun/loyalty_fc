#include "mpu6050.h"
#include "ahrs.h"
#include "vector.h"
#include "lpf.h"

imu_t imu;
ahrs_t ahrs;

vector3d_f_t accel_lpf_old, gyro_lpf_old;

void ahrs_ekf_init(void)
{
	mpu6050_read_unscaled_data(&imu.unscaled_accel, &imu.unscaled_gyro);
	mpu6050_fix_bias(&imu.unscaled_accel, &imu.unscaled_gyro);
	mpu6050_accel_convert_to_scale(&imu.unscaled_accel, &imu.raw_accel);
	mpu6050_gyro_convert_to_scale(&imu.unscaled_gyro, &imu.raw_gyro);
	accel_lpf_old = imu.raw_accel;
	gyro_lpf_old = imu.raw_gyro;
}

void ahrs_ekf_loop(void)
{
	mpu6050_read_unscaled_data(&imu.unscaled_accel, &imu.unscaled_gyro);
	mpu6050_fix_bias(&imu.unscaled_accel, &imu.unscaled_gyro);
	mpu6050_accel_convert_to_scale(&imu.unscaled_accel, &imu.raw_accel);
	mpu6050_gyro_convert_to_scale(&imu.unscaled_gyro, &imu.raw_gyro);
	
	lpf_ema_vector3d(&imu.raw_accel, &accel_lpf_old, &imu.filtered_accel, 0.9);
	lpf_ema_vector3d(&imu.raw_gyro, &gyro_lpf_old, &imu.filtered_gyro, 0.9);
}
