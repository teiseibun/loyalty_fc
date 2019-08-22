#include "mpu6050.h"
#include "ahrs.h"
#include "vector.h"

imu_t imu;
ahrs_t ahrs;

void ahrs_ekf_loop(void)
{
	mpu6050_read_unscaled_data(&imu.unscaled_accel, &imu.unscaled_gyro);
	mpu6050_fix_bias(&imu.unscaled_accel, &imu.unscaled_gyro);
	mpu6050_accel_convert_to_scale(&imu.unscaled_accel, &imu.raw_accel);
	mpu6050_gyro_convert_to_scale(&imu.unscaled_gyro, &imu.raw_gyro);
}
