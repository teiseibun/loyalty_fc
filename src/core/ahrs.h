#ifndef __AHRS__
#define __AHRS__

#include "vector.h"

typedef struct {
        vector3d_16_t unscaled_accel;
        vector3d_16_t unscaled_gyro;
        vector3d_16_t unscaled_mag;

        vector3d_f_t raw_accel;
        vector3d_f_t raw_gyro;
        vector3d_f_t raw_mag;

        vector3d_f_t filtered_accel;
        vector3d_f_t filtered_gyro;
        vector3d_f_t filtered_mag;
} imu_t;

typedef struct {
        float roll;
        float pitch;
        float yaw;
} attitude_t;

typedef struct {
        attitude_t accel;
        attitude_t gyro;
        attitude_t mag;
        attitude_t fusion;
} ahrs_t;

#endif
