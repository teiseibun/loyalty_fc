#include <math.h>

#include "arm_math.h"
#include "led.h"
#include "mpu6050.h"
#include "ahrs.h"
#include "vector.h"
#include "lpf.h"
#include "quaternion.h"
#include "uart.h"
#include "matrix.h"

#define dt 0.002 //500Hz = 0.002s

imu_t imu;
ahrs_t ahrs;

vector3d_f_t accel_lpf_old, gyro_lpf_old;

MAT_ALLOC(x, 4, 1);
MAT_ALLOC(y, 3, 1);
MAT_ALLOC(f, 4, 3);
MAT_ALLOC(F, 4, 4);
MAT_ALLOC(P, 4, 4);
MAT_ALLOC(R, 3, 3);
MAT_ALLOC(Q, 3, 3);
MAT_ALLOC(H, 3, 4);
MAT_ALLOC(K, 4, 3);
MAT_ALLOC(FP, 4, 4);
MAT_ALLOC(PF, 4, 4);
//-----------------------
MAT_ALLOC(dx, 4, 1);
MAT_ALLOC(H_trans, 4, 3);
MAT_ALLOC(HPHR, 3, 3);
MAT_ALLOC(HPHR_inv, 3, 3);
MAT_ALLOC(I, 4, 4) = {1, 0, 0 ,0,
		      0, 1, 0, 0,
		      0, 0, 1, 0,
		      0, 0, 0, 1};

void ahrs_ekf_init(void)
{
	//initialize matrices
	MAT_INIT(x, 4, 1);
	MAT_INIT(y, 3, 1);
	MAT_INIT(f, 4, 3);
	MAT_INIT(F, 4, 4);
	MAT_INIT(P, 4, 4);
	MAT_INIT(R, 4, 4);
	MAT_INIT(Q, 3, 3);
	MAT_INIT(H, 3, 4);
	MAT_INIT(K, 4, 3);
	//-----------------------
	MAT_INIT(dx, 4, 1);
	MAT_INIT(H_trans, 4, 3);
	MAT_INIT(HPHR_inv, 3, 3);
	MAT_INIT(FP, 4, 4);
	MAT_INIT(PF, 4, 4);

	//initialize lpf
	mpu6050_read_unscaled_data(&imu.unscaled_accel, &imu.unscaled_gyro);
	mpu6050_fix_bias(&imu.unscaled_accel, &imu.unscaled_gyro);
	mpu6050_accel_convert_to_scale(&imu.unscaled_accel, &imu.raw_accel);
	mpu6050_gyro_convert_to_scale(&imu.unscaled_gyro, &imu.raw_gyro);
	accel_lpf_old = imu.raw_accel;
	gyro_lpf_old = imu.raw_gyro;

	quat_t q_init;
	attitude_t att_init = {
		.roll = 0.0f,
		.pitch = 0.0f,
		.yaw = 0.0f
	};
	euler_to_quat(&att_init, &q_init);
	_mat_(x)[0] = q_init.q0;
	_mat_(x)[1] = q_init.q1;
	_mat_(x)[2] = q_init.q2;
	_mat_(x)[3] = q_init.q3;
}

//in: euler angle [radian], out: quaternion
void euler_to_quat(attitude_t *euler, quat_t *q)
{
	float phi = euler->roll * 0.5f;
	float theta = euler->pitch * 0.5f;
	float psi = euler->yaw * 0.5f;

	q->q0 = arm_cos_f32(phi)*arm_cos_f32(theta)*arm_cos_f32(psi) +
		arm_sin_f32(phi)*arm_sin_f32(theta)*arm_sin_f32(psi);
	q->q1 = arm_sin_f32(phi)*arm_cos_f32(theta)*arm_cos_f32(psi) -
		arm_cos_f32(phi)*arm_sin_f32(theta)*arm_sin_f32(psi);
	q->q2 = arm_cos_f32(phi)*arm_sin_f32(theta)*arm_cos_f32(psi) +
		arm_sin_f32(phi)*arm_cos_f32(theta)*arm_sin_f32(psi);
	q->q3 = arm_cos_f32(phi)*arm_cos_f32(theta)*arm_sin_f32(psi) -
		arm_sin_f32(phi)*arm_sin_f32(theta)*arm_cos_f32(psi);
}

void quat_normalize(quat_t *q)
{
	float sq_sum = (q->q0)*(q->q0) + (q->q1)*(q->q1) + (q->q2)*(q->q2) + (q->q3)*(q->q3);
	float norm;
	arm_sqrt_f32(sq_sum, &norm);
	q->q0 /= norm;
	q->q1 /= norm;
	q->q2 /= norm;
	q->q3 /= norm;
}

//in: quaterion, out: euler angle [radian]
void quat_to_euler(quat_t *q, attitude_t *euler)
{
	euler->roll = atan2(2.0*(q->q0*q->q1 + q->q2*q->q3), 1.0-2.0*(q->q1*q->q1 + q->q2*q->q2));
	euler->pitch = asin(2.0*(q->q0*q->q2 - q->q3*q->q1));
	euler->yaw = atan2(2.0*(q->q0*q->q3 + q->q1*q->q2), 1.0-2.0*(q->q2*q->q2 + q->q3*q->q3));
}

void calc_attitude_use_accel(void)
{
        ahrs.attitude.roll = rad_to_deg(atan2(imu.filtered_accel.x, imu.filtered_accel.z));
	ahrs.attitude.pitch = rad_to_deg(atan2(-imu.filtered_accel.y, imu.filtered_accel.z));
}

void ahr_ekf_state_predict(void)
{
	float q0 = _mat_(x)[0];
	float q1 = _mat_(x)[1];
	float q2 = _mat_(x)[2];
	float q3 = _mat_(x)[3];

	_mat_(f)[0]=-0.5*dt*q1;   _mat_(f)[1]=-0.5*dt*q2;   _mat_(f)[2]=-0.5*dt*q3;
	_mat_(f)[3]=+0.5*dt*q0;   _mat_(f)[4]=-0.5*dt*q3;   _mat_(f)[5]=+0.5*dt*q2;
	_mat_(f)[6]=+0.5*dt*q3;   _mat_(f)[7]=+0.5*dt*q0;   _mat_(f)[8]=-0.5*dt*q1;
	_mat_(f)[9]=-0.5*dt*q2;   _mat_(f)[10]=+0.5*dt*q1;  _mat_(f)[11]=+0.5*dt*q0;

	_mat_(y)[0] = imu.filtered_gyro.x;
	_mat_(y)[1] = imu.filtered_gyro.y;
	_mat_(y)[2] = imu.filtered_gyro.z;

#if 0
	MAT_MULT(&f, &y, &dx);
	MAT_ADD(&x, &x, &dx);

	//print_matrix(_mat_(f), 4, 3);
	//print_matrix(_mat_(dx), 4, 1);
	//printf("%f, %f, %f, %f\n\r", _mat_(dx)[0], _mat_(dx)[1], _mat_(dx)[2], _mat_(dx)[3]);

	quat_t q;
	q.q0 = _mat_(x)[0];
	q.q1 = _mat_(x)[1];
	q.q2 = _mat_(x)[2];
	q.q3 = _mat_(x)[3];
	quat_normalize(&q);
	_mat_(x)[0] = q.q0;
	_mat_(x)[1] = q.q1;
	_mat_(x)[2] = q.q2;
	_mat_(x)[3] = q.q3;

	quat_to_euler(&q, &ahrs.attitude);
        ahrs.attitude.roll = rad_to_deg(ahrs.attitude.roll);
	ahrs.attitude.pitch = rad_to_deg(ahrs.attitude.pitch);
#endif
}

void ahrs_ekf_loop(void)
{
	//read new data from sensor
	mpu6050_read_unscaled_data(&imu.unscaled_accel, &imu.unscaled_gyro);
	mpu6050_fix_bias(&imu.unscaled_accel, &imu.unscaled_gyro);
	mpu6050_accel_convert_to_scale(&imu.unscaled_accel, &imu.raw_accel);
	mpu6050_gyro_convert_to_scale(&imu.unscaled_gyro, &imu.raw_gyro);
	
	//smooth imu signal with lpf
	lpf_ema_vector3d(&imu.raw_accel, &accel_lpf_old, &imu.filtered_accel, 0.01725);
	lpf_ema_vector3d(&imu.raw_gyro, &gyro_lpf_old, &imu.filtered_gyro, 0.01725);

	ahr_ekf_state_predict();
	//calc_attitude_use_accel();
}
