#include <math.h>

#include "arm_math.h"
#include "led.h"
#include "mpu6050.h"
#include "ahrs.h"
#include "vector.h"
#include "lpf.h"
#include "uart.h"
#include "matrix.h"

#define dt 0.002 //500Hz = 0.002s

imu_t imu;
ahrs_t ahrs;

vector3d_f_t accel_lpf_old, gyro_lpf_old;

MAT_ALLOC(x, 4, 1);
MAT_ALLOC(w, 3, 1);
MAT_ALLOC(y, 3, 1);
MAT_ALLOC(f, 4, 3);
MAT_ALLOC(h_x, 3, 1);
MAT_ALLOC(resid, 3, 1);
//MAT_ALLOC(F, 4, 4);
//MAT_ALLOC(P, 4, 4);
//MAT_ALLOC(R, 3, 3);
//MAT_ALLOC(Q, 3, 3);
MAT_ALLOC(H, 4, 3);
MAT_ALLOC(K, 4, 3);
//-----------------------
MAT_ALLOC(dx, 4, 1);
MAT_ALLOC(I, 4, 4) = {1, 0, 0 ,0,
		      0, 1, 0, 0,
		      0, 0, 1, 0,
		      0, 0, 0, 1};

void ahrs_ekf_init(void)
{
	//initialize matrices
	MAT_INIT(x, 4, 1);
	MAT_INIT(w, 3, 1);
	MAT_INIT(y, 3, 1);
	MAT_INIT(f, 4, 3);
	MAT_INIT(h_x, 3, 1);
	MAT_INIT(resid, 3, 1);
//	MAT_INIT(F, 4, 4);
//	MAT_INIT(P, 4, 4);
//	MAT_INIT(R, 4, 4);
//	MAT_INIT(Q, 3, 3);
	MAT_INIT(H, 4, 3);
	MAT_INIT(K, 4, 3);
	//-----------------------
	MAT_INIT(dx, 4, 1);
	MAT_INIT(I, 4, 4);

	//initialize lpf
	mpu6050_read_unscaled_data(&imu.unscaled_accel, &imu.unscaled_gyro);
	mpu6050_fix_bias(&imu.unscaled_accel, &imu.unscaled_gyro);
	mpu6050_accel_convert_to_scale(&imu.unscaled_accel, &imu.raw_accel);
	mpu6050_gyro_convert_to_scale(&imu.unscaled_gyro, &imu.raw_gyro);
	accel_lpf_old = imu.raw_accel;
	gyro_lpf_old = imu.raw_gyro;

	attitude_t att_init = {
		.roll = 0.0f,
		.pitch = 0.0f,
		.yaw = 0.0f
	};
	euler_to_quat(&att_init, &_mat_(x)[0]);
}

//in: euler angle [radian], out: quaternion
void euler_to_quat(attitude_t *euler, float *q)
{
	float phi = euler->roll * 0.5f;
	float theta = euler->pitch * 0.5f;
	float psi = euler->yaw * 0.5f;

	q[0] = arm_cos_f32(phi)*arm_cos_f32(theta)*arm_cos_f32(psi) +
		arm_sin_f32(phi)*arm_sin_f32(theta)*arm_sin_f32(psi);
	q[1] = arm_sin_f32(phi)*arm_cos_f32(theta)*arm_cos_f32(psi) -
		arm_cos_f32(phi)*arm_sin_f32(theta)*arm_sin_f32(psi);
	q[2] = arm_cos_f32(phi)*arm_sin_f32(theta)*arm_cos_f32(psi) +
		arm_sin_f32(phi)*arm_cos_f32(theta)*arm_sin_f32(psi);
	q[3] = arm_cos_f32(phi)*arm_cos_f32(theta)*arm_sin_f32(psi) -
		arm_sin_f32(phi)*arm_sin_f32(theta)*arm_cos_f32(psi);
}

void quat_normalize(float *q)
{
	float sq_sum = (q[0])*(q[0]) + (q[1])*(q[1]) + (q[2])*(q[2]) + (q[3])*(q[3]);
	float norm;
	arm_sqrt_f32(sq_sum, &norm);
	q[0] /= norm;
	q[1] /= norm;
	q[2] /= norm;
	q[3] /= norm;
}

//in: quaterion, out: euler angle [radian]
void quat_to_euler(float *q, attitude_t *euler)
{
	euler->roll = atan2(2.0*(q[0]*q[1] + q[2]*q[3]), 1.0-2.0*(q[1]*q[1] + q[2]*q[2]));
	euler->pitch = asin(2.0*(q[0]*q[2] - q[3]*q[1]));
	euler->yaw = atan2(2.0*(q[0]*q[3] + q[1]*q[2]), 1.0-2.0*(q[2]*q[2] + q[3]*q[3]));
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

	_mat_(w)[0] = deg_to_rad(imu.filtered_gyro.x);
	_mat_(w)[1] = deg_to_rad(imu.filtered_gyro.y);
	_mat_(w)[2] = deg_to_rad(imu.filtered_gyro.z);

	MAT_MULT(&f, &w, &dx);
	MAT_ADD(&x, &dx, &x);

	quat_normalize(&_mat_(x)[0]);

#if 1
	quat_to_euler(&_mat_(x)[0], &ahrs.attitude);
        ahrs.attitude.roll = rad_to_deg(ahrs.attitude.roll);
	ahrs.attitude.pitch = rad_to_deg(ahrs.attitude.pitch);
#endif

#if 0   //debug print messages
	if(uart3_tx_busy() == false) {
		//print_matrix(_mat_(dx), 4, 1);
		//print_matrix(_mat_(f), 4, 3);
	}
#endif
}

void ahr_ekf_state_update(void)
{
	float q0 = _mat_(x)[0];
	float q1 = _mat_(x)[1];
	float q2 = _mat_(x)[2];
	float q3 = _mat_(x)[3];

	//predicted estimation
	_mat_(h_x)[0] = 2.0*(q1*q3 - q0*q2);
	_mat_(h_x)[1] = 2.0*(q2*q3 + q0*q1);
	_mat_(h_x)[2] = q0*q0 + q1*q1 - q2*q2 + q3*q3;

	//sensor estimation
	//_mat_(y)[0] = imu.filtered_accel.x;
	//_mat_(y)[1] = imu.filtered_accel.y;
	//_mat_(y)[2] = imu.filtered_accel.z;

	//residual = prediction - sensor estimation
	//MAT_SUB(&h_x, &y, &resid);

#if 0
	//debug print messages
        if(uart3_tx_busy() == false) {
		//print_matrix(_mat_(x), 4, 1);	
		//print_matrix(_mat_(h_x), 3, 1);	
		//print_matrix(_mat_(H), 4, 3);		
	}
#endif

	//kalman gain
	_mat_(H)[0]=-2.0*q2;  _mat_(H)[1]=+2.0*q3;  _mat_(H)[2]=-2.0*q0;   _mat_(H)[3]=+2.0*q1;
	_mat_(H)[4]=+2.0*q1;  _mat_(H)[5]=+2.0*q0;  _mat_(H)[6]=+2.0*q3;   _mat_(H)[7]=+2.0*q2;
	_mat_(H)[8]=+2.0*q0;  _mat_(H)[9]=-2.0*q1;  _mat_(H)[10]=-2.0*q2;  _mat_(H)[11]=+2.0*q3;
	//MAT_INV(&H, &H);

	//update inovation and prediction
	//MAT_MULT(&K, &resid, &dx);
	//MAT_ADD(&x, &dx, &x);

#if 0
	//debug print messages
        if(uart3_tx_busy() == false) {
		//print_matrix(_mat_(H), 4, 3);		
	}
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
	lpf_ema_vector3d(&imu.raw_accel, &accel_lpf_old, &imu.filtered_accel, 0.03);
	lpf_ema_vector3d(&imu.raw_gyro, &gyro_lpf_old, &imu.filtered_gyro, 0.03);

	ahr_ekf_state_predict();
	ahr_ekf_state_update();
	//calc_attitude_use_accel();
}
