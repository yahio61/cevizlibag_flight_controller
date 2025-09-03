/*
 * queternion.c
 *
 *  Created on: Jul 11, 2024
 *      Author: yahya
 */
#include "queternion.h"
#include "math.h"

#define twoKpDef	(float)4			// hızlı düzeltme, sensör ivmelerine karşı daha dirençli
#define twoKiDef	(float)0.002				// gyro bias'ına karşı yavaş ama kararlı düzeltme

volatile float twoKp = twoKpDef;				// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;				// 2 * integral gain (Ki)
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki

float euler[3];		//pitch roll yaw
float *saved_q;



void quaternon_init(float *backup_datas)
{
	saved_q = backup_datas;
	quaternionSet_zero();
}
void updateQuaternion(float gx, float gy, float gz, float dt) {
  // Convert angular velocities to quaternion rates of change
  float qDot1 = 0.5f * (-saved_q[1] * gx - saved_q[2] * gy - saved_q[3] * gz);
  float qDot2 = 0.5f * (saved_q[0] * gx + saved_q[2] * gz - saved_q[3] * gy);
  float qDot3 = 0.5f * (saved_q[0] * gy - saved_q[1] * gz + saved_q[3] * gx);
  float qDot4 = 0.5f * (saved_q[0] * gz + saved_q[1] * gy - saved_q[2] * gx);

  // Integrate to get new quaternion values
  saved_q[0] += qDot1 * dt;
  saved_q[1] += qDot2 * dt;
  saved_q[2] += qDot3 * dt;
  saved_q[3] += qDot4 * dt;

  // Normalize quaternion to prevent drift
  float norm = sqrt(saved_q[0] * saved_q[0] + saved_q[1] * saved_q[1] + saved_q[2] * saved_q[2] + saved_q[3] * saved_q[3]);
  saved_q[0] /= norm;
  saved_q[1] /= norm;
  saved_q[2] /= norm;
  saved_q[3] /= norm;
}

void quaternionToEuler(void) {
  euler[1] = atan2(2.0f * (saved_q[0] * saved_q[1] + saved_q[2] * saved_q[3]), 1.0f - 2.0f * (saved_q[1] * saved_q[1] + saved_q[2] * saved_q[2])) * (180.0 / M_PI);
  euler[0] = asin(2.0f * (saved_q[0] * saved_q[2] - saved_q[3] * saved_q[1])) * (180.0 / M_PI);
  euler[2] = atan2(2.0f * (saved_q[0] * saved_q[3] + saved_q[1] * saved_q[2]), 1.0f - 2.0f * (saved_q[2] * saved_q[2] + saved_q[3] * saved_q[3])) * (180.0 / M_PI);
}


float quaternionToTheta(){

	float theta = 0.0;

	float r13 = 2 * saved_q[1] * saved_q[3] + 2 * saved_q[2] * saved_q[0];
	float r23 = 2 * saved_q[2] * saved_q[3] - 2 * saved_q[1] * saved_q[0];
	float r33 = 1 - 2 * saved_q[1] * saved_q[1] - 2 * saved_q[2] * saved_q[2];

	float z_x = r13;
	float z_y = r23;
	float z_z = r33;

	float dotProduct = z_z;
	float magnitude = sqrt(z_x * z_x + z_y * z_y + z_z * z_z);

	theta = acos(dotProduct / magnitude) * 180.0 / 3.14;
	return theta;
}

// İvmeölçerden başlangıç quaternioni hesaplama
void getInitialQuaternion(float acc_x, float acc_y, float acc_z) {

    float norm = sqrt(acc_z * acc_z + acc_x * acc_x + acc_y * acc_y);
    float accel_temp[3];

    accel_temp[0] = acc_x;
    accel_temp[1] = acc_y;
    accel_temp[2] = acc_z;

    accel_temp[0] /= norm;
    accel_temp[1] /= norm;
    accel_temp[2] /= norm;

    float q_temp[4];

    q_temp[0] = sqrt(1.0 -accel_temp[1]) * 0.5;
    float k = 0.5 / q_temp[0];
    q_temp[1] = accel_temp[0] * k * 0.5;
    q_temp[2] = accel_temp[2] * k * 0.5;
    q_temp[3] = 0.0;

    norm = sqrt(q_temp[0] * q_temp[0] + q_temp[1] * q_temp[1] + q_temp[2] * q_temp[2] + q_temp[3] * q_temp[3]);

    saved_q[0] = q_temp[0] / norm;
    saved_q[1] = q_temp[1] / norm;
    saved_q[2] = q_temp[2] / norm;
    saved_q[3] = 0.0f;
}

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax_f, float ay_f, float az_f, float dt)
{
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		if(!((ax_f == 0.0f) && (ay_f == 0.0f) && (az_f == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax_f * ax_f + ay_f * ay_f + az_f * az_f);
		ax_f *= recipNorm;
		ay_f *= recipNorm;
		az_f *= recipNorm;

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = saved_q[1] * saved_q[3] - saved_q[0] * saved_q[2];
		halfvy = saved_q[0] * saved_q[1] + saved_q[2] * saved_q[3];
		halfvz = saved_q[0] * saved_q[0] - 0.5f + saved_q[3] * saved_q[3];

		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay_f * halfvz - az_f * halfvy);
		halfey = (az_f * halfvx - ax_f * halfvz);
		halfez = (ax_f * halfvy - ay_f * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * dt;	// integral error scaled by Ki
			integralFBy += twoKi * halfey * dt;
			integralFBz += twoKi * halfez * dt;
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * dt);		// pre-multiply common factors
	gy *= (0.5f * dt);
	gz *= (0.5f * dt);
	qa = saved_q[0];
	qb = saved_q[1];
	qc = saved_q[2];
	saved_q[0] += (-qb * gx - qc * gy - saved_q[3] * gz);
	saved_q[1] += (qa * gx + qc * gz - saved_q[3] * gy);
	saved_q[2] += (qa * gy - qb * gz + saved_q[3] * gx);
	saved_q[3] += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = invSqrt(saved_q[0] * saved_q[0] + saved_q[1] * saved_q[1] + saved_q[2] * saved_q[2] + saved_q[3] * saved_q[3]);
	saved_q[0] *= recipNorm;
	saved_q[1] *= recipNorm;
	saved_q[2] *= recipNorm;
	saved_q[3] *= recipNorm;
}

void quaternionSet_zero(void)
{
	saved_q[0] = 1.0;
	saved_q[1] = 0.0;
	saved_q[2] = 0.0;
	saved_q[3] = 0.0;
}
