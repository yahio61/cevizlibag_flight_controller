/*
 * queternion.h
 *
 *  Created on: Jul 11, 2024
 *      Author: yahya
 */

#ifndef INC_QUETERNION_H_
#define INC_QUETERNION_H_
#include "main.h"
#include "bmi088.h"

extern bmi088_struct_t BMI_sensor;

void quaternon_init(float *backup_datas);
void updateQuaternion(float gx, float gy, float gz, float dt);
void quaternionToEuler(void);
void quaternionSet_zero(void);
void getInitialQuaternion(float acc_x, float acc_y, float acc_z);
float quaternionToTheta(void);
#endif
