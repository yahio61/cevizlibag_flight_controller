/*
 * filters.h
 *
 *  Created on: Aug 28, 2025
 *      Author: yahya
 */

#ifndef INC_FILTERS_H_
#define INC_FILTERS_H_
#include "main.h"

#define ACCEL_MAX_VALUE_THRUST 150.0f   // İtki fazında max ivme (m/s²)
#define ACCEL_MAX_STD_THRUST 150.0f      // İtki fazında max std sapma

#define ALT_MAX_VALUE 10000.0f   // Maksimum yükseklik (m)
#define ALT_MIN_VALUE -100.0f    // Minimum yükseklik (m)
#define ALT_MAX_STD 100.0f       // Maksimum ani değişim (standart sapma)

typedef struct {
    float h;      // durum: irtifa (m)
    float v;      // durum: hız (m/s) (pozitif yukarı)
    float abias;  // ivme bias (m/s^2)
    float alpha;  // alpha (irtifa düzeltme)
    float beta;   // beta (hız düzeltme)
    float last_time;
    float dt;
    float last_h_meas;
} BaroAccelFilter;

void baf_init(BaroAccelFilter *f, float a0, float alpha, float beta, float dt);
void baf_step(BaroAccelFilter *f, float h_meas, float a_meas, float time);


#endif /* INC_FILTERS_H_ */
