/*
 * algorithms.h
 *
 *  Created on: May 10, 2024
 *      Author: yahya
 */

#ifndef INC_ALGORITHMS_H_
#define INC_ALGORITHMS_H_

#include "main.h"
#include "bmi088.h"
#include "bme280.h"

typedef uint8_t algorithmStatus;

void reset_algorithm_status();
flight_states_e algorithm_update(flight_data_t *rocket, uint32_t);
//void algorithm_2_update(bme280_struct_t* BME, bmi088_struct_t* BMI);

#endif /* INC_ALGORITHMS_H_ */
