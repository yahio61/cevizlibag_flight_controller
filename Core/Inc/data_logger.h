/*
 * data_logger.h
 *
 *  Created on: Jun 7, 2025
 *      Author: yahya
 */

#ifndef INC_DATA_LOGGER_H_
#define INC_DATA_LOGGER_H_
#include "main.h"

typedef struct data_logging_s
{
	uint32_t start;
	uint32_t time;
	float altitude_filtered;
	float altitude_normal;
	float velocity_filtered;
	float pressure;
	float temperature;
	float humidity;
	float accel_x;
	float accel_y;
	float accel_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;
	float roll;
	float pitch;
	float yaw;
	float q0;
	float q1;
	float q2;
	float q3;
	float lat;
	float lon;
	float GNSS_alittude;
	float GNSS_time;
	uint32_t status;
	float voltage;
	float power;
	uint32_t sat_count;
	uint32_t stop;
}data_logging_t;

typedef union data_log_s_to_arr
{
	data_logging_t datas;
	uint8_t arr[sizeof(data_logging_t)];
}data_to_arr_t;

void data_logger_init();

void log_datas(int page_adr, uint32_t time, float altitude_filtered, float altitude_normal, float velocity_filtered,
		float pressure, float temperature, float humidity, float accel_x, float accel_y, float accel_z,
		float gyro_x, float gyro_y, float gyro_z, float roll, float pitch, float yaw,
		float q0, float q1, float q2, float q3, float lat, float lon,
		float GNSS_altitude, float GNSS_time, uint32_t status, float voltage, float power, uint32_t sat_count);

void read_logged_datas(int page_adr);

#endif /* INC_DATA_LOGGER_H_ */
