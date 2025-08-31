/*
 * data_logger.c
 *
 *  Created on: Jun 7, 2025
 *      Author: yahya
 */
#include "data_logger.h"
#include "stdint.h"
#include "stdio.h"
#include "string.h"
#include "w25qxx.h"
#include "usart.h"

//For file operation functions look at https://elm-chan.org/fsw/ff/00index_e.html
uint8_t arr[200];
uint8_t page_counter = 0;

data_to_arr_t logging_values_u;



void data_logger_init()
{
	W25qxx_Init();
}

void log_datas(int page_adr, uint32_t time, float altitude_filtered, float altitude_normal, float velocity_filtered,
		float pressure, float temperature, float humidity, float accel_x, float accel_y, float accel_z,
		float gyro_x, float gyro_y, float gyro_z, float roll, float pitch, float yaw,
		float q0, float q1, float q2, float q3, float lat, float lon,
		float GNSS_altitude, float GNSS_time, uint32_t status, float voltage, float power, uint32_t sat_count)
{
	logging_values_u.datas.start = (uint32_t)'x';
	logging_values_u.datas.time = time;
	logging_values_u.datas.altitude_filtered = altitude_filtered;
	logging_values_u.datas.altitude_normal = altitude_normal;
	logging_values_u.datas.velocity_filtered = velocity_filtered;
	logging_values_u.datas.pressure = pressure;
	logging_values_u.datas.temperature = temperature;
	logging_values_u.datas.humidity = humidity;
	logging_values_u.datas.accel_x = accel_x;
	logging_values_u.datas.accel_y = accel_y;
	logging_values_u.datas.accel_z = accel_z;
	logging_values_u.datas.gyro_x = gyro_x;
	logging_values_u.datas.gyro_y = gyro_y;
	logging_values_u.datas.roll = roll;
	logging_values_u.datas.pitch = pitch;
	logging_values_u.datas.yaw = yaw;
	logging_values_u.datas.q0 = q0;
	logging_values_u.datas.q1 = q1;
	logging_values_u.datas.q2 = q2;
	logging_values_u.datas.q3 = q3;
	logging_values_u.datas.lat = lat;
	logging_values_u.datas.lon = lon;
	logging_values_u.datas.GNSS_alittude = GNSS_altitude;
	logging_values_u.datas.GNSS_time = GNSS_time;
	logging_values_u.datas.status = status;
	logging_values_u.datas.voltage = voltage;
	logging_values_u.datas.power = power;
	logging_values_u.datas.sat_count = sat_count;
	logging_values_u.datas.stop = (uint32_t)'\r';

	W25qxx_WritePage(logging_values_u.arr, page_adr, 0, sizeof(logging_values_u.arr));
}

void read_logged_datas(int page_adr)
{
	data_to_arr_t readed_vals;
	W25qxx_ReadPage(readed_vals.arr, page_adr, 0, sizeof(logging_values_u.arr));
	sprintf((char*)arr, "%f,%f,%f\r\n", readed_vals.datas.accel_x, readed_vals.datas.altitude_filtered, readed_vals.datas.pressure);
	HAL_UART_Transmit(&TTL_HNDLR, arr, strlen((char*)arr), 80);
}


















