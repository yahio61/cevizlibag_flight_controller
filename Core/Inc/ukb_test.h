/*
 * ukb_test.h
 *
 *  Created on: Aug 3, 2025
 *      Author: yahya
 */

#ifndef INC_UKB_TEST_H_
#define INC_UKB_TEST_H_

#include "main.h"
//#define LITTLE_ENDIAN_

#ifdef LITTLE_ENDIAN_
  #define arr_0 0
  #define arr_1 1
  #define arr_2 2
  #define arr_3 3
#else
  #define arr_0 3
  #define arr_1 2
  #define arr_2 1
  #define arr_3 0
#endif

extern UART_HandleTypeDef TTL_HNDLR;
extern UART_HandleTypeDef RS232_HNDLR;
typedef struct UKB_test
{
  float altitude;   //m
  float pressure;   //mBar
  float accel_x;    //m/s^2
  float accel_y;    //m/s^2
  float accel_z;    //m/s^2
  float angle_x;    //degree
  float angle_y;    //degree
  float angle_z;    //degree
}UKB_test_t;



union float_to_UINT8_converter
{
  float num;
  uint8_t array[4];
};

void ukb_test_init(UKB_test_t *UKB_datas);
uint8_t calc_checksum(uint8_t *packed_datas, uint16_t len);
void process_received_datas(uint8_t *data);
int unpack_datas_for_test(uint8_t *packed_datas, UKB_test_t *ukb_s);
void pack_datas_for_test(uint8_t *packed_datas, UKB_test_t *ukb_s);
working_mode_e get_test_mode();
void ukb_test_stat_update(flight_states_e status);

#endif /* INC_UKB_TEST_H_ */
