/*
 * ukb_test.c
 *
 *  Created on: Aug 3, 2025
 *      Author: yahya
 */
#include "ukb_test.h"

working_mode_e mode = MODE_NORMAL;

uint8_t sit_start[] = {0xAA,0x20,0xCA,0x0D,0x0A};
uint8_t sut_start[] = {0xAA,0x22,0xCC,0x0D,0x0A};
uint8_t test_stop[] = {0xAA,0x24,0xCE,0x0D,0x0A};

UKB_test_t *ukb_s;
uint16_to_uint8_u status_data;

uint8_t packet[37] = {0};
extern uint8_t	is_new_test_data;
/*
  ukb_test_s.pressure =
  ukb_test_s.altitude =
  ukb_test_s.accel_X  =
  ukb_test_s.accel_Y  =
  ukb_test_s.accel_Z  =
  ukb_test_s.angle_X  =
  ukb_test_s.angle_Y  =
  ukb_test_s.angle_Z  =
 */
void ukb_test_init(UKB_test_t *UKB_datas)
{
	ukb_s = UKB_datas;
	status_data.data16 = 0;
}

void process_received_datas(uint8_t *data)
{

	memcpy(packet, data, 36);
	packet[36] = '\0';

	if(mode == MODE_NORMAL)
	{
		if(!memcmp(packet, sut_start, 5))
		{
			serial_println("sut start", &TTL_HNDLR);
			mode = MODE_SUT_TEST;
		}
		else if(!memcmp(packet, sit_start, 5))
		{
			serial_println("sit start", &TTL_HNDLR);
			mode = MODE_SIT_TEST;
		}
	}
	else
	{
		if(!memcmp(packet, test_stop, 5))
		{
			serial_println("test stop", &TTL_HNDLR);
			mode = MODE_NORMAL;
			reset_algorithm_status();
			status_data.data16 = 0;
		}
		else
		{
			if(!unpack_datas_for_test(packet, ukb_s))
			{

			}

		}
	}
}

int unpack_datas_for_test(uint8_t *packed_datas, UKB_test_t *ukb_s)
{
  union float_to_UINT8_converter var;
  if(packed_datas[0] == 0xAB && packed_datas[34] == 0x0D && packed_datas[35] == 0x0A && calc_checksum(packed_datas, 33) == packed_datas[33])
  {
    var.array[arr_0] = packed_datas[1];
    var.array[arr_1] = packed_datas[2];
    var.array[arr_2] = packed_datas[3];
    var.array[arr_3] = packed_datas[4];
    ukb_s->altitude = var.num;

    var.array[arr_0] = packed_datas[5];
    var.array[arr_1] = packed_datas[6];
    var.array[arr_2] = packed_datas[7];
    var.array[arr_3] = packed_datas[8];
    ukb_s->pressure = var.num;

    var.array[arr_0] = packed_datas[9];
    var.array[arr_1] = packed_datas[10];
    var.array[arr_2] = packed_datas[11];
    var.array[arr_3] = packed_datas[12];
    ukb_s->accel_x = var.num;
    ukb_s->accel_x *= TO_MG;

    var.array[arr_0] = packed_datas[13];
    var.array[arr_1] = packed_datas[14];
    var.array[arr_2] = packed_datas[15];
    var.array[arr_3] = packed_datas[16];
    ukb_s->accel_y = var.num;
    ukb_s->accel_y *= TO_MG;

    var.array[arr_0] = packed_datas[17];
    var.array[arr_1] = packed_datas[18];
    var.array[arr_2] = packed_datas[19];
    var.array[arr_3] = packed_datas[20];
    ukb_s->accel_z = var.num;
    ukb_s->accel_z *= TO_MG;

    var.array[arr_0] = packed_datas[21];
    var.array[arr_1] = packed_datas[22];
    var.array[arr_2] = packed_datas[23];
    var.array[arr_3] = packed_datas[24];
    ukb_s->angle_x = var.num;

    var.array[arr_0] = packed_datas[25];
    var.array[arr_1] = packed_datas[26];
    var.array[arr_2] = packed_datas[27];
    var.array[arr_3] = packed_datas[28];
    ukb_s->angle_y = var.num;

    var.array[arr_0] = packed_datas[29];
    var.array[arr_1] = packed_datas[30];
    var.array[arr_2] = packed_datas[31];
    var.array[arr_3] = packed_datas[32];
    ukb_s->angle_z = var.num;

    return 0;
  }
  return 1;
}

void pack_datas_for_test(uint8_t *packed_datas, UKB_test_t *ukb_s)
{
  union float_to_UINT8_converter var;
  packed_datas[0] = 0xAB;

  var.num = ukb_s->altitude;
  packed_datas[1] = var.array[arr_0];
  packed_datas[2] = var.array[arr_1];
  packed_datas[3] = var.array[arr_2];
  packed_datas[4] = var.array[arr_3];

  var.num = ukb_s->pressure;
  packed_datas[5] = var.array[arr_0];
  packed_datas[6] = var.array[arr_1];
  packed_datas[7] = var.array[arr_2];
  packed_datas[8] = var.array[arr_3];

  var.num = ukb_s->accel_x;
  packed_datas[9] = var.array[arr_0];
  packed_datas[10] = var.array[arr_1];
  packed_datas[11] = var.array[arr_2];
  packed_datas[12] = var.array[arr_3];

  var.num = ukb_s->accel_y;
  packed_datas[13] = var.array[arr_0];
  packed_datas[14] = var.array[arr_1];
  packed_datas[15] = var.array[arr_2];
  packed_datas[16] = var.array[arr_3];

  var.num = ukb_s->accel_z;
  packed_datas[17] = var.array[arr_0];
  packed_datas[18] = var.array[arr_1];
  packed_datas[19] = var.array[arr_2];
  packed_datas[20] = var.array[arr_3];

  var.num = ukb_s->angle_x;
  packed_datas[21] = var.array[arr_0];
  packed_datas[22] = var.array[arr_1];
  packed_datas[23] = var.array[arr_2];
  packed_datas[24] = var.array[arr_3];

  var.num = ukb_s->angle_y;
  packed_datas[25] = var.array[arr_0];
  packed_datas[26] = var.array[arr_1];
  packed_datas[27] = var.array[arr_2];
  packed_datas[28] = var.array[arr_3];

  var.num = ukb_s->angle_z;
  packed_datas[29] = var.array[arr_0];
  packed_datas[30] = var.array[arr_1];
  packed_datas[31] = var.array[arr_2];
  packed_datas[32] = var.array[arr_3];

  packed_datas[33] = calc_checksum(packed_datas, 33);

  packed_datas[34] = 0x0D;
  packed_datas[35] = 0x0A;
}

working_mode_e get_test_mode()
{
	return mode;
}
/* 	STAT_ROCKET_READY	=	(uint8_t)0x00,
	STAT_FLIGHT_STARTED	=	(uint8_t)0x01,
	STAT_MOTOR_BURNOUT	=	(uint8_t)0x02,
	STAT_ARMING_DISABLE =	(uint8_t)0x03,
	STAT_ANGLE_HORIZ	=	(uint8_t)0x04,
	STAT_ALT_DECREASE	=	(uint8_t)0x05,
	STAT_P1_OK_P2_NO	=	(uint8_t)0x06,
	STAT_P1_OK_P2_OK	=	(uint8_t)0x07,
	STAT_TOUCH_DOWN		=	(uint8_t)0x08,
	STAT_P1_NO_P2_OK	=	(uint8_t)0x09,
 */

void ukb_test_stat_update(flight_states_e status)
{

	status_data.data16 = ((1 << (status)) - 1) | status;
	uint8_t data[7];

	data[0] = 0xaa;
	data[1] = status_data.data8[0];
	data[2] = status_data.data8[1];
	data[3] = calc_checksum(data, 3);
	data[4] = 0x0d;
	data[5] = 0x0a;

	HAL_UART_Transmit(&RS232_HNDLR, data, 6, 30);
}
uint8_t calc_checksum(uint8_t *packed_datas, uint16_t len)
{
  uint32_t sum = 0;
  for(int i = 0; i < len; i++)
  {
    sum += packed_datas[i];
  }
  return (uint8_t)(sum % 256);
}


