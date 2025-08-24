/*
 * configuration.h
 *
 *  Created on: Aug 19, 2024
 *      Author: yahya
 */

#ifndef INC_CONFIGURATION_H_
#define INC_CONFIGURATION_H_
#include <stdint.h>

#define TO_MG 	(float)101.94
#define TO_SI	(float)0.00981

//************************************   Card Choice   ************************************
//Comment if payload is being coded.
#define ROCKET_CARD
//#define ROCKET_IGNITER_TEST

//************************************   Frequency Choice   ************************************
//freq = freq_val + 410.125
#define ROCKET_TELEM_FREQ			25;					//435.125 MHz
#define PAYLOAD_TELEM_FREQ			30;					//440.125 MHz

//************************************   Algorithms Choices   ************************************
//Uncomment if the algorithm used.
#define ALGORITHM_1							//Only pressure sensor (vertical velocity, altitude)
#define ALGORITHM_2							//Gyro, accelerometer, pressure sensor.

//************************************   Algorithms Thresholds   ************************************
#define SECOND_DEPLOY_ALTITUDE 		(float)2100		//meters		570.0

#define ARMING_ALTITUDE				(float)2000.0		//m				1000
#define RISING_VELOCITY_TRESHOLD	(float)10.0			//m/sn			30.0
#define ALGORITHM_1_LOCKOUT_TIME	(uint32_t)13000		//ms			13000
#define FALLING_VELOCITY_TRESHOLD	(float)3.0			//m/sn			3.0

//#define ARMING_ALTITUDE_2			(float)-10.0		//m				1000
#define QUATERNION_ZERO_TIME		(uint32_t)12000		//ms			12000
#define ALGORITHM_2_LOCKOUT_TIME	(uint32_t)13000		//ms			13000
#define RISING_G_TRESHOLD 			(float)1500.0		//mG			3000.0
#define BURNOUT_THRESHOLD			(float)-500			//mG			-2000.0
#define ANGLE_THRESHOLD				(float)65			//degree		80.0

#define IGNITER_TIME				(uint32_t)1000		//ms			100
//#define Q_SET_ZERO_ACTIVATE
//************************************   Battery Settings   ************************************
#define LOW_BAT						(float)7.0			// Low battery threshold (V)
#define VOLT_COEF					(float)0.0032226563 // Voltage coefficient (V)
#define CRNT_COEF					(float)0.0008056641	// Current coefficient (A)
//************************************   Print Settings   ************************************
//Uncomment if print datas decoded.
#define PRINT_DECODED


typedef enum flight_states{
	STAT_ROCKET_READY	=	(uint8_t)0x00,
	STAT_FLIGHT_STARTED	=	(uint8_t)0x01,
	STAT_MOTOR_BURNOUT	=	(uint8_t)0x02,
	STAT_ARMING_PASSED 	=	(uint8_t)0x03,
	STAT_ANGLE_HORIZ	=	(uint8_t)0x04,
	STAT_ALT_DECREASE	=	(uint8_t)0x05,
	STAT_P1_OK_P2_NO	=	(uint8_t)0x06,
	STAT_SECOND_ALT		=	(uint8_t)0x07,
	STAT_P1_OK_P2_OK	=	(uint8_t)0x08,
	STAT_TOUCH_DOWN		=	(uint8_t)0x09,
}flight_states_e;

typedef struct flight_data
{
  float altitude;   	// Altitude by ground level. (m)
  float alt_sea_level;	// Altitude by sea level. (m)
  float velocity;		// This is value is vertical velocity measured by integral of pressure sensor. (m/s)
  float velocity_est;	// This is value is vertical velocity estimation measured by double integral of accelerometer sensor. (m/s)
  float accel_x;    	// (m/s^2)
  float accel_y;    	// (m/s^2)
  float accel_z;    	// (m/s^2)
  float angle_x;    	// (degree)
  float angle_y;    	// (degree)
  float angle_z;    	// (degree)
  float abs_angle;		//  Absolute angle of rocket by the world surface vector. (degree)
}flight_data_t;

typedef union uint16_to_uint8
{
	uint8_t 	data8[2];
	uint16_t 	data16;
}uint16_to_uint8_u;

typedef struct power
{
	float voltage;
	float wattage;
	float wattage_calced;
	uint32_t last_time;
}power_t;

#endif /* INC_CONFIGURATION_H_ */
