/*
 * algorithms.c
 *
 *  Created on: May 10, 2024
 *      Author: yahya
 */
#include "main.h"
#include "algorithms.h"
#include "ukb_test.h"
//#include "queternion.h"

#define DEBUG_ALGORITHM

extern int is_BME_ok;

uint32_t currentTime_1 = 0;
uint32_t currentTime_2 = 0;
uint32_t lastTime_1 = 0;
uint32_t lastTime_2 = 0;
static float last_altitude = 0.0;
float lastAltitude_2 = 0.0;

/*
static int is_quaternion_zeroed = 0;
static int TD_counter = 0;
static int secondP_counter = 0;
*/

uint32_t algorithm_1_start_time_u32 = 0;
uint32_t algorithm_2_start_time_u32 = 0;

static flight_states_e rocket_status = STAT_ROCKET_READY;

uint32_t counter = 0;
uint32_t counter_2 = 0;

extern uint8_t	is_new_test_data;
uint8_t buffer_alg[100];

/*
static double sqr(double nmbr)
{
	return pow(nmbr, 2);
}
*/

static float resultant_accel(float accel_x, float accel_y, float accel_z)
{
	return (fabs(accel_z) / (accel_z)) * sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
}

void reset_algorithm_status()
{
	rocket_status = STAT_ROCKET_READY;
}

/*
 * @brief	This function works pressure and IMU sensor. Uses the vertical velocity,
 * 			angles the altitude. Detects the first deploy with angle or velocity.
 * 			Detecets the second deploy via altitude only.
 *
 * @param	(flight_data_t) rocket: Struct which includes rocket attitude.
 * 			(working_mode_e) mode: Rocket's working mode. SIT_TEST, SUT_TEST or NORMAL.
 *
 * @retval none
 */
flight_states_e algorithm_update(flight_data_t *rocket, uint32_t mode)
{

	currentTime_1 = HAL_GetTick();
	uint32_t delta_time = currentTime_1 - lastTime_1;
	float resultanted_accel = resultant_accel(rocket->accel_x, rocket->accel_y, rocket->accel_z);
/*
	if(delta_time >= 90)
  {
	  float currentAltitude = rocket->altitude;
	  velocity = (currentAltitude - lastAltitude_1) / ((float)(delta_time) / 1000);
	  lastAltitude_1 = currentAltitude;

	  lastTime_1 = currentTime_1;
	  is_updated = 1;
  }

*/
	if(mode == MODE_SUT_TEST)
	{
		if(!is_new_test_data)
		{
			return rocket_status;
		}
		is_new_test_data = 0;
		rocket->abs_angle = (rocket->angle_x > rocket->angle_y) ? rocket->angle_x : rocket->angle_y;
		rocket->velocity = (rocket->altitude - last_altitude) / (float)delta_time * 1000.0;
		last_altitude = rocket->altitude;
		lastTime_1 = currentTime_1;
		char str[200];
		//sprintf(str, "delt time = %d", delta_time);
		//sprintf((char*)str,"velocity= %f  test:alt = %f, acx=%f  acy=%f  acz=%f angx=%f angy=%f angz=%f", rocket->velocity, rocket->altitude,rocket->accel_x,  rocket->accel_y, rocket->accel_z, rocket->angle_x, rocket->angle_y, rocket->angle_z);
		//sprintf(str, "resultanted accel = %f", resultanted_accel);
		//serial_println((char*)str, &TTL_HNDLR);
	}

	switch(rocket_status)
	{
		case STAT_ROCKET_READY:						//rising detection
			if(rocket->velocity > RISING_VELOCITY_TRESHOLD || resultanted_accel > RISING_G_TRESHOLD)
			{
				counter++;
			}
			else
			{
				counter = 0;
			}

			if(counter == 3)
			{
				rocket_status = STAT_FLIGHT_STARTED;
				counter = 0;
			#ifdef DEBUG_ALGORITHM
				serial_println("Fligth started", &TTL_HNDLR);
			#endif
			}
			break;

		case STAT_FLIGHT_STARTED:					//Burnout detect
			if(resultanted_accel < BURNOUT_THRESHOLD)
			{
				counter++;
			}
			else
			{
				counter = 0;
			}

			if(counter == 3)
			{
				rocket_status = STAT_MOTOR_BURNOUT;
				counter = 0;
			#ifdef DEBUG_ALGORITHM
				serial_println("Burnout detected", &TTL_HNDLR);
			#endif
			}
			break;

		case STAT_MOTOR_BURNOUT:
			if(rocket->altitude > ARMING_ALTITUDE)	//Arming altitude reached
			{
				counter++;
			}
			else
			{
				counter = 0;
			}

			if(counter == 3)
			{
				rocket_status = STAT_ARMING_PASSED;
				counter = 0;
			#ifdef DEBUG_ALGORITHM
				serial_println("Arming altitude reached", &TTL_HNDLR);
			#endif
			}
			break;

		case STAT_ARMING_PASSED:
			if(rocket->velocity < FALLING_VELOCITY_TRESHOLD) //Falling detection via pressure sensor.
			{
				counter++;
			}
			else if(rocket->abs_angle > ANGLE_THRESHOLD)	//Falling detection via IMU sensor.
			{
				counter_2++;
			}
			else
			{
				counter = 0;
				counter_2 = 0;
			}

			if(counter == 3)
			{
				rocket_status = STAT_ALT_DECREASE;
				counter = 0;
			#ifdef DEBUG_ALGORITHM
				serial_println("Altitude decreasing", &TTL_HNDLR);
			#endif
			}
			else if(counter_2 == 3)
			{
				rocket_status = STAT_ANGLE_HORIZ;
				counter_2 = 0;
			#ifdef DEBUG_ALGORITHM
				serial_println("Angle reached the threshold", &TTL_HNDLR);
			#endif
			}
			break;

		case STAT_ANGLE_HORIZ:
		case STAT_ALT_DECREASE:
			apoge_deploy();
			rocket_status = STAT_P1_OK_P2_NO;
			#ifdef DEBUG_ALGORITHM
				serial_println("Apogee parachute deploy", &TTL_HNDLR);
			#endif
			break;

		case STAT_P1_OK_P2_NO:							//Second parachute deploy
			if(rocket->altitude < SECOND_DEPLOY_ALTITUDE)
			{
				counter++;
			}
			else
			{
				counter = 0;
			}

			if(counter == 3)
			{
				rocket_status = STAT_SECOND_ALT;
				counter = 0;
				#ifdef DEBUG_ALGORITHM
					serial_println("Second altitude reached", &TTL_HNDLR);
				#endif
			}
			break;

		case STAT_SECOND_ALT:
			main_deploy();
			rocket_status = STAT_P1_OK_P2_OK;
			#ifdef DEBUG_ALGORITHM
				serial_println("Second parachute deployed", &TTL_HNDLR);
			#endif
			break;

		case STAT_P1_OK_P2_OK:
			#ifdef DEBUG_ALGORITHM
				serial_println("Everything is fine", &TTL_HNDLR);
			#endif
			break;

		case STAT_TOUCH_DOWN:
			#ifdef DEBUG_ALGORITHM
				serial_println("Touchdown confirmed", &TTL_HNDLR);
			#endif
			break;

		default:
			break;

	}
	return rocket_status;
}


/*
void algorithm_2_update(BME_280_t* BME, bmi088_struct_t* BMI)
{
	//Rising detection
	if((sqrtf(sqr(BMI->acc_x) + sqr(BMI->acc_y) + sqr(BMI->acc_z)) > RISING_G_TRESHOLD) && isRising_2 == 0)
	{
		if(BME->altitude < 200.0 && BME->altitude > -200.0){
			saved_datas->base_altitude = BME->altitude + saved_datas->base_altitude;
		}

		isRising_2 = 1;
		algorithm_2_start_time_u32 = HAL_GetTick();
		saved_datas->r_status = saved_datas->r_status < STAT_FLIGHT_STARTED ? STAT_FLIGHT_STARTED : saved_datas->r_status;
		ext_pin_open(&buzzer);
		sd_csv_log_transmit("Rising via Accel");
	}

	//Burnout detection
	static int burnout_counter = 0;
	if(BMI->acc_y < BURNOUT_THRESHOLD && isRising_2 == 1 && burnout_counter < 12)
	{
		burnout_counter++;
	}
	if(burnout_counter == 10)
	{
		saved_datas->r_status = saved_datas->r_status < STAT_MOTOR_BURNOUT ? STAT_MOTOR_BURNOUT : saved_datas->r_status;
		ext_pin_open(&buzzer);
		sd_csv_log_transmit("Burnout");
	}

#ifdef Q_SET_ZERO_ACTIVATE
	//quaternion setting to zero
	if((HAL_GetTick() - algorithm_2_start_time_u32) > QUATERNION_ZERO_TIME && is_quaternion_zeroed == 0 && isRising_2 == 1)
	{
	  quaternionSet_zero();
	  is_quaternion_zeroed = 1;
	  ext_pin_open(&buzzer);
	  sd_csv_log_transmit("Quaternion set zero");
	}
#endif

	//Falling detection || First parachute
	if(BMI->angle > ANGLE_THRESHOLD && isRising_2 == 1 && isFalling_2 == 0 && HAL_GetTick() - algorithm_2_start_time_u32 > ALGORITHM_2_LOCKOUT_TIME)
	{
		isFalling_2 = 1;
		//saved_datas->r_status = saved_datas->r_status < STAT_P1_OK_P2_NO ? STAT_P1_OK_P2_NO : saved_datas->r_status;
		//deploy_p_1();
		sd_csv_log_transmit("P_1 via Gyro");
	}

	if(is_BME_ok == 1)
	{
		//Second Parachute
		if(BME->altitude < SECOND_DEPLOY_ALTITUDE && isFalling_2 == 1 && is_secondP_OK == 0)
		{
			secondP_counter++;
		}
		else{
			secondP_counter = 0;
		}
		if(secondP_counter == 10)
		{
			//saved_datas->r_status = saved_datas->r_status < STAT_P1_OK_P2_OK ? STAT_P1_OK_P2_OK : saved_datas->r_status;
			is_secondP_OK = 1;
			//deploy_p_2();
			sd_csv_log_transmit("P_2 via pressure algorithm_2");
		}
	}
	//Touchdown Detection
	static uint8_t is_TD = 0;
	if(sqrt(sqr(BMI->gyro_x) + sqr(BMI->gyro_y) + sqr(BMI->gyro_z)) < 10.0 && isFalling_2 == 1 && is_secondP_OK == 1 && is_TD == 0)
	{
			TD_counter++;
	}
	else{
		TD_counter = 0;
	}
	if(TD_counter > 1000)
	{
		is_TD = 1;
		saved_datas->r_status = saved_datas->r_status < STAT_TOUCH_DOWN ? STAT_TOUCH_DOWN : saved_datas->r_status;
		ext_pin_open(&buzzer);
		sd_csv_log_transmit("TD");
	}
}
*/
