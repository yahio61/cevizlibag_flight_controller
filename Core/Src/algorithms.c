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

extern int is_BME_ok;

uint32_t currentTime_1 = 0;
uint32_t currentTime_2 = 0;
uint32_t lastTime_1 = 0;
uint32_t lastTime_2 = 0;
float lastAltitude_1 = 0.0;
float lastAltitude_2 = 0.0;

/*
static int is_quaternion_zeroed = 0;
static int TD_counter = 0;
static int secondP_counter = 0;
*/

uint32_t algorithm_1_start_time_u32 = 0;
uint32_t algorithm_2_start_time_u32 = 0;

flight_states_e rocket_status = STAT_ROCKET_READY;

uint32_t counter = 0;
uint8_t	is_updated = 0;

uint8_t buffer_alg[100];

/*
static double sqr(double nmbr)
{
	return pow(nmbr, 2);
}
*/

void reset_algorithm_status()
{
	rocket_status = STAT_ROCKET_READY;

}

/*
 * it works only with BME280 pressure sensor. Measures the vertical velocity.
 * it detects the first deploy
 * it detecets the second deploy via altitude
 */
void algorithm_1_update(flight_data_t *rocket)
{

	float velocity = 0.0;

	//velocity measuiring
	currentTime_1 = HAL_GetTick();
	uint32_t delta_time = currentTime_1 - lastTime_1;

  if(delta_time >= 90)
  {
	  float currentAltitude = rocket->altitude;
	  velocity = (currentAltitude - lastAltitude_1) / ((float)(delta_time) / 1000);
	  lastAltitude_1 = currentAltitude;

	  lastTime_1 = currentTime_1;
	  is_updated = 1;
  }

  if(is_updated)
  {
	  is_updated = 0;

	switch(rocket_status)
	{
		case STAT_ROCKET_READY:						//rising detection
			if(velocity > RISING_VELOCITY_TRESHOLD || rocket->accel_Y > RISING_G_TRESHOLD)
			{
				counter++;
			}
			else
			{
				counter = 0;
			}

			if(counter == 2)
			{
				rocket_status = STAT_FLIGHT_STARTED;
				counter = 0;
				//serial_println("Fligth started", &TTL_HNDLR);

			}
			break;

		case STAT_FLIGHT_STARTED:					//Burnout detect
			if(rocket->accel_Y < BURNOUT_THRESHOLD)
			{
				counter++;
			}
			else
			{
				counter = 0;
			}

			if(counter == 1)
			{
				rocket_status = STAT_MOTOR_BURNOUT;
				counter = 0;
				//serial_println("Burnout", &TTL_HNDLR);
			}
			break;

		case STAT_MOTOR_BURNOUT:					//Arming altitude achived
			if(rocket->altitude > ARMING_ALTITUDE)
			{
				counter++;
			}
			else
			{
				counter = 0;
			}

			if(counter == 3)
			{
				rocket_status = STAT_ARMING_DISABLE;
				counter = 0;
				//serial_println("Arming altitude achived", &TTL_HNDLR);
			}
			break;

		case STAT_ARMING_DISABLE:					//Falling detection || First parachute
			if(velocity < FALLING_VELOCITY_TRESHOLD || rocket->angle_X > ANGLE_THRESHOLD || rocket->angle_Y > ANGLE_THRESHOLD || rocket->angle_Z > ANGLE_THRESHOLD)
			{
				counter++;
			}
			else
			{
				counter = 0;
			}

			if(counter == 3)
			{
				rocket_status = STAT_P1_OK_P2_NO;
				counter = 0;
				//serial_println("First parachute_deploy", &TTL_HNDLR);
				apoge_deploy();

			}
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
				rocket_status = STAT_P1_OK_P2_OK;
				counter = 0;
				//serial_println("Second parachute deploy", &TTL_HNDLR);
				main_deploy();

			}
			break;

		case STAT_P1_OK_P2_OK:

			break;

		case STAT_TOUCH_DOWN:

			break;

		default:
			break;

	}

  }
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
