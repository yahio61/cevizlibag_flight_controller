/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bme280.h"
#include "bmi088.h"
#include "ukb_test.h"
#include "algorithms.h"
#include "queternion.h"
#include "dataPacking.h"
#include "usr_gnss_l86_parser.h"
#include "e22_lib.h"
#include "dwt_profiler.h"
#include "data_logger.h"
#include "w25qxx.h"
#include "filters.h"
#include "z_flash_W25QXXX.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct bckp_sram_datas
{
	bme_parameters_t	bme_params;
	bmi088_offsets_t	bmi_offsets;
	float q[4];
	uint32_t logger_counter;
}bckp_sram_datas_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
bckp_sram_datas_t* const backup_datas = (bckp_sram_datas_t*) BKPSRAM_BASE;
static bme280_struct_t 	bme_sensor_s;
static bmi088_struct_t	bmi_imu_s;
static e22_conf_struct_t lora_1;
static flight_states_e rocket_status;

working_mode_e working_mode = MODE_NORMAL;
power_t power_s;
UKB_test_t test_datas;
flight_data_t rocket_flight_datas;
S_GPS_L86_DATA gps_s;
BaroAccelFilter filter_1;

extern float euler[3];
extern uint8_t dma_rx_buf[RX_BUFFER_LEN + 10];
extern uint8_t *gps_buf;
uint8_t datas_packed[36];

uint32_t main_mos_counter = 0;
uint32_t apoge_mos_counter = 0;
uint32_t last_vel_measure_time = 0;

uint8_t *packed_datas_p;
uint8_t str[200];
uint8_t is_1000ms = 0;
uint8_t is_200ms = 0;
uint8_t is_100ms = 0;
uint8_t is_10ms = 0;
uint8_t is_1ms = 0;
uint8_t is_telem_timer_ok = 0;
uint8_t is_power_1s = 0;
uint8_t last_mode = MODE_NORMAL;
uint8_t is_dma_idle = 0;
uint8_t	is_new_test_data = 0;
uint8_t is_zeroed = 0;
uint8_t beep_counter = 0;

int counter = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t bme280_begin(void);
uint8_t bmi088_begin(void);
void bmi_callback(bmi088_struct_t *BMI);
void lora_init(void);
void calc_power(power_t* pow);
void reset_fligth_datas();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_RTC_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM9_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM7_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */

#ifdef CALC_TIME
  dwt_profiler_init();
#endif

  HAL_PWR_EnableBkUpAccess();
  RCC->AHB1ENR |= RCC_AHB1ENR_BKPSRAMEN;
  HAL_PWR_EnableBkUpReg();

  HAL_GPIO_WritePin(SENS_RES_GPIO_Port, SENS_RES_Pin, GPIO_PIN_SET);
  HAL_Delay(50);
  HAL_GPIO_WritePin(SENS_RES_GPIO_Port, SENS_RES_Pin, GPIO_PIN_RESET);
  HAL_Delay(50);

  for(int i = 0; i < 16; i++)
  {
	  HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
	  HAL_Delay(50);
  }

  uint8_t bme_ret = bme280_begin();
  if(bme_ret)
  {
	  serial_println("bme sensor fail", &TTL_HNDLR);
  }
  else
  {
	  //serial_println("bme sensor success", &TTL_HNDLR);
  }

  bme280_config(&bme_sensor_s);

  uint8_t bmi_ret = bmi088_begin();
  if(bmi_ret)
  {
	  serial_println("bmi fail", &TTL_HNDLR);
      if((bmi_ret & 0x01) == 0x01)
      {
        sprintf((char*)str, "error in accel");
        serial_println((char*) str, &TTL_HNDLR);
      }
      if((bmi_ret & 0x02) == 0x02)
      {
        sprintf((char*)str, "error in gyro");
        serial_println((char*) str, &TTL_HNDLR);
      }
  }
  else
  {
	  //serial_println("bmi success", &TTL_HNDLR);
  }

    bmi088_config(&bmi_imu_s);
    get_offset(&bmi_imu_s);

    ukb_test_init(&test_datas);
    quaternon_init(backup_datas->q);
    bmi088_update(&bmi_imu_s);
    getInitialQuaternion(bmi_imu_s.datas.acc_x, bmi_imu_s.datas.acc_y, bmi_imu_s.datas.acc_z);

    // GNSS config baud rate 57600 with PMTK command.
    serial_println("$PMTK251,57600*2C", &GPS_UART_HNDLR);	// GNSS baud set 57600
    HAL_UART_DeInit(&GPS_UART_HNDLR);
    GPS_UART_HNDLR.Init.BaudRate = 57600;
    HAL_UART_Init(&GPS_UART_HNDLR);
    UsrGpsL86Init(&GPS_UART_HNDLR);
    //VIEW_GPS()											// Read and write to TTL raw GNSS raw value.

    // Filter config.
    //baf_init(&filter_1, -bmi_imu_s.datas.acc_y, 0.3, 0.1, (float)bme_sensor_s.datas.time_of_update);
    baf_init(&filter_1, -bmi_imu_s.datas.acc_y, 0.3, 0.2, (float)HAL_GetTick()-1);

    // Lora module config.
    lora_init();
    data_logger_init();

#ifdef ERASE_FLASH_CHIP
    serial_println("silmeye baslandi", &TTL_HNDLR);
    //W25qxx_EraseChip();
    Flash_ChipErase();
    serial_println("chip silindi", &TTL_HNDLR);

#endif




    // Config phase finished beep.
    beep(1000);

    // Start timer interrupts.
    HAL_TIM_Base_Start_IT(&htim3);
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim7);
    HAL_TIM_Base_Start_IT(&htim9);
    HAL_TIM_Base_Start_IT(&htim8);

    // Start receiving DMA form RS232 UART.
    __HAL_UART_ENABLE_IT(&RS232_HNDLR, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&RS232_HNDLR, dma_rx_buf, RX_BUFFER_LEN + 6);
    //TIM9->ARR = 10000;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#ifdef CALC_TIME
	  PROFILE_START(PROF_MAIN_LOOP);
#endif

	  if(get_test_mode() != last_mode)
	  {
		  working_mode = get_test_mode();
		  last_mode = get_test_mode();
		  reset_algorithm_status(&rocket_flight_datas);
		  rocket_status = STAT_ROCKET_READY;
		  reset_test_datas();
		  reset_fligth_datas();
		  switch(working_mode)
		  {
		  case MODE_NORMAL:
			  baf_init(&filter_1, -bmi_imu_s.datas.acc_y, 0.3, 0.2, (float)HAL_GetTick() - 1);
			  break;
		  case MODE_SUT_TEST:
			  baf_init(&filter_1, 0, 0.3, 0.2, (float)HAL_GetTick() - 1);
			  break;
		  case MODE_SIT_TEST:
			  break;
		  }
		  //sprintf((char*)str,"mode = %d", working_mode);
		  //serial_println((char*)str, &TTL_HNDLR);
	  }

	  if(is_1000ms)	// This condition works at 1Hz.
	  {
		  Usr_GpsL86GetValues(&gps_s, &GPS_UART_HNDLR);
		  is_power_1s = 1;
		  is_1000ms = 0;
	  }
	  if(is_200ms)
	  {
		  if(working_mode == MODE_NORMAL)
		  {

		  }

		  is_200ms = 0;
	  }
	  if(is_100ms)	// This condition works at 5Hz.
	  {
		  //sprintf((char*)str,"h=%f \t v=%f \t time=%f \t abia=%f \t beta=%f \t alpha=%f", filter_1.h, filter_1.v, filter_1.dt, filter_1.abias, filter_1.beta, filter_1.alpha);
		  ///printf((char*)str,"h=%f \t v=%f \t time=%f \t abia=%f",  rocket_flight_datas.altitude ,  rocket_flight_datas.velocity, filter_1.dt, filter_1.abias);
		  //sprintf((char*)str, "accel_x=%f \t accel_y=%f \t accelz=%f", bmi_imu_s.datas.acc_x, bmi_imu_s.datas.acc_y, bmi_imu_s.datas.acc_z);
		  //serial_println((char*)str, &TTL_HNDLR);


		  switch(get_test_mode())
		  {

		  case MODE_NORMAL:
			  rocket_flight_datas.velocity		= bme_sensor_s.datas.velocity;
			  rocket_flight_datas.alt_sea_level = bme_sensor_s.datas.height;
			  rocket_flight_datas.altitude 		= bme_sensor_s.datas.altitude;
			  rocket_flight_datas.accel_x 		= bmi_imu_s.datas.acc_x / 1000;
			  rocket_flight_datas.accel_y 		= bmi_imu_s.datas.acc_z / 1000;
			  rocket_flight_datas.accel_z 		= -bmi_imu_s.datas.acc_y / 1000;
			  rocket_flight_datas.angle_x 		= euler[0];
			  rocket_flight_datas.angle_y 		= euler[1];
			  rocket_flight_datas.angle_z 		= euler[2];
			  rocket_flight_datas.abs_angle			= quaternionToTheta();
			  rocket_flight_datas.data_taken_time 	= bme_sensor_s.datas.time_of_update;

			  baf_step(&filter_1, rocket_flight_datas.altitude, rocket_flight_datas.accel_z * 1000, rocket_flight_datas.data_taken_time);
			  rocket_flight_datas.altitude = filter_1.h;
			  rocket_flight_datas.velocity = filter_1.v;
			  bme_sensor_s.datas.velocity = filter_1.v;
			  rocket_flight_datas.is_new_data = 1;
#ifdef CALC_TIME
			  PROFILE_START(PROF_PACKET_SEND);
#endif
			  packed_datas_p = packDatas(&bmi_imu_s, &bme_sensor_s, &gps_s, &power_s, rocket_status, &rocket_flight_datas);
			  //send_datas(&TTL_HNDLR, packed_datas_p, 64);	// Sends the packets via uart bridge to GCS.
			  //log_datas(gps_s.altitudeInMeter, gps_s.lat, gps_s.lon, gps_s.timeDateBuf, bme_sensor_s.datas.altitude, bme_sensor_s.datas.temperature, bme_sensor_s.datas.humidity);
			  //send_datas(&RS232_HNDLR, packed_datas_p, 64);
			  if(rocket_status > STAT_ROCKET_READY && rocket_status < STAT_TOUCH_DOWN && backup_datas->logger_counter < 130000)
			  {
				  log_datas(backup_datas->logger_counter++, HAL_GetTick(), filter_1.h, bme_sensor_s.datas.altitude, filter_1.v, bme_sensor_s.datas.pressure, bme_sensor_s.datas.temperature,
				      			  bme_sensor_s.datas.humidity, bmi_imu_s.datas.acc_x, bmi_imu_s.datas.acc_y, bmi_imu_s.datas.acc_z,
				      			  bmi_imu_s.datas.gyro_x, bmi_imu_s.datas.gyro_y, bmi_imu_s.datas.gyro_z, euler[0], euler[1], euler[2],
				      			  backup_datas->q[0], backup_datas->q[1], backup_datas->q[2], backup_datas->q[3], gps_s.lat, gps_s.lon,
				      			  gps_s.altitudeInMeter, gps_s.timeDateBuf, rocket_status, power_s.voltage, power_s.wattage_calced, gps_s.satInUse);
			  }
#ifdef CALC_TIME
			  PROFILE_END(PROF_PACKET_SEND);
#endif
			  break;

		  case MODE_SIT_TEST:
			  test_datas.altitude	= rocket_flight_datas.alt_sea_level;
			  test_datas.pressure 	= bme_sensor_s.datas.pressure;
			  test_datas.accel_x 	= rocket_flight_datas.accel_x * TO_SI;
			  test_datas.accel_y 	= rocket_flight_datas.accel_z * TO_SI;
			  test_datas.accel_z 	= rocket_flight_datas.accel_y * TO_SI;
			  test_datas.angle_x 	= rocket_flight_datas.angle_x;	//bmi_imu_s.datas.gyro_x_angle;
			  test_datas.angle_y 	= rocket_flight_datas.angle_y;
			  test_datas.angle_z 	= rocket_flight_datas.angle_z; //bmi_imu_s.datas.gyro_z_angle;

			  pack_datas_for_test(datas_packed, &test_datas);
			  HAL_UART_Transmit_DMA(&RS232_HNDLR, datas_packed, 36);
			  HAL_UART_Transmit_DMA(&TTL_HNDLR, datas_packed, 36);
			  break;

		  case MODE_SUT_TEST:

			  break;

		  }
		  is_100ms = 0;
	  }


	  if(is_dma_idle)	// sut test
	  {
		  process_received_datas(dma_rx_buf);
		  HAL_UART_Receive_DMA(&RS232_HNDLR, dma_rx_buf, RX_BUFFER_LEN + 6);

		  if(working_mode == MODE_SUT_TEST)
		  {
			  //rocket_flight_datas.altitude			= test_datas.altitude;
			  rocket_flight_datas.accel_x 			= test_datas.accel_x;
			  rocket_flight_datas.accel_y 			= test_datas.accel_y;
			  rocket_flight_datas.accel_z 			= test_datas.accel_z;
			  rocket_flight_datas.angle_x 			= test_datas.angle_x;
			  rocket_flight_datas.angle_y 			= test_datas.angle_y;
			  rocket_flight_datas.angle_z 			= test_datas.angle_z;
			  rocket_flight_datas.data_taken_time 	= test_datas.data_taken_time;
			  //sprintf((char*)str,">velocity:%f", rocket_flight_datas.velocity);
			  //serial_println((char*)str, &TTL_HNDLR);
			  //sprintf((char*)str,"test:alt = %f, acx=%f  acy=%f  acz=%f angx=%f angy=%f angz=%f", test_datas.altitude, rocket_flight_datas.accel_x,  rocket_flight_datas.accel_y , rocket_flight_datas.accel_z, test_datas.angle_x, test_datas.angle_y, test_datas.angle_z);
			  //serial_println((char*)str, &TTL_HNDLR);

			  baf_step(&filter_1, test_datas.altitude, test_datas.accel_z,  test_datas.data_taken_time);
			  rocket_flight_datas.altitude = filter_1.h;
			  rocket_flight_datas.velocity = filter_1.v;

			  sprintf((char*)str,"%f,%f,%f,%f,\n\r", filter_1.h, filter_1.v, test_datas.altitude, test_datas.accel_z/1000);
			  //sprintf((char*)str,"%f,%f,%f,%f,%f,%f,%f\n\r", test_datas.accel_x, test_datas.accel_y, test_datas.accel_z, test_datas.angle_x, test_datas.angle_y, test_datas.angle_z, test_datas.altitude);
			  //serial_println((char*)str, &TTL_HNDLR);

			  ukb_test_stat_update(rocket_status );
			  rocket_flight_datas.is_new_data = 1;
		  }
		  is_dma_idle = 0;
	  }

	  if(is_10ms)// This condition works at 100Hz.
	  {
#ifdef CALC_TIME
		  PROFILE_START(PROF_BME280_UPDATE);
#endif

		  bme280_update(&bme_sensor_s);	// 100Hz call is enough for baro sensor.

#ifdef CALC_TIME
		  PROFILE_END(PROF_BME280_UPDATE);
#endif

		  if(working_mode == MODE_SIT_TEST)
		  {
			  rocket_flight_datas.velocity		= bme_sensor_s.datas.velocity;
			  rocket_flight_datas.alt_sea_level = bme_sensor_s.datas.height;
			  rocket_flight_datas.altitude 		= bme_sensor_s.datas.altitude;
			  rocket_flight_datas.accel_x 		= bmi_imu_s.datas.acc_x;
			  rocket_flight_datas.accel_y 		= bmi_imu_s.datas.acc_y;
			  rocket_flight_datas.accel_z 		= bmi_imu_s.datas.acc_z;
			  rocket_flight_datas.angle_x 		= euler[0];
			  rocket_flight_datas.angle_y 		= euler[1];
			  rocket_flight_datas.angle_z 		= euler[2];
		  }
		  is_10ms = 0;
	  }
	  if(is_1ms)	// This condition works at 1kHz.
	  {
		  if(rocket_status > STAT_ROCKET_READY && rocket_status < STAT_P1_OK_P2_NO)
		  {
			  TIM9->ARR = 2499;
		  }

		  calc_power(&power_s);
		  if(working_mode == MODE_NORMAL || working_mode == MODE_SUT_TEST)
		  {
			  rocket_status = algorithm_update(&rocket_flight_datas, working_mode);
		  }
		  is_1ms = 0;
	  }
	  if(is_telem_timer_ok)
	  {
		if(power_s.voltage > LOW_BAT)
		{
		  e22_chMode_transmit(&lora_1);
		  send_datas(&TELEM_UART_HNDLR, packed_datas_p, 64);
		}
		else
		{
		  e22_chMode_sleep(&lora_1);
		}
		is_telem_timer_ok = 0;
	  }
#ifdef CALC_TIME
	  PROFILE_START(PROF_BMI088_UPDATE);
#endif

	  bmi088_update(&bmi_imu_s);	// IMU sensor uses interrupts so it can be called always.

#ifdef CALC_TIME
	  PROFILE_END(PROF_BMI088_UPDATE);
#endif

#ifdef CALC_TIME
	  PROFILE_END(PROF_MAIN_LOOP);
#endif

#ifdef CALC_TIME
	  //dwt_profiler_print_results();
	  dwt_profiler_print_compact();
#endif
	 }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void lora_init(void)
{
	lora_1.baud_rate 		= 	E22_BAUD_RATE_115200;
	lora_1.parity_bit		=	E22_PARITY_8N1;
	lora_1.air_rate			=	E22_AIR_DATA_RATE_38400;
	lora_1.packet_size		=	E22_PACKET_SIZE_64;
	lora_1.rssi_noise		=	E22_RSSI_NOISE_DISABLE;
	lora_1.power			=	E22_TRANSMITTING_POWER_22;
	lora_1.rssi_enable		=	E22_ENABLE_RSSI_DISABLE;
	lora_1.mode				= 	E22_TRANSMISSION_MODE_TRANSPARENT;
	lora_1.repeater_func	=	E22_REPEATER_FUNC_DISABLE;
	lora_1.lbt				=	E22_LBT_DISABLE;
	lora_1.wor				=	E22_WOR_RECEIVER;
	lora_1.wor_cycle		=	E22_WOR_CYCLE_1000;
	lora_1.channel			=	ROCKET_TELEM_FREQ;

	lora_1.pins.m0_pin = RF_M0_Pin;
	lora_1.pins.m0_pin_port = RF_M0_GPIO_Port;
	lora_1.pins.m1_pin = RF_M1_Pin;
	lora_1.pins.m1_pin_port = RF_M1_GPIO_Port;

	e22_init(&lora_1, &TELEM_UART_HNDLR);

	HAL_UART_DeInit(&TELEM_UART_HNDLR);
	TELEM_UART_HNDLR.Init.BaudRate = 115200;
	HAL_UART_Init(&TELEM_UART_HNDLR);
}

uint8_t bme280_begin()
{
	bme_sensor_s.device_config.filter = BME280_FILTER_OFF;
	bme_sensor_s.device_config.mode = BME280_MODE_NORMAL;
	bme_sensor_s.device_config.over_sampling = BME280_OS_8;
	bme_sensor_s.device_config.period = BME280_PERIOD_05;
	bme_sensor_s.device_config.BME_I2C = &BAR_I2C_HNDLR;
	bme_sensor_s.parameters = &backup_datas->bme_params; //if no backup data, write NULL
	return bme280_init(&bme_sensor_s);
}

void bmi_callback(bmi088_struct_t *BMI)
{

	if(is_zeroed == 1)
	{
		updateQuaternion(-BMI->datas.gyro_z * M_PI / 180.0, BMI->datas.gyro_x * M_PI / 180.0, -BMI->datas.gyro_y * M_PI / 180.0, BMI->datas.delta_time);
	}
	else
	{
		MahonyAHRSupdateIMU(-BMI->datas.gyro_z * M_PI / 180.0, BMI->datas.gyro_x * M_PI / 180.0, -BMI->datas.gyro_y * M_PI / 180.0, -BMI->datas.acc_z * TO_SI, BMI->datas.acc_x * TO_SI, -BMI->datas.acc_y * TO_SI, BMI->datas.delta_time);
		if(rocket_status > STAT_ROCKET_READY)
		{
			is_zeroed = 1;
			quaternionSet_zero();
		}
	}
		quaternionToEuler();
}

uint8_t bmi088_begin(void)
{
	//Acc config
	bmi_imu_s.device_config.acc_bandwith = ACC_BWP_OSR4;
	bmi_imu_s.device_config.acc_outputDateRate = ACC_ODR_200;
	bmi_imu_s.device_config.acc_powerMode = ACC_PWR_SAVE_ACTIVE;
	bmi_imu_s.device_config.acc_range = ACC_RANGE_24G;

	// Gyro config
	bmi_imu_s.device_config.gyro_bandWidth = GYRO_BW_116;
	bmi_imu_s.device_config.gyro_range = GYRO_RANGE_2000;
	bmi_imu_s.device_config.gyro_powerMode = GYRO_LPM_NORMAL;

	//Device config
	bmi_imu_s.device_config.acc_IRQ = ACC_IRQ;
	bmi_imu_s.device_config.gyro_IRQ = GYRO_IRQ;
	bmi_imu_s.device_config.BMI_I2c = &IMU_I2C_HNDLR;
	bmi_imu_s.device_config.offsets = &backup_datas->bmi_offsets;	//Offset datas stored in backup sram for saving them unwanted reset.
	bmi_imu_s.IMU_callback = bmi_callback;
	return	bmi088_init(&bmi_imu_s);
}

void serial_println(char* str, UART_HandleTypeDef *huart_disp)
{
	HAL_UART_Transmit_DMA(huart_disp, (uint8_t*)str, strlen(str));
	HAL_UART_Transmit_DMA(huart_disp, (uint8_t*)"\r\n", 2);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == INT_ACC_Pin)
	{
		bmi088_set_accel_INT(&bmi_imu_s);
	}
	if(GPIO_Pin == INT_GYRO_Pin)
	{
		bmi088_set_gyro_INT(&bmi_imu_s);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)	// This repeats in every 1000ms
    {
		is_1000ms = 1;
    }

    if (htim->Instance == TIM8)	// This repeats in every 200ms
	{
    	is_200ms = 1;
	}

    if (htim->Instance == TIM4)	// This repeats in every 100ms
    {
    	is_100ms = 1;
    }

    if(htim->Instance == TIM5)	// This repeats in every 10ms
    {
		is_10ms = 1;
    }

    if(htim->Instance == TIM6)	// This repeats in every 1ms
    {
    	is_1ms = 1;
    }

    if(htim->Instance == TIM9)	// This timer's frequency is changed by software for some purposes. Default 1Hz.
    {
    	is_telem_timer_ok = 1;
    }

	if(htim->Instance == TIM7)	// This block is for external pins lie buzzers leds. period = 10ms interrupt.
	{
		if(!(--main_mos_counter))
		{
			HAL_GPIO_WritePin(MAIN_MOS_GPIO_Port, MAIN_MOS_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MAIN_LED_GPIO_Port, MAIN_LED_Pin, GPIO_PIN_SET);
		}
		if(!(--apoge_mos_counter))
		{
			HAL_GPIO_WritePin(APOGE_MOS_GPIO_Port, APOGE_MOS_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(APOGEE_LED_GPIO_Port, APOGEE_LED_Pin, GPIO_PIN_SET);
		}
		if(!(--beep_counter))
		{
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
		}
	}
}

void main_deploy(int time_ms)
{
	HAL_GPIO_WritePin(MAIN_MOS_GPIO_Port, MAIN_MOS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MAIN_LED_GPIO_Port, MAIN_LED_Pin, GPIO_PIN_RESET);
	main_mos_counter = time_ms / 10;

}
void apoge_deploy(int time_ms)
{
	HAL_GPIO_WritePin(APOGE_MOS_GPIO_Port, APOGE_MOS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(APOGEE_LED_GPIO_Port, APOGEE_LED_Pin, GPIO_PIN_RESET);
	apoge_mos_counter = time_ms / 10;
}
void beep(int time_ms)
{
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
	beep_counter = time_ms / 10;
}

void calc_power(power_t* pow)
{
	if(is_power_1s == 1)
	{
		pow->wattage_calced= pow->wattage;
		pow->wattage = 0;
		is_power_1s = 0;
	}
		  HAL_ADC_Start(&hadc1);
		  HAL_ADC_Start(&hadc2);
		  HAL_ADC_PollForConversion(&hadc1, 2);
		  HAL_ADC_PollForConversion(&hadc2, 2);
		  float volt = (float)HAL_ADC_GetValue(&hadc1) * VOLT_COEF;
		  float current = (float)HAL_ADC_GetValue(&hadc2) * CRNT_COEF;
		  pow->voltage = volt + 0.05; // Offset val added.
		  pow->wattage += pow->voltage * current * (HAL_GetTick() - pow->last_time);
		  pow->last_time = HAL_GetTick();
}

void reset_fligth_datas()
{
	rocket_flight_datas.abs_angle = 0;
	rocket_flight_datas.accel_x = 0;
	rocket_flight_datas.accel_y = 0;
	rocket_flight_datas.accel_z = 0;
	rocket_flight_datas.alt_sea_level = 0;
	rocket_flight_datas.altitude = 0;
	rocket_flight_datas.angle_x = 0;
	rocket_flight_datas.angle_y = 0;
	rocket_flight_datas.angle_z = 0;
	rocket_flight_datas.data_taken_time = 0;
	rocket_flight_datas.velocity = 0;
	rocket_flight_datas.velocity_est = 0;
	rocket_flight_datas.is_new_data = 0;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
