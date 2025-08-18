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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct bckp_sram_datas
{
	bme_parameters_t	bme_params;
	bmi088_offsets_t	bmi_offsets;
	float q[4];
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
static bme280_struct_t 	bme_sensor_s;
static bmi088_struct_t	bmi_imu_s;
bckp_sram_datas_t* backup_datas = (bckp_sram_datas_t*) BKPSRAM_BASE;
uint8_t str[200];
extern uint8_t *dma_rx_buf;
extern uint8_t *gps_buf;
UKB_test_t test_datas;
flight_data_t rocket_flight_datas;
uint8_t last_mode = 0;
extern float euler[3];
S_GPS_L86_DATA gps_s;
uint8_t is_1second = 0;
uint32_t main_mos_counter = 0;
uint32_t apoge_mos_counter = 0;
extern flight_states_e rocket_status;
static e22_conf_struct_t lora_1;
uint8_t *packed_datas_p;
power_t power_s;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t bme280_begin(void);
uint8_t bmi088_begin(void);
void bmi_callback(bmi088_struct_t *BMI);
void lora_init(void);

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
  MX_TIM7_Init();
  MX_TIM9_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  HAL_PWR_EnableBkUpAccess();
  RCC->AHB1ENR |= RCC_AHB1ENR_BKPSRAMEN;
  HAL_PWR_EnableBkUpReg();

  HAL_GPIO_WritePin(SENS_RES_GPIO_Port, SENS_RES_Pin, GPIO_PIN_SET);
  HAL_Delay(50);
  HAL_GPIO_WritePin(SENS_RES_GPIO_Port, SENS_RES_Pin, GPIO_PIN_RESET);
  HAL_Delay(50);

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

    // Lora module config.
    lora_init();

    // Config phase finished beep.
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

    // Start timer interrupts.
    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim7);
    HAL_TIM_Base_Start_IT(&htim9);

    // Start receiving DMA form GNSS UART.
    HAL_UART_Receive_DMA(&RS232_HNDLR, dma_rx_buf, RX_BUFFER_LEN + 6);
    __HAL_UART_ENABLE_IT(&RS232_HNDLR, UART_IT_IDLE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  bme280_update(&bme_sensor_s);
	  bmi088_update(&bmi_imu_s);

	  if(get_test_mode() != last_mode)
	  {
		  last_mode = get_test_mode();
		  switch(get_test_mode())
		  {
		  case 0:
			  HAL_TIM_Base_Stop_IT(&htim4);
			  HAL_TIM_Base_Stop_IT(&htim3);
			  HAL_TIM_Base_Start_IT(&htim5);
			  break;

		  case 1:
			  HAL_TIM_Base_Stop_IT(&htim5);
			  HAL_TIM_Base_Start_IT(&htim3);
			  break;

		  case 2:
			  HAL_TIM_Base_Stop_IT(&htim5);
			  HAL_TIM_Base_Start_IT(&htim4);
			  break;
		  }
	  }
	  if(is_1second)	// This condition works at 1Hz.
	  {
		  is_1second = 0;
		  Usr_GpsL86GetValues(&gps_s, &GPS_UART_HNDLR);

		  HAL_ADC_Start(&hadc1);
		  HAL_ADC_Start(&hadc2);
		  HAL_ADC_PollForConversion(&hadc1, 2);
		  HAL_ADC_PollForConversion(&hadc2, 2);
		  float volt = (float)HAL_ADC_GetValue(&hadc1) * VOLT_COEF;
		  float current = (float)HAL_ADC_GetValue(&hadc2) * CRNT_COEF;
		  power_s.voltage = volt + 0.05; // Offset val added.
		  power_s.current = current;

	  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
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
uint8_t bme280_begin()
{
	bme_sensor_s.device_config.filter = BME280_FILTER_8;
	bme_sensor_s.device_config.mode = BME280_MODE_NORMAL;
	bme_sensor_s.device_config.over_sampling = BME280_OS_8;
	bme_sensor_s.device_config.period = BME280_PERIOD_05;
	bme_sensor_s.device_config.BME_I2C = &BAR_I2C_HNDLR;
	bme_sensor_s.parameters = &backup_datas->bme_params; //if no backup data, write NULL
	return bme280_init(&bme_sensor_s);
}

void bmi_callback(bmi088_struct_t *BMI)
{
	updateQuaternion(-BMI->datas.gyro_z * M_PI / 180.0, BMI->datas.gyro_x * M_PI / 180.0, -BMI->datas.gyro_y * M_PI / 180.0, BMI->datas.delta_time);
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

	HAL_UART_Transmit(huart_disp, (uint8_t*)str, strlen(str), 50);
	HAL_UART_Transmit(huart_disp, (uint8_t*)"\r\n", 2, 50);
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
    if (htim->Instance == TIM3)
    {
		rocket_flight_datas.altitude = test_datas.altitude;
		rocket_flight_datas.accel_X = test_datas.accel_X;
		rocket_flight_datas.accel_Y = test_datas.accel_Y;
		rocket_flight_datas.accel_Z = test_datas.accel_Z;
		rocket_flight_datas.angle_X = test_datas.angle_X;
		rocket_flight_datas.angle_Y = test_datas.angle_Y;
		rocket_flight_datas.angle_Z = test_datas.angle_Z;
		algorithm_1_update(&rocket_flight_datas);
		ukb_test_stat_update(rocket_status);
    }
    if (htim->Instance == TIM4)
    {
		test_datas.altitude = bme_sensor_s.datas.height;
		test_datas.pressure = bme_sensor_s.datas.pressure;
		test_datas.accel_X 	= bmi_imu_s.datas.acc_x * TO_SI;
		test_datas.accel_Y 	= bmi_imu_s.datas.acc_z * TO_SI;
		test_datas.accel_Z 	= bmi_imu_s.datas.acc_y * TO_SI;
		test_datas.angle_X 	= euler[0];	//bmi_imu_s.datas.gyro_x_angle;
		test_datas.angle_Y 	= bmi_imu_s.datas.gyro_z_angle;
		test_datas.angle_Z 	= euler[2]; //bmi_imu_s.datas.gyro_z_angle;
    	uint8_t datas_packed[36];
    	pack_datas_for_test(datas_packed, &test_datas);
    	HAL_UART_Transmit(&RS232_HNDLR, datas_packed, 36, 50);
    }
    if(htim->Instance == TIM5)
    {
		rocket_flight_datas.altitude = bme_sensor_s.datas.altitude;
		rocket_flight_datas.accel_X = bmi_imu_s.datas.acc_x;
		rocket_flight_datas.accel_Y = bmi_imu_s.datas.acc_y;
		rocket_flight_datas.accel_Z = bmi_imu_s.datas.acc_z;
		rocket_flight_datas.angle_X = euler[0];	//bmi_imu_s.datas.gyro_x_angle;
		rocket_flight_datas.angle_Y = euler[1];	//bmi_imu_s.datas.gyro_y_angle;
		rocket_flight_datas.angle_Z = euler[2];	//bmi_imu_s.datas.gyro_z_angle;

		packed_datas_p = packDatas(&bmi_imu_s, &bme_sensor_s, &gps_s, &power_s, rocket_status + 1);
		algorithm_1_update(&rocket_flight_datas);
    }
    if(htim->Instance == TIM6)
    {
    	//packDatas(&bmi_imu_s, &bme_sensor_s, 0, 0, 0);
    	is_1second = 1;
    }
    if(htim->Instance == TIM7)
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
    }
    if(htim->Instance == TIM9)
    {
		  if(power_s.voltage > 7.0)
		  {
			  e22_chMode_transmit(&lora_1);
			  send_datas(&TELEM_UART_HNDLR, packed_datas_p, 64);
		  }
		  send_datas(&TTL_HNDLR, packed_datas_p, 64);
    }

}

void main_deploy()
{
	HAL_GPIO_WritePin(MAIN_MOS_GPIO_Port, MAIN_MOS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MAIN_LED_GPIO_Port, MAIN_LED_Pin, GPIO_PIN_RESET);
	main_mos_counter = 50;
}
void apoge_deploy()
{
	HAL_GPIO_WritePin(APOGE_MOS_GPIO_Port, APOGE_MOS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(APOGEE_LED_GPIO_Port, APOGEE_LED_Pin, GPIO_PIN_RESET);
	apoge_mos_counter = 50;
}

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
	lora_1.channel			=	25;

	lora_1.pins.m0_pin = RF_M0_Pin;
	lora_1.pins.m0_pin_port = RF_M0_GPIO_Port;
	lora_1.pins.m1_pin = RF_M1_Pin;
	lora_1.pins.m1_pin_port = RF_M1_GPIO_Port;

	e22_init(&lora_1, &TELEM_UART_HNDLR);

	HAL_UART_DeInit(&TELEM_UART_HNDLR);
	TELEM_UART_HNDLR.Init.BaudRate = 115200;
	HAL_UART_Init(&TELEM_UART_HNDLR);
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
