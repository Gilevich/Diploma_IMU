/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lsm6dsox_reg.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SENSOR_BUS hi2c1
#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];

static float acceleration_mg[3];
static float angular_rate_mdps[3];

float output_data[3] = {0.0f, 0.0f, 0.0f};
float old_data[3] = {0.0f, 0.0f, 0.0f};
uint8_t tx_buffer[1000];
uint8_t whoami, rst;
volatile float pi = 3.141592653589793238f;
// madgwick var
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
volatile float beta = 0.01f;
volatile float sampleFreq = 10.0f;

const float dt = 0.1f;
// Kalman var
// macierz stanu
float A[6][6] = {{1.0f, 0, 0, -dt, 0, 0},
			     {0, 1.0f, 0, 0, -dt, 0},
			     {0, 0, 1.0f, 0, 0, -dt},
			     {0, 0, 0, 1.0f, 0, 0},
			     {0, 0, 0, 0, 1.0f, 0},
			     {0, 0, 0, 0, 0, 1.0f}};
//macierz wejścia
float B[6][3] = {{dt, 0, 0},
				 {0, dt, 0},
				 {0, 0, dt},
				 {0, 0, 0},
				 {0, 0, 0},
				 {0, 0, 0}};
//macierz wyjścia
float H[3][6] = {{1.0f, 0, 0, 0, 0, 0,},
			     {0, 1.0f, 0, 0, 0, 0,},
			     {0, 0, 1.0f, 0, 0, 0,}};
const float Kq = 0.005f; //bląd modelu
const float Kr = 10.0f; // bląd pomiaru
// macierz kowariancji modelu
float Q[6][6] = {{Kq, 0, 0, 0, 0, 0},
				 {0, Kq, 0, 0, 0, 0},
			     {0, 0, Kq, 0, 0, 0},
				 {0, 0, 0, Kq, 0, 0},
				 {0, 0, 0, 0, Kq, 0},
				 {0, 0, 0, 0, 0, Kq}};
// macierz kowariancji pomiaru
float R[3][3] = {{Kr, 0, 0},
				 {0, Kr, 0},
				 {0, 0, Kr}};
// macierz jednostkowa
float I[6][6] = {{1.0f, 0, 0, 0, 0, 0},
				 {0, 1.0f, 0, 0, 0, 0},
				 {0, 0, 1.0f, 0, 0, 0},
				 {0, 0, 0, 1.0f, 0, 0},
				 {0, 0, 0, 0, 1.0f, 0},
				 {0, 0, 0, 0, 0, 1.0f}};
float x_pri[6];
float p_pri[6][6];
float p_post[6][6];
float mult_A_xpost[6];
float mult_B_gyr_data[6];
float mult_A_ppost[6][6];
float A_trans[6][6];
float mult_A_ppost_A_trans[6][6];
float H_trans[6][3];
float mult_ppri_H_trans[6][3];
float mult_H_ppri[6][3];
float mult_H_ppri_H_trans[3][3];
float H_ppri_H_trans_plus_R_dev_1[3][3];
float GainK[6][3];
float mult_GainK_H[6][6];
float mult_H_x_pri[3];
float acc_data_minus_H_xpri[3];
float mult_GainK_acc_data_minus_H_xpri[6];
float I_minus_GainK_H[6][6];
int filter = 0, notfilter = 0, complem = 1, kalman = 2, madgwick = 3, stOrient = 4;
float offsetX = 0.4431f;
float offsetY = 0.4177f;
float offsetZ = -0.5678f;
stmdev_ctx_t dev_ctx;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
void tx_com(uint8_t *tx_buffer, uint16_t len);
static void platform_delay(uint32_t ms);
int32_t complementary_filter(float acc_data[], float gyr_data[]);
int32_t kalman_filter(float acc_data[], float gyr_data[]);
int32_t madgwick_filter(float acc_data[], float gyr_data[]);
void mult_matrix(int m, int n, int l, float A[m][n], float B[n][l], float C[m][l]);
float invSqrt(float x);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		uint8_t reg;

		/* Read output only if new values is available */
		lsm6dsox_xl_flag_data_ready_get(&dev_ctx, &reg);

		if (reg)
		{
			/* Read data */
			memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
			lsm6dsox_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
			acceleration_mg[0] = lsm6dsox_from_fs2_to_mg(data_raw_acceleration[0])/1000.0f;
			acceleration_mg[1] = lsm6dsox_from_fs2_to_mg(data_raw_acceleration[1])/1000.0f;
			acceleration_mg[2] = lsm6dsox_from_fs2_to_mg(data_raw_acceleration[2])/1000.0f;
			memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
			lsm6dsox_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
			angular_rate_mdps[0] = lsm6dsox_from_fs250_to_mdps(data_raw_angular_rate[0])/1000.0f;
			angular_rate_mdps[1] = lsm6dsox_from_fs250_to_mdps(data_raw_angular_rate[1])/1000.0f;
			angular_rate_mdps[2] = lsm6dsox_from_fs250_to_mdps(data_raw_angular_rate[2])/1000.0f;

			/*Filter data*/
			if (filter == complem)
				complementary_filter(acceleration_mg, angular_rate_mdps);
			else if (filter == kalman)
				kalman_filter(acceleration_mg, angular_rate_mdps);
			else if (filter == madgwick)
				madgwick_filter(acceleration_mg, angular_rate_mdps);

			/*Transmit data*/
			if (filter == notfilter)
			{
				sprintf((char *)tx_buffer,
					   "%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f\r",
					   acceleration_mg[0], acceleration_mg[1], acceleration_mg[2],
					   angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
			}
			else
			{
				sprintf((char *)tx_buffer,
						"%.2f,%.2f,%.2f\r",
						output_data[0], output_data[1], output_data[2]);
			}
		}
	}
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
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &SENSOR_BUS;
  /* Wait Boot Time */
  platform_delay(10);
  /* Check device ID */
  lsm6dsox_device_id_get(&dev_ctx, &whoami);

  if (whoami != LSM6DSOX_ID)
	  while(1)
	  {
		  HAL_GPIO_TogglePin(Red_led_GPIO_Port, Red_led_Pin);
		  HAL_Delay(500);
	  }

  /* Restore default configuration */
  lsm6dsox_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
	  lsm6dsox_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Disable I3C interface */
  lsm6dsox_i3c_disable_set(&dev_ctx, LSM6DSOX_I3C_DISABLE);
  /* Enable Block Data Update */
  lsm6dsox_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set XL and Gyro Output Data Rate */
  lsm6dsox_xl_data_rate_set(&dev_ctx, LSM6DSOX_XL_ODR_417Hz);
  lsm6dsox_gy_data_rate_set(&dev_ctx, LSM6DSOX_GY_ODR_417Hz);
  /* Set 2g full XL scale and 250 dps full Gyro */
  lsm6dsox_xl_full_scale_set(&dev_ctx, LSM6DSOX_2g);
  lsm6dsox_gy_full_scale_set(&dev_ctx, LSM6DSOX_250dps);

  HAL_GPIO_WritePin(Green_led_GPIO_Port, Green_led_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  memset(old_data, 0.0f, sizeof(old_data));
  filter = madgwick;
  HAL_TIM_Base_Start_IT(&htim6);
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{

  HAL_I2C_Mem_Write(handle, LSM6DSOX_I2C_ADD_L, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
  return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  HAL_I2C_Mem_Read(handle, LSM6DSOX_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}

void tx_com(uint8_t *tx_buffer, uint16_t len)
{
	CDC_Transmit_FS(tx_buffer, len);
}

int32_t complementary_filter(float acc_data[], float gyr_data[])
{
	float acc_data_[3] = {0.0f, 0.0f, 0.0f};
	float gyr_data_[3] = {0.0f, 0.0f, 0.0f};
	acc_data_[0] = atan(acc_data[0]/sqrt(acc_data[1] * acc_data[1] + acc_data[2] * acc_data[2]));
	acc_data_[1] = atan(acc_data[1]/sqrt(acc_data[0] * acc_data[0] + acc_data[2] * acc_data[2]));
	acc_data_[2] = atan(acc_data[2]/sqrt(acc_data[0] * acc_data[0] + acc_data[1] * acc_data[1]));
	gyr_data_[0] = gyr_data[0] - offsetX;
	gyr_data_[1] = gyr_data[1] - offsetY;
	gyr_data_[2] = gyr_data[2] - offsetZ;
	float k = 0.9995f;

	for (int i = 0; i < 3; i++)
	{
		output_data[i] = k * (old_data[i] + gyr_data_[i] * dt) + (1-k) * acc_data_[i];
		old_data[i] = output_data[i];
	}

	return 0;
}

int32_t kalman_filter(float acc_data[], float gyr_data[])
{
	//init
	float acc_data_[3];
	float gyr_data_[3];
	acc_data_[0] = atan(acc_data[0]/sqrt(acc_data[1] * acc_data[1] + acc_data[2] * acc_data[2]));
	acc_data_[1] = atan(acc_data[1]/sqrt(acc_data[0] * acc_data[0] + acc_data[2] * acc_data[2]));
	acc_data_[2] = atan(acc_data[2]/sqrt(acc_data[0] * acc_data[0] + acc_data[1] * acc_data[1]));
	gyr_data_[0] = gyr_data[0] - offsetX;
	gyr_data_[1] = gyr_data[1] - offsetY;
	gyr_data_[2] = gyr_data[2] - offsetZ;
	memset(x_pri, 0.0f, sizeof(x_pri));
	memset(p_pri, 0.0f, sizeof(p_pri));
	float x_post[6] = {output_data[0], output_data[1], output_data[2], 0.0f, 0.0f, 0.0f};
	memset(p_post, 0.0f, sizeof(p_post));
	memset(mult_A_xpost, 0.0f, sizeof(mult_A_xpost));
	memset(mult_B_gyr_data, 0.0f, sizeof(mult_B_gyr_data));
	memset(mult_A_ppost, 0.0f, sizeof(mult_A_ppost));
	memset(A_trans, 0.0f, sizeof(A_trans));
	memset(mult_A_ppost_A_trans, 0.0f, sizeof(mult_A_ppost_A_trans));
	memset(H_trans, 0.0f, sizeof(H_trans));
	memset(mult_ppri_H_trans, 0.0f, sizeof(mult_ppri_H_trans));
	memset(mult_H_ppri, 0.0f, sizeof(mult_H_ppri));
	memset(mult_H_ppri_H_trans, 0.0f, sizeof(mult_H_ppri_H_trans));
	memset(H_ppri_H_trans_plus_R_dev_1, 0.0f, sizeof(H_ppri_H_trans_plus_R_dev_1));
	memset(GainK, 0.0f, sizeof(GainK));
	memset(mult_GainK_H, 0.0f, sizeof(mult_GainK_H));
	memset(mult_H_x_pri, 0.0f, sizeof(mult_H_x_pri));
	memset(acc_data_minus_H_xpri, 0.0f, sizeof(acc_data_minus_H_xpri));
	memset(mult_GainK_acc_data_minus_H_xpri, 0.0f, sizeof(mult_GainK_acc_data_minus_H_xpri));
	memset(I_minus_GainK_H, 0.0f, sizeof(I_minus_GainK_H));

	//Predykcja

	// A * x_post
	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < 6; j++)
		{
			mult_A_xpost[i] += A[i][j] * x_post[j];
		}
	}

	// B * gyr_data
	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			mult_B_gyr_data[i] += B[i][j] * gyr_data_[j];
		}
	}

	//x_pri = A * x_post + B * gyr_data

	for (int i = 0; i < 6; i++)
	{
		x_pri[i] = mult_A_xpost[i] + mult_B_gyr_data[i];
	}

	// A * p_post
	mult_matrix(6, 6, 6, A, p_post, mult_A_ppost);
	// A'
	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < 6; j++)
		{
			A_trans[i][j] = A[j][i];
		}
	}
	//A' * A * p_post
	mult_matrix(6, 6, 6, mult_A_ppost, A_trans, mult_A_ppost_A_trans);
	// p_pri = A * p_post * A' + Q
	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < 6; j++)
		{
			p_pri[i][j] = mult_A_ppost_A_trans[i][j] + Q[i][j];
		}
	}

	//Korekcja
	// K = P_pri * H' * 1/(H * p_pri * H' + R)
	// H'
	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			H_trans[i][j] = H[j][i];
		}
	}

	mult_matrix(6, 6, 3, p_pri, H_trans, mult_ppri_H_trans);

	mult_matrix(3, 6, 6, H, p_pri, mult_H_ppri);

	mult_matrix(3, 6, 3, mult_H_ppri, H_trans, mult_H_ppri_H_trans);

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			if (i == j)
				H_ppri_H_trans_plus_R_dev_1[i][j] = 1.0f/(mult_H_ppri_H_trans[i][j] + R[i][j]);
			else
				H_ppri_H_trans_plus_R_dev_1[i][j] = (mult_H_ppri_H_trans[i][j] + R[i][j]);
		}
	}

	mult_matrix(6, 3, 3, mult_ppri_H_trans, H_ppri_H_trans_plus_R_dev_1, GainK);

	// x_post = xpri + GainK * (acc_data - H * x_pri)

	for (int k = 0; k < 3; k++)
	{
		for (int j = 0; j < 6; j++)
		{
			mult_H_x_pri[k] += H[k][j] * x_pri[j];
		}
	}

	for (int i = 0; i < 3; i++)
	{
		acc_data_minus_H_xpri[i] = acc_data_[i] - mult_H_x_pri[i];
	}


	for (int k = 0; k < 6; k++)
	{
		for (int j = 0; j < 3; j++)
		{
			mult_GainK_acc_data_minus_H_xpri[k] += GainK[k][j] * acc_data_minus_H_xpri[j];
		}
	}

	for (int i = 0; i < 6; i++)
	{
		x_post[i] = x_pri[i] + mult_GainK_acc_data_minus_H_xpri[i];
	}

	//p_post = (I - K * H) * P_pri

	mult_matrix(6, 3, 6, GainK, H, mult_GainK_H);


	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < 6; j++)
		{
			I_minus_GainK_H[i][j] = I[i][j] - mult_GainK_H[i][j];
		}
	}

	mult_matrix(6, 6, 6, I_minus_GainK_H, p_pri, p_post);

	output_data[0] = x_post[0];
	output_data[1] = x_post[1];
	output_data[2] = x_post[2];
	return 0;
}

void mult_matrix(int m, int n, int l, float A[m][n], float B[n][l], float C[m][l])
{
	for (int k = 0; k < 6; k++)
	{
		for (int i = 0; i < 6; i++)
		{
			for (int j = 0; j < 6; j++)
			{
				C[k][i] += A[k][j] * B[j][i];
			}
		}
	}
}

int32_t madgwick_filter(float acc_data[], float gyr_data[])
{

	float ax = acc_data[0];
	float ay = acc_data[1];
	float az = acc_data[2];
	float gx = ToRad(gyr_data[0] - offsetX);
	float gy = ToRad(gyr_data[1] - offsetY);
	float gz = ToRad(gyr_data[2] - offsetZ);

	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;


	// quatern2euler
	output_data[0] = ToDeg(atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2)));
	output_data[1] = ToDeg(asin(2*(q0*q2-q3*q1)));
	output_data[2] = ToDeg(atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3)));


	return 0;
}

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

static void platform_delay(uint32_t ms)
{
  HAL_Delay(ms);
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

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
