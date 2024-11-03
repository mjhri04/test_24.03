/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>
#define _USE_MATH_DEFINES
#include <sys/time.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

/* USER CODE BEGIN PV */
unsigned char IMUarray[31] = {0};
char *pos = NULL;
float roll = 0.0, pitch = 0.0, yaw = 0.0;
float roll_tp = 0.0, pitch_tp = 0.0, yaw_tp = 0.0;
int cnt0 = 0, cnt1 = 0, cnt2 = 0, cnt3 = 0, cnt4 = 0;
char alpha[10] = {0}, beta[10] = {0}, gamm[10] = {0};
double target_pitch = 0;
double target_v = 0;
//double Kp = 17.35555555;
double Kp = 15.55555555;
//double Ki = 0.000001;
//double Kd = 5.065;
double Kd = 0.5765;
double Ki=0;//1.0000000001;
//double Kd=0;
double pTerm, iTerm, dTerm;
double prevError = 0;
double integratedSum = 0;
struct timeval tv, tv2;
double current_pitch = 0;
double current_w1 = 0;
double current_w2 = 0;


uint16_t counter1 = 0;
uint16_t counter2 = 0;
uint16_t current_counter1 = 0;
uint16_t current_counter2 = 0;
float count1_w = 0.0;
float count2_w = 0.0;
uint16_t Motor_CCR1 = 2100;
uint16_t Motor_CCR2 = 2100;
int cnt = 0;
int cnt_20ms=0;
int32_t delta_count1;
int32_t delta_count2;
double angle1 = 0;
double angle2 = 0;
double angle1_pre = 0;
double angle2_pre = 0;
double omega1 = 0;
double omega2 = 0;
double rpm1 = 0;
double rpm2 = 0;

double d_error;
int flag = 0;
double eeeeeeeeeee = 0;
double d_deltaT = 0.020;
double c_roll;

int a = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
double PI(double target_v, double current_v);
double PD(double current_pitch);
void get_pitch();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim9){ //?  ?  주기]
		cnt++; //0.001s

		angle1_pre = angle1;
		current_counter1 = counter1;
		counter1 = TIM2->CNT;
		delta_count1 = (counter1 - current_counter1);

		angle2_pre = angle2;
		current_counter2 = counter2;
		counter2 = TIM5->CNT;
		delta_count2 = (counter2 - current_counter2);

		if(counter1>60000 && current_counter1 < 10000) delta_count1 -= 65535;
		else if(counter1 < 10000 && current_counter1 > 60000) delta_count1 += 65535;
		angle1 += (((double)delta_count1/2048.0/19.0)*360)*(M_PI/180.0);
		omega1 = (angle1 - angle1_pre) / 0.001;
		rpm1 = ((60/(2*M_PI))*omega1);

		if(counter2>60000 && current_counter2 < 10000) delta_count2 -= 65535;
		else if(counter2 < 10000 && current_counter2 > 60000) delta_count2 += 65535;
		angle2 += (((double)delta_count2/2048.0/19.0)*360)*(M_PI/180.0);
		omega2 = (angle2 - angle2_pre) / 0.001;
		rpm2 = ((60/(2*M_PI))*omega2);

		// 10 ??????  2ms(encoder)  20ms(imu)


		if(cnt%20 == 0){
			d_error = target_pitch - current_pitch;
			cnt_20ms++;
			get_pitch();
			current_pitch = roll; // 멈추�?? if문에 ?���??
			c_roll = PD(current_pitch);

			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

			if(PD(current_pitch)<4200 && PD(current_pitch)>0){
				target_v = PD(current_pitch);
			}

			if(PD(current_pitch)<0 ){
				target_v = -1*PD(current_pitch);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

			}

			if(PD(current_pitch)>4200){
				target_v = 4200;
				//최댓 ?????? ?   ??????
			}

			if(current_pitch > 15 || current_pitch <-15){
				target_v = 0;
			}

			prevError = d_error;
		}
//		Motor_CCR = target_v;

		if(cnt%2 == 0){

			current_w1 = omega1;
			Motor_CCR1 = PI(target_v, current_w1);
			current_w2 = omega2;
			Motor_CCR2 = PI(target_v, current_w2);
			if(current_pitch <5 && current_pitch >-5){
				Kp = 15.05;
				Kd = 0.655;
				Ki = 0.01;
				if(PI(target_v, current_w1)<4200 && PI(target_v, current_w1)>0){
					Motor_CCR1 = PI(target_v, current_w1);
				}
				if(PI(target_v, current_w1)<0){
					Motor_CCR1 = -1*PI(target_v, current_w1);
					//모터 방향 ?  ?
				}
				if(PI(target_v, current_w1)>4200){
					Motor_CCR1 = 4200;

					//최댓 ?????? ?   ??????
				}
				if(PI(target_v, current_w2)<4200 && PI(target_v, current_w2)>0){
					Motor_CCR2 = PI(target_v, current_w2);
				}
				if(PI(target_v, current_w2)<0){
					Motor_CCR2 = -1*PI(target_v, current_w2);
					//모터 방향 ?  ?
				}
				if(PI(target_v, current_w2)>4200){
					Motor_CCR2 = 4200;
					//최댓 ?????? ?   ??????
				}
			}
			else if((current_pitch < 10 && current_pitch >5) || (current_pitch >-10 && current_pitch <-5)){
				Kp = 17.0;
				Kd = 0.2;
				Ki = 0.001;
				if(PI(target_v, current_w1)<4200 && PI(target_v, current_w1)>0){
					Motor_CCR1 = PI(target_v, current_w1);
				}
				if(PI(target_v, current_w1)<0){
					Motor_CCR1 = -1*PI(target_v, current_w1);
					//모터 방향 ?  ?
				}
				if(PI(target_v, current_w1)>4200){
					Motor_CCR1 = 4200;
				}
				if(PI(target_v, current_w2)<4200 && PI(target_v, current_w2)>0){
					Motor_CCR2 = PI(target_v, current_w2);
				}
				if(PI(target_v, current_w2)<0){
					Motor_CCR2 = -1*PI(target_v, current_w2);
					//모터 방향 ?  ?
				}
				if(PI(target_v, current_w2)>4200){
					Motor_CCR2 = 4200;
					//최댓 ?????? ?   ??????
				}
			}
			else if((current_pitch < 15 && current_pitch >10) || (current_pitch >-15 && current_pitch <-10)) {
				Motor_CCR1 = 4200;
				Motor_CCR2 = 4200;
			}
//			else if((current_pitch < 15 && current_pitch >10) || (current_pitch >-15 && current_pitch <-10)){
//				Kp = 19.5;
//				Kd = 0.08;
//				Ki = 0.00001;
//				if(PI(target_v, current_w1)<4200 && PI(target_v, current_w1)>0){
//					Motor_CCR1 = PI(target_v, current_w1);
//				}
//				if(PI(target_v, current_w1)<0){
//					Motor_CCR1 = -1*PI(target_v, current_w1);
//					//모터 방향 ?  ?
//				}
//				if(PI(target_v, current_w1)>4200){
//					Motor_CCR1 = 4200;
//				}
//				if(PI(target_v, current_w2)<4200 && PI(target_v, current_w2)>0){
//					Motor_CCR2 = PI(target_v, current_w2);
//				}
//				if(PI(target_v, current_w2)<0){
//					Motor_CCR2 = -1*PI(target_v, current_w2);
//					//모터 방향 ?  ?
//				}
//				if(PI(target_v, current_w2)>4200){
//					Motor_CCR2 = 4200;
//					//최댓 ?????? ?   ??????
//				}
//			}
/*			if(PI(target_v, current_w1)<4200 && PI(target_v, current_w1)>0){
				Motor_CCR1 = PI(target_v, current_w1);
			}
			if(PI(target_v, current_w1)<0){
				Motor_CCR1 = -1*PI(target_v, current_w1);
				//모터 방향 ?  ?
			}
			if(PI(target_v, current_w1)>4200){
				Motor_CCR1 = 4200;
				//최댓 ?????? ?   ??????
			}*/


			/*if(PI(target_v, current_w2)<4200 && PI(target_v, current_w2)>0){
				Motor_CCR2 = PI(target_v, current_w2);
			}
			if(PI(target_v, current_w2)<0){
				Motor_CCR2 = -1*PI(target_v, current_w2);
//				//모터 방향 ?  ?
			}
			if(PI(target_v, current_w2)>4200){
				Motor_CCR2 = 4200;
				//최댓 ?????? ?   ??????
			}*/
			else if(target_v == 0){
				Motor_CCR1 = 0;
				Motor_CCR2 = 0;
			}
		}

		// Motor_CCR = tager_v + Motor_CCR;  // target_v + Motor_CCR

		TIM3->CCR1 = Motor_CCR1;
		/*HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); //break*/

		TIM4->CCR1 = Motor_CCR2;
		/*HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET); //break*/






//
//		counter1 = TIM2->CNT;
//		counter2 = TIM5->CNT;
//		delta_count1 = (int32_t)(counter1 - current_counter1);
//		delta_count2 = (int32_t)(counter2 - current_counter2);
//		count1_w = (2*M_PI*delta_count1)/(0.1*512*4);
//		count2_w = (2*M_PI*delta_count2)/(0.1*512*4);
//		current_counter1 = counter1;
//		current_counter2 = counter2;
	}

}
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM9_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART6_UART_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start_IT(&htim5, TIM_CHANNEL_ALL);

  HAL_TIM_Base_Start_IT(&htim9);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim4, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* TIM4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
  /* TIM5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM5_IRQn);
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 839;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4200-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 4200-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 839;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 99;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */
  HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM5_IRQn);
  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 839;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 99;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4_Pin|GPIO_PIN_5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0_Pin|GPIO_PIN_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PCPin PCPin */
  GPIO_InitStruct.Pin = GPIO_PIN_4_Pin|GPIO_PIN_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin */
  GPIO_InitStruct.Pin = GPIO_PIN_0_Pin|GPIO_PIN_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void get_pitch(){
	unsigned char star = '*';

	HAL_UART_Transmit(&huart1, &star, 1, 10);
	HAL_UART_Receive_DMA(&huart1, &IMUarray[0], 31);
	for(int i = 1; i < 31; i++){
		if(IMUarray[i] == ','){
			if(cnt0 == 0){
				cnt1 = i + 1;
			}
			else if (cnt0 == 1){
				cnt2 = i + 1;
			}
			cnt0++;
		}
		else{
			switch (cnt0){
			case 0:{
				alpha[i - 1] = IMUarray[i];
				break;
			}
			case 1:{
				beta[i - cnt1] = IMUarray[i];
				break;
			}
			case 2:{
				gamm[i - cnt2] = IMUarray[i];
				break;
			}
			}
		}
		if(IMUarray[i] == '\r'){
			cnt3 = i + 1;
			break;
		}
	}
	for (int j = cnt1 - 2; j < 7; j++){
		alpha[j] = '0';
	}
	for (int k = cnt2 - (cnt1); k < 8; k++){
		beta[k - 1] = '0';
	}
	for (int l = cnt3 - (cnt2); l < 8; l++){
		gamm[l - 1] = '0';
	}
	cnt0 = 0;

	roll_tp = strtod(alpha, &pos);
	pitch_tp = strtod(beta, &pos);
	yaw_tp = strtod(gamm, &pos);

	cnt4 += 1;
	pitch = roll_tp;
	roll = pitch_tp;
	yaw = yaw_tp;
}

unsigned long long getTimestamp(){
	gettimeofday(&tv, NULL);
	return (unsigned long long)tv.tv_sec * 1000000 + tv.tv_usec;
}
double P_D(double current_pitch){
	double error;
	error = target_pitch - current_pitch;
	return pTerm = Kp*error;
}

double P_I(double target_v, double current_v){
	double error;
	error = target_v - current_v;
	return pTerm = Kp*error;
}

double D(double current_pitch){
	d_deltaT = 0.020;
	/*prevError = d_error;
	d_error = target_pitch - current_pitch;*/
	dTerm = (double)Kd * (d_error - prevError)/d_deltaT;

	eeeeeeeeeee = (double)d_error - prevError;
	return dTerm;
}

double I(double target_v, double current_v){
	double error;
	error = target_v - current_v;
	double deltaT = 0.002;
	//integratedSum = integratedSum + error * deltaT;
	if(integratedSum < 4200 && integratedSum >-4200){
		integratedSum += error * deltaT;
	}
	/*static uint32_t integralResetCounter = 0;
	static const uint32_t integralResetInterval = 5000; // 500 * 0.001s = 0.5�??
	integralResetCounter++;
	if (integralResetCounter >= integralResetInterval) {
		integratedSum = 0.0; // ?��분기 초기?��
		integralResetCounter = 0; // 카운?�� 초기?��
	}*/
	if(integratedSum > 4200){
		integratedSum = 4200;
	}
	else if(integratedSum < -4200){
		integratedSum = -4200;
	}
	return iTerm = Ki * integratedSum;

}

double PD(double current_pitch){
	return P_D(current_pitch) + D(current_pitch);
}

double PI(double target_v, double current_v){
	return P_I(target_v, current_v) + I(target_v, current_v);
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
