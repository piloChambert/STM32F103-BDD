/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "adc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */
#define SYSCLOCK 72000000

volatile uint32_t delayLength = 512;
volatile uint16_t buffer[8192];
volatile uint16_t p = 0;

volatile uint16_t start = 2000;
volatile int32_t max = 200;
volatile int32_t t = 0;
volatile int32_t inc = 1;

volatile uint32_t period;
volatile uint32_t tim3_counter = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define LERP(A, B, X) (A * X + B * (1.0f - X))
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
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  unsigned short pinState = 0;
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, pinState);


  while(HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK);
  while(HAL_ADCEx_Calibration_Start(&hadc2) != HAL_OK);

  uint32_t sampleRate = 50000;
  TIM2->ARR = SYSCLOCK / sampleRate;
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim2);

  TIM2->ARR = 720 * 5.0f;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		HAL_ADC_Start(&hadc2);
		HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
		uint16_t raw = HAL_ADC_GetValue(&hadc2);

		float freq = LERP(0.25f, 5.0f, (raw / 4096.0f));// 4.0f;

		HAL_ADC_Start(&hadc2);
		HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
		raw = HAL_ADC_GetValue(&hadc2);
		float depth = raw / 4096.0f;

		max = 50 + 2830 / (freq * 2.0f);
		max *= depth;

		float FTim3 = max * freq * 2.0f;

		uint32_t prescaler = 450;
		period = 72000000 / (prescaler * FTim3);

		uint32_t mid = 2880;
		//start = mid - max / 2;

		TIM3->ARR = period-1;
		TIM3->PSC = prescaler-1;

		HAL_Delay(20);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
#define DAC_OUTPUT(X) GPIOA->ODR = ((X & 0x7FF) << 1) + ((X & 0x800) << 4);

//#define DEBUG_DAC

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim == &htim3) {
		if(t >= max)
			inc = -1;
		if(t <= 0)
			inc = 1;

		t = t + inc;

#ifndef DEBUG_DAC
		TIM2->ARR = 720 + start + t;
#endif
	}

	else if(htim == &htim2) {

#ifdef DEBUG_DAC
#define M_PI 3.1415926535897932384
		int s = 64;
		//float a = 2.0f * 3.14f * (float)p / (float)s;
		//GPIOA->ODR = buffer[p] = ((uint16_t)((sin(200.0f*a) * 0.5f + 0.5f) * 4096.0f)) << 1;
		//GPIOA->ODR =  (p <= s ? p : 2 * s - p);
		//p = (p + 1) % (s * 2 - 1);
		//GPIOA->ODR = (p > 0 ? 512 : 0) << 1;

		int v = 4095.0f * (sinf(2.0f * M_PI * (float)p / (float)s) * 0.5f + 0.5f);
		DAC_OUTPUT(v);

		p = (p + 1) % s;
#else
		uint16_t value = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Start(&hadc1);

		DAC_OUTPUT(buffer[p]);

		buffer[p] = value + 100;

		p = (p + 1) % delayLength;
#endif
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
