/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int mode = 0;
#define gap_ms 100
/*
mode == 0:πÿ±’µ∆π‚
mode == 1:∫Ïµ∆≥£¡¡
mode == 2:¿∂µ∆≥£¡¡
mode == 3:∫Ïµ∆…¡À∏
mode == 4:¿∂µ∆…¡À∏
*/

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == KEY_R_Pin){
		mode = 1;
	}
	if(GPIO_Pin == KEY_Pin){
		mode = 0;
	}
	if(GPIO_Pin == KEY_B_Pin){
		mode = 2;
	}

}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == RX2_Pin){
		if(mode == 1){
			mode = 3;
		}
		else if(mode == 2){
			mode = 4;
		}
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

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(mode == 0){
		  HAL_GPIO_WritePin(EN_R_GPIO_Port,EN_R_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(EN_B_GPIO_Port,EN_B_Pin,GPIO_PIN_RESET);
	  }
	  if(mode == 1){
		  HAL_GPIO_WritePin(EN_R_GPIO_Port,EN_R_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(EN_B_GPIO_Port,EN_B_Pin,GPIO_PIN_RESET);
	  }
	  if(mode == 2){
		  HAL_GPIO_WritePin(EN_R_GPIO_Port,EN_R_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(EN_B_GPIO_Port,EN_B_Pin,GPIO_PIN_SET);
	  }
	  if(mode == 3){
		  HAL_GPIO_WritePin(EN_R_GPIO_Port,EN_R_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(EN_B_GPIO_Port,EN_B_Pin,GPIO_PIN_RESET);
		  HAL_Delay(gap_ms);
		  HAL_GPIO_WritePin(EN_R_GPIO_Port,EN_R_Pin,GPIO_PIN_SET);
		  HAL_Delay(gap_ms);
		  HAL_GPIO_WritePin(EN_R_GPIO_Port,EN_R_Pin,GPIO_PIN_RESET);
		  HAL_Delay(gap_ms);
		  HAL_GPIO_WritePin(EN_R_GPIO_Port,EN_R_Pin,GPIO_PIN_SET);
		  HAL_Delay(gap_ms);
		  HAL_GPIO_WritePin(EN_R_GPIO_Port,EN_R_Pin,GPIO_PIN_RESET);		  
		  mode = 1;
		  
	  }
	  if(mode == 4){
		  HAL_GPIO_WritePin(EN_R_GPIO_Port,EN_R_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(EN_B_GPIO_Port,EN_B_Pin,GPIO_PIN_RESET);
		  HAL_Delay(gap_ms);
		  HAL_GPIO_WritePin(EN_B_GPIO_Port,EN_B_Pin,GPIO_PIN_SET);
		  HAL_Delay(gap_ms);
		  HAL_GPIO_WritePin(EN_B_GPIO_Port,EN_B_Pin,GPIO_PIN_RESET);
		  HAL_Delay(gap_ms);
		  HAL_GPIO_WritePin(EN_B_GPIO_Port,EN_B_Pin,GPIO_PIN_SET);
		  HAL_Delay(gap_ms);
		  HAL_GPIO_WritePin(EN_B_GPIO_Port,EN_B_Pin,GPIO_PIN_RESET);		  
		  mode = 2;
		
		}
	  
		if(mode == 5)
		{
		  HAL_GPIO_WritePin(EN_B_GPIO_Port,EN_B_Pin,GPIO_PIN_SET);
		  HAL_Delay(50);
		  HAL_GPIO_WritePin(EN_B_GPIO_Port,EN_B_Pin,GPIO_PIN_RESET);
		  HAL_Delay(50);
		  HAL_GPIO_WritePin(EN_B_GPIO_Port,EN_R_Pin,GPIO_PIN_SET);
		  HAL_Delay(50);
		  HAL_GPIO_WritePin(EN_B_GPIO_Port,EN_R_Pin,GPIO_PIN_RESET);
		  HAL_Delay(50);
					  HAL_GPIO_WritePin(EN_B_GPIO_Port,EN_B_Pin,GPIO_PIN_SET);
		  HAL_Delay(50);
		  HAL_GPIO_WritePin(EN_B_GPIO_Port,EN_B_Pin,GPIO_PIN_RESET);
		  HAL_Delay(50);
		  HAL_GPIO_WritePin(EN_B_GPIO_Port,EN_R_Pin,GPIO_PIN_SET);
		  HAL_Delay(50);
		  HAL_GPIO_WritePin(EN_B_GPIO_Port,EN_R_Pin,GPIO_PIN_RESET);
		  HAL_Delay(50);
					  HAL_GPIO_WritePin(EN_B_GPIO_Port,EN_B_Pin,GPIO_PIN_SET);
		  HAL_Delay(50);
		  HAL_GPIO_WritePin(EN_B_GPIO_Port,EN_B_Pin,GPIO_PIN_RESET);
		  HAL_Delay(50);
		  HAL_GPIO_WritePin(EN_B_GPIO_Port,EN_R_Pin,GPIO_PIN_SET);
		  HAL_Delay(50);
		  HAL_GPIO_WritePin(EN_B_GPIO_Port,EN_R_Pin,GPIO_PIN_RESET);
		  HAL_Delay(50);
		  		  HAL_GPIO_WritePin(EN_B_GPIO_Port,EN_B_Pin,GPIO_PIN_SET);
		  HAL_Delay(50);
		  HAL_GPIO_WritePin(EN_B_GPIO_Port,EN_B_Pin,GPIO_PIN_RESET);
		  HAL_Delay(50);
		  HAL_GPIO_WritePin(EN_B_GPIO_Port,EN_R_Pin,GPIO_PIN_SET);
		  HAL_Delay(50);
		  HAL_GPIO_WritePin(EN_B_GPIO_Port,EN_R_Pin,GPIO_PIN_RESET);
		  HAL_Delay(50);
		  		  HAL_GPIO_WritePin(EN_B_GPIO_Port,EN_B_Pin,GPIO_PIN_SET);
		  HAL_Delay(50);
		  HAL_GPIO_WritePin(EN_B_GPIO_Port,EN_B_Pin,GPIO_PIN_RESET);
		  HAL_Delay(50);
		  HAL_GPIO_WritePin(EN_B_GPIO_Port,EN_R_Pin,GPIO_PIN_SET);
		  HAL_Delay(50);
		  HAL_GPIO_WritePin(EN_B_GPIO_Port,EN_R_Pin,GPIO_PIN_RESET);
		  HAL_Delay(50);
		  		  HAL_GPIO_WritePin(EN_B_GPIO_Port,EN_B_Pin,GPIO_PIN_SET);
		  HAL_Delay(50);
		  HAL_GPIO_WritePin(EN_B_GPIO_Port,EN_B_Pin,GPIO_PIN_RESET);
		  HAL_Delay(50);
		  HAL_GPIO_WritePin(EN_B_GPIO_Port,EN_R_Pin,GPIO_PIN_SET);
		  HAL_Delay(50);
		  HAL_GPIO_WritePin(EN_B_GPIO_Port,EN_R_Pin,GPIO_PIN_RESET);
		  HAL_Delay(50);
		  
		  mode = 0;
	  }
		
	  if(HAL_GPIO_ReadPin(GPIOB,KEY_B_Pin) == GPIO_PIN_RESET && HAL_GPIO_ReadPin(GPIOB,KEY_R_Pin) == GPIO_PIN_RESET && mode != 5)
	  {
			
			mode = 5;
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

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
  /* EXTI4_15_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EN_R_Pin|EN_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : KEY_Pin KEY_R_Pin KEY_B_Pin */
  GPIO_InitStruct.Pin = KEY_Pin|KEY_R_Pin|KEY_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RX2_Pin */
  GPIO_InitStruct.Pin = RX2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RX2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_R_Pin EN_B_Pin */
  GPIO_InitStruct.Pin = EN_R_Pin|EN_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

}

/* USER CODE BEGIN 4 */

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
