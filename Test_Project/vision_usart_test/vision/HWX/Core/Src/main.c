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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "driver.h"
#include "device.h"
#include "3508_motor.h"
#include "2006_motor.h"
#include "6020_motor.h"
#include "dji_pid.h"
#include "remote.h"
#include "my_chassis.h"
#include "my_gimbal.h"
#include "car.h"
#include "bmi.h"
#include "my_shoot.h"
#include "vision.h"
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
/*底盘电机*/
motor_3508_t           motor_3508_LF_structure;
motor_3508_base_info_t motor_3508_LF_base_info;
motor_3508_info_t      motor_3508_LF_info;

motor_3508_t           motor_3508_RF_structure;
motor_3508_base_info_t motor_3508_RF_base_info;
motor_3508_info_t      motor_3508_RF_info;

motor_3508_t           motor_3508_LB_structure;
motor_3508_base_info_t motor_3508_LB_base_info;
motor_3508_info_t      motor_3508_LB_info;

motor_3508_t           motor_3508_RB_structure;
motor_3508_base_info_t motor_3508_RB_base_info;
motor_3508_info_t      motor_3508_RB_info;
/*底盘*/
chassis_t              chassis_structure;
chassis_config_t       chassis_config_structure;
chassis_info_t         chassis_info_structure;
/*云台电机*/
motor_6020_t           motor_6020_YAW_structure;
motor_6020_base_info_t motor_6020_YAW_base_info;
motor_6020_info_t      motor_6020_YAW_info;

motor_6020_t           motor_6020_PIT_structure;
motor_6020_base_info_t motor_6020_PIT_base_info;
motor_6020_info_t      motor_6020_PIT_info;


/*地盘电机PID*/
pid_t                  chassis_pid_speed_structure[4];
pid_t                  chassis_pid_position_structure;

/*云台YAW-PID*/
pid_t                  yaw_lock_pid_speed_structure;
pid_t                  yaw_pid_speed_structure;
pid_t                  yaw_lock_pid_position_structure;
pid_t                  yaw_follow_pid_position_structure;
pid_t                  yaw_head_pid_position_structure;

/*云台PIT-PID*/
pid_t                  pit_pid_speed_structure;
pid_t                  pit_pid_position_structure;

pid_t                  pit_pid_bmispeed_structure;
pid_t                  pit_pid_bmiposition_structure;


/*发射机构*/
/*波胆点击*/
motor_2006_t           motor_2006_PLUCK_structure;
motor_2006_base_info_t motor_2006_PLUCK_base_info;
motor_2006_info_t      motor_2006_PLUCK_info;

/*摩擦轮*/
motor_3508_t           motor_3508_FIRC_L_structure;
motor_3508_base_info_t motor_3508_FIRC_L_base_info;
motor_3508_info_t      motor_3508_FIRC_L_info; 

motor_3508_t           motor_3508_FIRC_R_structure;
motor_3508_base_info_t motor_3508_FIRC_R_base_info;
motor_3508_info_t      motor_3508_FIRC_R_info;

pid_t                  FIRC_L_speed_structure;
pid_t                  FIRC_R_speed_structure;
pid_t                  STRI_three_speed_structure;
pid_t                  STRI_one_speed_structure;

pid_t                  STRI_position_structure;

pid_t                  shoot_deal_done_structure;

pid_t                  shoot_judge_structure;

/*遥控器*/
rc_t                   rc_structure;
rc_info_t              rc_info_structure;
rc_base_info_t         rc_base_info_structure;

/*云台*/
gimbal_work_info_t     gimbal_work_info_structure;
s_pitch_work_info_t    s_pitch_work_info_structure;
s_yaw_work_info_t      s_yaw_work_info_structure;
gimbal_info_t          gimbal_info_structure;


gimbal_config_t gimbal_config_structure = 
{
	.watch_init_out_max = 2000,
	.watch_normal_out_max = 9000,
	.watch_init_speed = 2000,
	.watch_up_angle = 145000,
	.pitch_init_speed = 500,
	.pitch_up_angle = 26000,
	.yaw_check_angle = -3122,
	.pitch_check_angle = -7255,
	.yaw_middle_angle = 4724,
	.lock_cnt_max = 100,
	.pitch_angle_permit_max = 7300,//pitch角度最大值（下限 低头最大值）
	.pitch_angle_permit_min = 5800,//pitch角度最小值（上限 抬头最大值）
};

gimbal_t gimbal_structure = 
{
	.pitch = &motor_6020_PIT_structure,
	.yaw   = &motor_6020_YAW_structure,
	.work = &gimbal_work_info_structure,
	.pitch_work = &s_pitch_work_info_structure,
	.yaw_work = &s_yaw_work_info_structure,
	
	.config = &gimbal_config_structure,
	.info = &gimbal_info_structure,
};



/*整车状态*/
car_t             car_structure = {.f_times = 0};

/*陀螺仪*/
bmi_t             bmi_structure;

shoot_t           shoot_structure;

float sin_x,cos_x,tese,bbb;
#if TASK7
info_pack_t            TASK_info_pack;
CAN_TxHeaderTypeDef    TASK_Tx;
#endif
int16_t aaa = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_UART4_Init();
  MX_USART3_UART_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // 驱动层初始化
  	BMI_Init();
	DRIVER_Init();
	// 设备层初始化
	//DEV_Init();//  仿佛进入死循环
	// 结构体初始化

	
	SHOOT_INIT(&shoot_structure);
	VISION_INIT();
	//MOTOR_2006_INIT( &motor_2006_PLUCK_structure,&motor_2006_PLUCK_base_info,&motor_2006_PLUCK_info);
	car_structure.ctrl_mode = RC_CAR;
	car_structure.move_mode_status = offline_CAR;

	chassis_init( &chassis_structure);
	gimbal_init ( &gimbal_structure);
	
	
	rc_init(&rc_structure ,&rc_info_structure, &rc_base_info_structure);
	
	
	

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
