#ifndef __LED_H
#define __LED_H

#include "stm32f4xx_hal.h"
#include "main.h"

#define LED_Blue_Off   HAL_GPIO_WritePin(LED_BLUE_GPIO_Port,   LED_BLUE_Pin,   GPIO_PIN_SET);
#define LED_Blue_On    HAL_GPIO_WritePin(LED_BLUE_GPIO_Port,   LED_BLUE_Pin,   GPIO_PIN_RESET);
#define LED_ORANGE_Off HAL_GPIO_WritePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin, GPIO_PIN_SET);
#define LED_ORANGE_On  HAL_GPIO_WritePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin, GPIO_PIN_RESET);
#define LED_RED_Off    HAL_GPIO_WritePin(LED_RED_GPIO_Port,    LED_RED_Pin,    GPIO_PIN_SET);
#define LED_RED_On     HAL_GPIO_WritePin(LED_RED_GPIO_Port,    LED_RED_Pin,    GPIO_PIN_RESET);
#define LED_GREEN_Off  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port,  LED_GREEN_Pin,  GPIO_PIN_SET);
#define LED_GREEN_On   HAL_GPIO_WritePin(LED_GREEN_GPIO_Port,  LED_GREEN_Pin,  GPIO_PIN_RESET);

/* µÆ×´Ì¬Ã¶¾Ù */
typedef enum
{
	OFF,
	LIGHT,
	INTERVAL,
	WAIT,
}LED_Statue;

typedef struct
{
	uint8_t statue;
	uint16_t config_times;
	uint16_t target_times;
	uint16_t measure_times;
	uint16_t cnt;
}Led;

extern Led led[4];

void LED_LIGHT(uint8_t LED, uint16_t times);
void led_work(void);

#endif
