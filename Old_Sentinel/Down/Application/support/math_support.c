/**
  ******************************************************************************
  * @file           : math_support.c\h
  * @brief          : 
  * @note           : 2022-3-3 15:27:58
  ******************************************************************************
  */
	
#include "math_support.h"
#include "math.h"

float lowpass(float X_last, float X_new, float K)
{
	return (X_last + (X_new - X_last) * K);
}

/**
  * @brief  字符串转数值（仅支持整数，支持负数）
  * @param  
  * @retval 
  */
int16_t str_to_num(uint8_t *str,uint16_t len)
{
	uint8_t minus_flag;
	uint16_t i;
	float result = 0;
	
	if(str[0] == '-')
	{
		minus_flag = 1;
		i = 1;
	}
	else
	{
		minus_flag = 0;
		i = 0;
	}
	
	for(;i<len;i++)
	{
		result += (str[i] - '0') * pow(10,len - i - 1);
	}
	
	if(minus_flag == 1)
	{
		result *= -1;
	}
	
	return result;
}

/**
  * @brief  字符串转数值（仅支持九位及九位以内整数，支持负数）
  * @param  
  * @retval 
  */
uint8_t num_to_str(int16_t num, uint8_t *str, uint16_t *len)
{
	uint8_t str_temp[10] = "0000000000";
	uint16_t str_index_temp = 0;
	uint16_t str_len_temp;
	int16_t num_temp = num;

	for(;num != 0;num /= 10)
	{
		str_temp[str_index_temp] = '0' + abs(num % 10);
		str_index_temp++;
	}

	if(str_index_temp > 9)
	{
		return 1;
	}

	/* 正负判断 */
	if(num_temp < 0)
	{
		str_temp[str_index_temp] = '-';
		str_index_temp++;
	}

	*len = str_index_temp;

	for(str_len_temp = str_index_temp; str_index_temp > 0; str_index_temp--)
	{
		str[str_len_temp - str_index_temp] = str_temp[str_index_temp-1];
	}

	return 0;
}
