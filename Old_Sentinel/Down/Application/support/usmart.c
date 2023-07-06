/**
  ******************************************************************************
  * @file           : usmart.c\h
  * @brief          : 
  * @note           : 2022-3-3 14:21:23
  ******************************************************************************
  */
	
#include "usmart.h"
#include "string.h"
#include "math_support.h"
#include "launcher.h"

extern UART_HandleTypeDef huart1;

uint8_t usmart_rxbuf[40];
uint8_t usmart_txbuf[40];

usmart_info_t usmart_info;
extern usmart_cmd_list_t usmart_cmd_list[];
uint8_t usmart_fname[20];
float usmart_fpara[10];
usmart_t usmart = 
{
	.info = &usmart_info,
	.list = usmart_cmd_list,
	.cmd_list_sum = sizeof(*usmart_cmd_list)/sizeof(usmart_cmd_list_t),
	.fname = usmart_fname,
	.fpara = usmart_fpara,
};

/**
  * @brief  usmart 初始化
  * @param  
  * @retval 
  */
void usmart_init(usmart_t *usmart)
{
	usmart_info_init(usmart->info);
}

/**
  * @brief  usmart 信息初始化
  * @param  
  * @retval 
  */
void usmart_info_init(usmart_info_t *info)
{
	info->interrupt = 0;
}

/**
  * @brief  usmart 串口中断更新
  * @param  
  * @retval 
  */
void usmart_update(usmart_t *usmart,uint8_t *rxbuf)
{
	usmart->info->interrupt = 1;
	memcpy(usmart_rxbuf,rxbuf,40);
}

/**
  * @brief  usmart 扫描任务
  * @param  
  * @retval 
  */
void usmart_scan_task(usmart_t *usmart)
{
	usmart_info_t *info = usmart->info;
	float sta;
	
	/* 接收到命令 */
	if(info->interrupt == 1)
	{
		sta = usmart_get_num(usmart_rxbuf);
		if(sta != -1)
		{
			launcher_motor_speed_change(sta);
			usmart_send_data("OK",2);
		}
		else 
		{
			usmart_send_data("fail",4);
		}
		info->interrupt = 0;
	}
}

/**
  * @brief  usmart 命令响应
  * @param  
  * @retval 
  */
uint8_t usmart_cmd_res(usmart_t *usmart)
{
	uint8_t sta;
	
	/* 获取函数名 */
	sta = usmart_get_fname(usmart_rxbuf, usmart->fname);
	if(sta != usmart_ok)
	{
		return sta;
	}
	
	/* 获取函数参数及参数个数 */
	sta = usmart_get_fpara(usmart_rxbuf, usmart->fpara, &usmart->fpara_sum);
	if(sta != usmart_ok)
	{
		return sta;
	}
	
	/*  */
	
	return usmart_ok;
}

/**
  * @brief  usmart 获取函数名
  * @param  
  * @retval 
  */
uint8_t usmart_get_fname(uint8_t *str, uint8_t *name)
{
	uint8_t *strtemp = str;
	uint8_t *nametemp = name;

	for(; *strtemp!='\0'; strtemp++)
	{
		if(*strtemp == '(')
		{
			return usmart_ok;
		}
		else
		{
			*nametemp = *strtemp;
		}
	}
	return usmart_name_err;
}

/**
  * @brief  usmart 获取函数参数及参数个数
  * @param  
  * @retval 
  */
uint8_t usmart_get_fpara(uint8_t *str, float *para, uint16_t *sum)
{
	uint8_t *strtemp = str;  //源字符串地址
	float *paratemp = para;  //参数地址
	uint8_t aparatemp[20];  //单个参数缓存
	uint8_t apara_len = 0;  //单个参数长度
	uint16_t para_sum = 0;  //参数个数
	
	/* 寻找参数起始地址 */
	for(; *strtemp!='('; strtemp++);
	strtemp++;
	
	for(; *strtemp!='\0'; strtemp++)
	{
		if(*strtemp == ' ')
		{
		}
		else if(*strtemp == ',')
		{
			*paratemp = str_to_num(aparatemp,apara_len);
			paratemp++;
			para_sum++;
			apara_len = 0;
		}
		else if(*strtemp == ')')
		{
			*paratemp = str_to_num(aparatemp,apara_len);
			para_sum++;
			*sum = para_sum;
			return usmart_ok;
		}
		else if(((*strtemp >= '0')&&(*strtemp <= '9'))||(*strtemp == '-'))
		{
			aparatemp[apara_len] = *strtemp;
			apara_len++;
		}
	}
	
	return usmart_para_err;
}

float usmart_get_num(uint8_t *str)
{
	uint8_t *strtemp = str;  //源字符串地址
	uint8_t aparatemp[20];  //单个参数缓存
	uint8_t apara_len = 0;  //单个参数长度
	
	for(; *strtemp!='\0'; strtemp++)
	{
		if(*strtemp == ' ')
		{
		}
		else if((*strtemp >= '0')&&(*strtemp <= '9'))
		{
			aparatemp[apara_len] = *strtemp;
			apara_len++;
		}
		else 
		{
			return -1;
		}
	}
	
	return str_to_num(aparatemp, apara_len);
}

void usmart_send_data(uint8_t *str, uint16_t len)
{
	HAL_UART_Transmit_DMA(&huart1,str,len);
}
