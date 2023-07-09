/**
  ************************************* Copyright ******************************   
  *                 (C) Copyright 2022, hwx, China, SZU.
  *                            N0   Rights  Reserved
  *                              
  *                   
  * @FileName   : rp_chassis.c   
  * @Version    : v1.1		
  * @Author     : hwx			
  * @Date       : 2023��2��28��         
  * @Description:    
  *
  *
  ******************************************************************************
 */
 /* Includes ------------------------------------------------------------------*/
#include "circular_buffer.h"
#include "judge_protocol.h"
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
CircularBuffer    Delta_X;
CircularBuffer    Delta_Y;
map_sentry_data_t Map_sentry_data;

/* Exported functions --------------------------------------------------------*/

void initializeBuffer(CircularBuffer *buffer) 
{
    buffer->buffer = (uint8_t *)malloc(BUFFER_SIZE * sizeof(uint8_t));
    buffer->front = 0;
    buffer->rear = -1;
    buffer->count = 0;
}


/**
  * @Name    freeBuffer
  * @brief   ��ջ�����
  * @param   None
  * @retval  
  * @author  HWX
  * @Date    2023-7-9
**/
void freeBuffer(CircularBuffer *buffer) 
{
    free(buffer->buffer);
}


/**
  * @Name    insertData
  * @brief   ��������
  * @param   None
  * @retval  
  * @author  HWX
  * @Date    2023-7-9
**/
void insertData(CircularBuffer *buffer, int8_t data) 
{
    buffer->rear = (buffer->rear + 1) % BUFFER_SIZE;
    if (buffer->count == BUFFER_SIZE) {
        buffer->front = (buffer->front + 1) % BUFFER_SIZE;
    } else {
        buffer->count++;
    }
    buffer->buffer[buffer->rear] = data;
}


/**
  * @Name    freeBuffer
  * @brief   �������
  * @param   None
  * @retval  
  * @author  HWX
  * @Date    2023-7-9
**/
void printBuffer(const CircularBuffer *buffer,int8_t* desination) 
{
    if (buffer->count > 0) {
        int index = buffer->front;
        for (int i = 0; i < buffer->count; i++) {
           desination[i]= buffer->buffer[index];
           index = (index + 1) % BUFFER_SIZE;
        }
    }
}

/**
  * @Name    CircularBuffer_Update
  * @brief   �ϲ㺯��
  * @param   None
  * @retval  
  * @author  HWX
  * @Date    2023-7-9
**/
void CircularBuffer_Update(int8_t X,int8_t Y)
{
	insertData(&Delta_X , X);
	insertData(&Delta_Y , Y);
	
	/*1s����һ�Σ����Ͳ������﷢��,����ֻ��Ҫ���͵�����д��*/
	printBuffer(&Delta_X,Map_sentry_data.delta_x);
	printBuffer(&Delta_Y,Map_sentry_data.delta_y);
	
}

