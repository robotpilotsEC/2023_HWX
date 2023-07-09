/**
  ************************************* Copyright ******************************   
  *                 (C) Copyright 2022, hwx, China, SZU.
  *                            N0   Rights  Reserved
  *                              
  *                   
  * @FileName   : rp_chassis.c   
  * @Version    : v1.1		
  * @Author     : hwx			
  * @Date       : 2023Äê7ÔÂ8ÈÕ         
  * @Description:    
  *
  *
  ******************************************************************************
 */

	
#ifndef __CIRCULAR_BUFFER_H
#define __CIRCULAR_BUFFER_H

#include "stm32f4xx_hal.h"
#include <stdlib.h>
#include <stdint.h>

#define BUFFER_SIZE 49

typedef struct {
    uint8_t *buffer;
    int front;
    int rear;
    int count;
} CircularBuffer;



void initializeBuffer(CircularBuffer *buffer);
void freeBuffer(CircularBuffer *buffer);
void insertData(CircularBuffer *buffer, int8_t data);
void CircularBuffer_init(void);
void printBuffer(const CircularBuffer *buffer,int8_t* desination);
void CircularBuffer_Insert(int8_t X,int8_t Y);


extern CircularBuffer Delta_X;
extern CircularBuffer Delta_Y;
#endif
