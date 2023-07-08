#include "circular_buffer.h"

CircularBuffer Delta_X;
CircularBuffer Delta_Y;

void initializeBuffer(CircularBuffer *buffer) 
{
    buffer->buffer = (uint8_t *)malloc(BUFFER_SIZE * sizeof(uint8_t));
    buffer->front = 0;
    buffer->rear = -1;
    buffer->count = 0;
}


void freeBuffer(CircularBuffer *buffer) 
{
    free(buffer->buffer);
}

void insertData(CircularBuffer *buffer, uint8_t data) 
{
    buffer->rear = (buffer->rear + 1) % BUFFER_SIZE;
    if (buffer->count == BUFFER_SIZE) {
        buffer->front = (buffer->front + 1) % BUFFER_SIZE;
    } else {
        buffer->count++;
    }
    buffer->buffer[buffer->rear] = data;
}

void CircularBuffer_Insert(uint8_t X,uint8_t Y)
{
	insertData(&Delta_X , X);
	insertData(&Delta_Y , Y);
	
}


