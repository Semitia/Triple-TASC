#ifndef __RINGBUFFER_H
#define __RINGBUFFER_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

typedef struct __RingBuffer_t
{
  //  uint16_t *buffer; // dynamically allocate memory
  uint16_t buffer[1];
  uint16_t sum; // current sum of datas in buffer
  int capacity; // capacity of buffer
  int head;     // head pointer, point to the oldest data which would be pop/read
  int tail;     // tail pointer, point to the loaction that new data would be write
} RingBuffer_t;

uint8_t initRing(RingBuffer_t *rb, int size);
void freeRing(RingBuffer_t *rb);
uint8_t isFull(RingBuffer_t *rb);
uint8_t isEmpty(RingBuffer_t *rb);
void pushBuffer(RingBuffer_t *rb, uint16_t data);
uint8_t popBuffer(RingBuffer_t *rb, uint16_t *data);
uint16_t getAve(RingBuffer_t *rb);
uint8_t readBuffer(RingBuffer_t *rb, uint16_t *data);

#endif // __RINGBUFFER_H
