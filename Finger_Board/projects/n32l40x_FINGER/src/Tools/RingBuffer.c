#include "RingBuffer.h"

/**
 * @brief 初始化缓冲区
 * @param rb 缓冲区指针
 * @param size 缓冲区大小
 */
uint8_t initRing(RingBuffer_t *rb, int size)
{
    //    rb->buffer = (uint16_t *)malloc(sizeof(uint16_t) * size);
    //    if (rb->buffer == NULL)
    //    {
    //        // 内存分配失败处理
    //        // fprintf(stderr, "Failed to allocate memory for circular buffer.\n");
    //        // exit(EXIT_FAILURE);
    //			return 0;
    //    }
    rb->sum = 0;
    rb->capacity = size;
    rb->head = 0;
    rb->tail = 0;
    return 1;
}

/**
 * @brief 释放缓冲区内存
 */
void freeRing(RingBuffer_t *rb)
{
    //    free(rb->buffer);  // 释放动态分配的内存
    //    rb->buffer = NULL; // 避免野指针
    return;
}

uint8_t isFull(RingBuffer_t *rb)
{
    return ((rb->tail + 1) % rb->capacity) == rb->head;
}

uint8_t isEmpty(RingBuffer_t *rb)
{
    return rb->head == rb->tail;
}

/**
 * @brief 将数据推入缓冲区
 * @param rb 缓冲区指针
 * @param data 数据
 */
void pushBuffer(RingBuffer_t *rb, uint16_t data)
{
    rb->sum -= rb->buffer[rb->tail];
    rb->sum += data;
    rb->buffer[rb->tail] = data;
    rb->tail = (rb->tail + 1) % rb->capacity;
    if (rb->tail == rb->head)
    {
        // 缓冲区已满，移动头指针以覆盖最旧的数据
        rb->head = (rb->head + 1) % rb->capacity;
    }
}

/**
 * @brief 从缓冲区中弹出数据
 * @param rb 缓冲区指针
 * @param data 数据
 */
uint8_t popBuffer(RingBuffer_t *rb, uint16_t *data)
{
    if (!isEmpty(rb))
    {
        *data = rb->buffer[rb->head];
        rb->sum -= rb->buffer[rb->head];
        rb->head = (rb->head + 1) % rb->capacity;
        return 1; // 成功读取
    }
    return 0; // 缓冲区为空
}

/**
 * @brief 获取缓冲区平均值
 * @param rb 缓冲区指针
 * @return 平均值
 */
uint16_t getAve(RingBuffer_t *rb)
{
    short num = rb->tail - rb->head;
    if (num <= 0)
    {
        num += rb->capacity;
    }
		num++;
    return rb->sum / num;
}

/**
 * @brief 读取缓冲区head指向数据
 */
uint8_t readBuffer(RingBuffer_t *rb, uint16_t *data)
{
    if (!isEmpty(rb))
    {
        *data = rb->buffer[rb->head];
        return 1; // 成功读取
    }
    return 0; // 缓冲区为空
}
