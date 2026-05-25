#include "ring_buffer.h"

RingBuffer_Structure u1TxRingBufferStruct;
RingBuffer_Structure USBRxRingBufferStruct;
RingBuffer_Structure u3RxRingBufferStruct;

// 初始化环形缓冲区
void RingBuff_Init(RingBuffer_Structure *ring)
{
    if (ring == NULL)
        return;

    ring->head = 0;
    ring->tail = 0;
    ring->size = 0;
    // memset(ring->buffer,0,RING_BUFFER_SIZE);
}

// 判断缓冲区是否为空(true:空 false:非空)
bool RingBuff_IsEmpty(RingBuffer_Structure *ring)
{
    return (ring->size == 0);
}

// 判断缓冲区是否为满(true:满 false:未满)
bool RingBuff_IsFull(RingBuffer_Structure *ring)
{
    return (ring->size == RING_BUFFER_SIZE);
}

// 获取当前数据量
uint16_t RingBuff_GetSize(RingBuffer_Structure *ring)
{
    return ring->size;
}

// 获取剩余空间
uint16_t RingBuff_GetSpace(RingBuffer_Structure *ring)
{
    return RING_BUFFER_SIZE - ring->size;
}

// 写入1字节
bool RingBuff_WriteByte(RingBuffer_Structure *ring, uint8_t data)
{
    if (RingBuff_IsFull(ring)) {
        return false;  // 缓冲区满，写入失败
    }
    
    ring->buffer[ring->head] = data;
    ring->head = (ring->head + 1) % RING_BUFFER_SIZE;
    ring->size++;
    
    return true;
}

// 读取1字节
bool RingBuff_ReadByte(RingBuffer_Structure *ring, uint8_t *data)
{
    if (RingBuff_IsEmpty(ring)) {
        return false;  // 缓冲区空，读取失败
    }
    
    *data = ring->buffer[ring->tail];
    ring->tail = (ring->tail + 1) % RING_BUFFER_SIZE;
    ring->size--;
    
    return true;
}

// 批量写入
// uint16_t RingBuff_WriteBytes(RingBuffer_Structure *ring, uint8_t *src, uint16_t len)
// {
//     uint16_t space = RingBuff_GetSpace(ring);
//     uint16_t write_len = (len < space) ? len : space;
    
//     for (uint16_t i = 0; i < write_len; i++) {
//         ring->buffer[ring->head] = src[i];
//         ring->head = (ring->head + 1) % RING_BUFFER_SIZE;
//         ring->size++;
//     }
    
//     return write_len;
// }
uint16_t RingBuff_WriteBytes(RingBuffer_Structure *ring, uint8_t *src, uint16_t len)
{
    uint16_t space = RingBuff_GetSpace(ring);
    uint16_t write_len = (len < space) ? len : space;
    
    // 计算连续空间大小
    uint16_t until_end = RING_BUFFER_SIZE - ring->head;
    
    if (write_len <= until_end)
    {
        // 一次拷贝完成
        memcpy(&ring->buffer[ring->head], src, write_len);
    }
    else
    {
        // 分两次拷贝（环形区域）
        memcpy(&ring->buffer[ring->head], src, until_end);
        memcpy(&ring->buffer[0], src + until_end, write_len - until_end);
    }
    
    ring->head = (ring->head + write_len) % RING_BUFFER_SIZE;
    ring->size += write_len;
    
    return write_len;
}

// 批量读取
// uint16_t RingBuff_ReadBytes(RingBuffer_Structure *ring, uint8_t *dst, uint16_t len)
// {
//     uint16_t read_len = (len < ring->size) ? len : ring->size;
    
//     for (uint16_t i = 0; i < read_len; i++) {
//         dst[i] = ring->buffer[ring->tail];
//         ring->tail = (ring->tail + 1) % RING_BUFFER_SIZE;
//         ring->size--;
//     }
    
//     return read_len;
// }
uint16_t RingBuff_ReadBytes(RingBuffer_Structure *ring, uint8_t *dst, uint16_t len)
{
    uint16_t read_len = (len < ring->size) ? len : ring->size;
    
    // 计算连续数据大小
    uint16_t until_end = RING_BUFFER_SIZE - ring->tail;
    
    if (read_len <= until_end) {
        // 一次拷贝完成
        memcpy(dst, &ring->buffer[ring->tail], read_len);
    } else {
        // 分两次拷贝（环形区域）
        memcpy(dst, &ring->buffer[ring->tail], until_end);
        memcpy(dst + until_end, &ring->buffer[0], read_len - until_end);
    }
    
    ring->tail = (ring->tail + read_len) % RING_BUFFER_SIZE;
    ring->size -= read_len;
    
    return read_len;
}

// 清空缓冲区
void RingBuff_Clear(RingBuffer_Structure *ring)
{
    ring->head = 0;
    ring->tail = 0;
    ring->size = 0;
    // memset(ring->buffer, 0, RING_BUFFER_SIZE);
}
