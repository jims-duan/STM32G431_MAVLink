#ifndef _SBUS_H
#define _SBUS_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define SBUS_FRAME_SIZE        25
#define SBUS_START_SYMBOL      0x0F
#define SBUS_INPUT_CHANNELS    16
#define SBUS_FLAGS_BYTE        23
#define SBUS_FAILSAFE_BIT      3
#define SBUS_FRAMELOST_BIT     2

// 映射范围（与原代码一致）
#define SBUS_RANGE_MIN         200.0f
#define SBUS_RANGE_MAX         1800.0f
#define SBUS_TARGET_MIN        1000.0f
#define SBUS_TARGET_MAX        2000.0f

// 预计算缩放因子（编译时计算）
#define SBUS_SCALE_FACTOR ((SBUS_TARGET_MAX - SBUS_TARGET_MIN) / (SBUS_RANGE_MAX - SBUS_RANGE_MIN))
#define SBUS_SCALE_OFFSET (int)(SBUS_TARGET_MIN - (SBUS_SCALE_FACTOR * SBUS_RANGE_MIN + 0.5f))

// 解码状态机
typedef enum {
    SBUS_DECODE_STATE_DESYNC = 0,
    SBUS_DECODE_STATE_SYNC = 1
} SBUS_DecodeState_t;

// 解码器结构体
typedef struct {
    uint8_t rx_buffer[SBUS_FRAME_SIZE];
    uint8_t rx_count;
    SBUS_DecodeState_t state;
    uint32_t frame_drops;
    uint32_t good_frames;
    uint32_t last_rx_time_ms;
} SBUS_Decoder_t;
extern SBUS_Decoder_t sbus_struct;
extern SBUS_Decoder_t sbus1_struct;

// 解码结果
typedef struct {
    uint16_t channels[SBUS_INPUT_CHANNELS];
    uint8_t channel_count;
    bool failsafe;
    bool frame_lost;
    bool frame_valid;
} SBUS_Frame_t;
extern SBUS_Frame_t sbus_frame;
extern SBUS_Frame_t sbus_encode_frame_t;

// 函数声明
void SBUS_Init(SBUS_Decoder_t *decoder);
void SBUS_Reset(SBUS_Decoder_t *decoder);
bool SBUS_ParseByte(SBUS_Decoder_t *decoder, uint8_t data, SBUS_Frame_t *frame);
void SBUS_SetLastRxTime(SBUS_Decoder_t *decoder, uint32_t time_ms);
// void SBUS_Encode(uint16_t *channels, uint8_t *output_frame);
void sbus_encode_frame(SBUS_Frame_t *input, uint8_t *frame);

#endif
