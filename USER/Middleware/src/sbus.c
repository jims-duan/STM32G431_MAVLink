#include "sbus.h"
#include <string.h>

SBUS_Decoder_t sbus_struct;
SBUS_Decoder_t sbus1_struct;
SBUS_Frame_t sbus_frame;
SBUS_Frame_t sbus_encode_frame_t;

// 解码矩阵（直接从原代码复制）
typedef struct {
    uint8_t byte;
    uint8_t rshift;
    uint8_t mask;
    uint8_t lshift;
} sbus_bit_pick_t;

static const sbus_bit_pick_t sbus_decoder[SBUS_INPUT_CHANNELS][3] = {
    /*  0 */ { { 0, 0, 0xff, 0}, { 1, 0, 0x07, 8}, { 0, 0, 0x00,  0} },
    /*  1 */ { { 1, 3, 0x1f, 0}, { 2, 0, 0x3f, 5}, { 0, 0, 0x00,  0} },
    /*  2 */ { { 2, 6, 0x03, 0}, { 3, 0, 0xff, 2}, { 4, 0, 0x01, 10} },
    /*  3 */ { { 4, 1, 0x7f, 0}, { 5, 0, 0x0f, 7}, { 0, 0, 0x00,  0} },
    /*  4 */ { { 5, 4, 0x0f, 0}, { 6, 0, 0x7f, 4}, { 0, 0, 0x00,  0} },
    /*  5 */ { { 6, 7, 0x01, 0}, { 7, 0, 0xff, 1}, { 8, 0, 0x03,  9} },
    /*  6 */ { { 8, 2, 0x3f, 0}, { 9, 0, 0x1f, 6}, { 0, 0, 0x00,  0} },
    /*  7 */ { { 9, 5, 0x07, 0}, {10, 0, 0xff, 3}, { 0, 0, 0x00,  0} },
    /*  8 */ { {11, 0, 0xff, 0}, {12, 0, 0x07, 8}, { 0, 0, 0x00,  0} },
    /*  9 */ { {12, 3, 0x1f, 0}, {13, 0, 0x3f, 5}, { 0, 0, 0x00,  0} },
    /* 10 */ { {13, 6, 0x03, 0}, {14, 0, 0xff, 2}, {15, 0, 0x01, 10} },
    /* 11 */ { {15, 1, 0x7f, 0}, {16, 0, 0x0f, 7}, { 0, 0, 0x00,  0} },
    /* 12 */ { {16, 4, 0x0f, 0}, {17, 0, 0x7f, 4}, { 0, 0, 0x00,  0} },
    /* 13 */ { {17, 7, 0x01, 0}, {18, 0, 0xff, 1}, {19, 0, 0x03,  9} },
    /* 14 */ { {19, 2, 0x3f, 0}, {20, 0, 0x1f, 6}, { 0, 0, 0x00,  0} },
    /* 15 */ { {20, 5, 0x07, 0}, {21, 0, 0xff, 3}, { 0, 0, 0x00,  0} }
};

void SBUS_Init(SBUS_Decoder_t *decoder) {
    decoder->rx_count = 0;
    decoder->state = SBUS_DECODE_STATE_DESYNC;
    decoder->frame_drops = 0;
    decoder->good_frames = 0;
    decoder->last_rx_time_ms = 0;
    memset(decoder->rx_buffer, 0, SBUS_FRAME_SIZE);
}

void SBUS_Reset(SBUS_Decoder_t *decoder) {
    decoder->rx_count = 0;
    decoder->state = SBUS_DECODE_STATE_DESYNC;
}

void SBUS_SetLastRxTime(SBUS_Decoder_t *decoder, uint32_t time_ms) {
    decoder->last_rx_time_ms = time_ms;
}

// 核心解码函数（直接从原代码 sbus_decode 修改）
static bool sbus_decode_frame(SBUS_Decoder_t *decoder, uint8_t *frame, SBUS_Frame_t *result, uint16_t max_values) {
    // 检查帧头
    if (frame[0] != SBUS_START_SYMBOL) {
        decoder->frame_drops++;
        decoder->state = SBUS_DECODE_STATE_DESYNC;
        return false;
    }
    
    // 原代码中的帧尾检查（可选，原代码用于判断SBUS1/SBUS2）
    // 我们简化处理，只检查帧尾是否为0x00
    if (frame[24] != 0x00) {
        // 不是标准SBUS1，但仍然尝试解码（兼容性）
        // 可以选择忽略或继续
    }
    
    // 计算通道数
    uint8_t chancount = (max_values > SBUS_INPUT_CHANNELS) ? SBUS_INPUT_CHANNELS : max_values;
    
    // 使用解码矩阵提取通道数据
    for (unsigned channel = 0; channel < chancount; channel++) {
        uint16_t value = 0;
        
        for (int pick = 0; pick < 3; pick++) {
            const sbus_bit_pick_t *decode = &sbus_decoder[channel][pick];
            
            if (decode->mask != 0) {
                uint16_t piece = frame[1 + decode->byte];
                piece >>= decode->rshift;
                piece &= decode->mask;
                piece <<= decode->lshift;
                value |= piece;
            }
        }
        
        // 使用浮点转换（与原代码完全一致）
        result->channels[channel] = (uint16_t)roundf(value * SBUS_SCALE_FACTOR + SBUS_SCALE_OFFSET);
    }
    
    result->channel_count = chancount;
    
    // 解码标志位（与原代码一致）
    uint8_t flags = frame[SBUS_FLAGS_BYTE];
    
    if (flags & (1 << SBUS_FAILSAFE_BIT)) {
        result->failsafe = true;
        result->frame_lost = true;
    } else if (flags & (1 << SBUS_FRAMELOST_BIT)) {
        result->failsafe = false;
        result->frame_lost = true;
    } else {
        result->failsafe = false;
        result->frame_lost = false;
    }
    
    result->frame_valid = true;
    decoder->good_frames++;
    
    return true;
}

// 逐字节解析（替代原代码的 sbus_parse）
bool SBUS_ParseByte(SBUS_Decoder_t *decoder, uint8_t data, SBUS_Frame_t *frame) {
    bool frame_ready = false;
    
    switch (decoder->state) {
    case SBUS_DECODE_STATE_DESYNC:
        // 寻找帧头
        if (data == SBUS_START_SYMBOL) {
            decoder->rx_buffer[0] = data;
            decoder->rx_count = 1;
            decoder->state = SBUS_DECODE_STATE_SYNC;
        }
        break;
        
    case SBUS_DECODE_STATE_SYNC:
        // 接收数据
        if (decoder->rx_count < SBUS_FRAME_SIZE) {
            decoder->rx_buffer[decoder->rx_count++] = data;
        }
        
        // 检查是否收到完整帧
        if (decoder->rx_count >= SBUS_FRAME_SIZE) {
            // 尝试解码
            if (sbus_decode_frame(decoder, decoder->rx_buffer, frame, SBUS_INPUT_CHANNELS)) {
                frame_ready = true;
            }
            
            // 重置状态
            decoder->state = SBUS_DECODE_STATE_DESYNC;
            decoder->rx_count = 0;
        }
        break;
    }
    
    return frame_ready;
}


// 核心合成函数：将通道数据和标志位打包成 SBUS 帧
void sbus_encode_frame(SBUS_Frame_t *input, uint8_t *frame) {
    // 初始化帧为0
    memset(frame, 0, SBUS_FRAME_SIZE);

    // 帧头
    frame[0] = SBUS_START_SYMBOL;

    // 使用解码矩阵的逆向思路打包通道数据
    for (unsigned channel = 0; channel < SBUS_INPUT_CHANNELS; channel++) {
        // 反向浮点转换回原始值
        uint16_t value = (uint16_t)roundf((input->channels[channel] - SBUS_SCALE_OFFSET) / SBUS_SCALE_FACTOR);

        for (int pick = 0; pick < 3; pick++) {
            const sbus_bit_pick_t *decode = &sbus_decoder[channel][pick];

            if (decode->mask != 0) {
                // 提取对应片段
                uint16_t piece = (value >> decode->lshift) & decode->mask;
                // 写入对应字节
                frame[1 + decode->byte] |= piece << decode->rshift;
            }
        }
    }

    // 设置标志位
    uint8_t flags = 0;
    if (input->failsafe) {
        flags |= (1 << SBUS_FAILSAFE_BIT);
        flags |= (1 << SBUS_FRAMELOST_BIT); // 解码函数中 failsafe 时 frame_lost 也置位
    } else if (input->frame_lost) {
        flags |= (1 << SBUS_FRAMELOST_BIT);
    }
    frame[SBUS_FLAGS_BYTE] = flags;

    // 帧尾（标准 SBUS1）
    frame[24] = 0x00;
}
