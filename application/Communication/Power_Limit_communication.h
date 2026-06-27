//
// Created by SkLin on 26-1-28.
//

#ifndef POWER_LIMIT_COMMUNICATION_H
#define POWER_LIMIT_COMMUNICATION_H

#include <stdint.h>
#include "Hero.h"

#define RX_FRAME_START_BYTE     0xAA
#define RX_FRAME_END_BYTE       0xFF
#define RX_FRAME_TOTAL_SIZE     18
#define RX_BUFFER_SIZE          64   // 建议稍大，可容纳多帧

#pragma pack(push, 1)
typedef struct {
    uint8_t header;
    float voltage;
    float current;
    float power;
    float energy;
    uint8_t footer;
} rx_PowerLimit_t;
#pragma pack(pop)

// 全局变量声明
extern rx_PowerLimit_t rx_PowerLimit;
extern volatile bool_t rx_power_valid;  // 新增：标志位，表示数据是否有效/已更新
extern uint8_t usart1_Power[RX_BUFFER_SIZE];
// 函数声明
extern void PowerLimit_init(void);
extern bool_t PowerLimit_read_data(uint8_t *buffer, uint16_t len);

#endif // POWER_LIMIT_COMMUNICATION_H