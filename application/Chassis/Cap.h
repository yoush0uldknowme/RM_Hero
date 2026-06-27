//
// Created by Gularx on 26-2-26.
//

#ifndef DJ_CAP_H
#define DJ_CAP_H

/*********************************************************************************************************
*                                              包含头文件
*********************************************************************************************************/
#include "Referee.h"

/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/
/* 任务开始空闲一段时间 */
#define CAP_TASK_INIT_TIME 357

/*********************************************************************************************************
*                                              枚举结构体定义
*********************************************************************************************************/
typedef enum {
    CAP_CONTROL=0X301,      // 控制帧
    CAP_FEEDBACK=0X302,      // 反馈帧
}CAP_SAFETY_ID_e;

typedef enum {
    HIGH=1,
    MID=2,
    OFF=3,
}CAP_LEVEL;


/**
 * @brief 控制帧结构体 (超电RX，主控TX)
 * @note  ID: 0x01，标准数据帧，2字节数据长度
 */
typedef struct {
    union {
        uint8_t data[8];            // 原始数据缓冲区（2字节）
        struct {
            uint8_t enable   : 8;   // [0-7]   Enable: 0x00=失能, 0x01=使能
            uint8_t power_set : 8;  // [8-15]  Power_set (W)
        } bits;
        uint16_t value;              // 16位组合值（可选）
    };
} Supercap_ControlFrame_t;


/**
 * @brief 反馈帧结构体 (超电TX，主控RX)
 * @note  ID: 0x02，标准数据帧，6字节数据长度
 * @note  发送频率：100Hz
 */
typedef struct {
    uint16_t power_set;             // 功率设定值 (×100)
    uint16_t current_power;         // 当前功率值 (×100)
    uint16_t voltage;               // 电容组电压值 (×100)
} Supercap_FeedbackFrame_t;

extern Supercap_FeedbackFrame_t feedback_frame;
extern uint8_t cap_level;
extern void Cap_Data_Get(uint32_t can_id, const uint8_t *rx_data);
fp32 cap_Get_Boosting_Power(fp32 _upper, fp32 _mid, fp32 _lower);

#endif //DJ_CAP_H
