//
// Created by SyLin on 26-1-26.
//

#ifndef POWERLIMIT_STEERING_H
#define POWERLIMIT_STEERING_H

#include "PID.h"
#include "PowerLimiter.h"
#include "stdint.h"

#define CHASSIS_CURRENT_LIMIT_TOTAL 20000
#define CHASSIS_CURRENT_LIMIT_40W 7300
#define CHASSIS_POWER_BUFF 60

typedef struct
{
    fp32 limit_k;
    fp32 total_current;
    fp32 total_current_limit;

    uint8_t power_boost;        // 底盘功率boost模式的标志位
    fp32    chassis_power_limit;   // 底盘功率限制
    fp32    chassis_remain_energy; // 底盘功率
    fp32    chassis_power_buffer;  // 底盘缓冲能量
}Chassis_Power_Limit_Steering_t;


extern Limiter_t Steering_LF_Limiter;
extern Limiter_t Steering_RF_Limiter;
extern Limiter_t Steering_RB_Limiter;
extern Limiter_t Steering_LB_Limiter;
extern Limiter_t Drive_RF_Limiter;
extern Limiter_t Drive_LF_Limiter;
extern Limiter_t Drive_LB_Limiter;
extern Limiter_t Drive_RB_Limiter;
extern LimiterScheduler_t Steering_scheduler;
extern LimiterScheduler_t Drive_scheduler;
extern pid_type_def buffer_energy_cl;

extern void PowerLimit_Steering_init();
extern void Chassis_Power_Ctrl_Handler(int16_t steering[4], int16_t drive[4]);
extern void chassis_power_limit_steering(void);
#endif //POWERLIMIT_STEERING_H
