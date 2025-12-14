//
// Created by SR on 2024/12/13.
//

#ifndef INC_2024_HERO_THREE_POWER_LIMIT_H
#define INC_2024_HERO_THREE_POWER_LIMIT_H
/** 结构体 **/
#include "Hero.h"
#include "protocol_hero.h"

#define CHASSIS_POWER_K0  0.0019f   //转矩系数    //0.0019  //0.0019
#define CHASSIS_POWER_R0  0.0512f  //电机电阻    //0.0512   //0.00518
#define CHASSIS_POWER_P0  16.4817f   //底盘静息功率    //16.4817  //16.0710
#define CHASSIS_CURRENT_CONVERT (20/16384.0f)  //电机反馈电流毫安转安

typedef struct {
    fp32 power_buff;
    fp32 limit_k;
    fp32 total_current;
    fp32 total_current_limit;

    fp32 warning_power;
    fp32 warning_buff;

    fp32 K[4];
    fp32 M[4];
    fp32 k_c;
    fp32 predict_send_power;
    fp32 power_set;//最大功率限制
}Chassis_Power_Limit_t;

extern void chassis_power_limit(fp32 buffer_limit, fp32 k);


#endif //INC_2024_HERO_THREE_POWER_LIMIT_H
