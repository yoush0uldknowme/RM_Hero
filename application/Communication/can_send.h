//
// Created by Gularx on 2024/12/10.
//

#ifndef INC_2024_HERO_THREE_CAN_SEND_H
#define INC_2024_HERO_THREE_CAN_SEND_H
#include "struct_typedef.h"
#include "bsp_can.h"

union ctrl_ch{
    uint8_t data[2];
    int16_t value;
};

union angle{
    uint8_t data[4];
    fp32 value;
};

union mode{
    uint8_t data[2];
    int16_t mode;
};

union referee{
    uint8_t data[4];
    fp32 referee;
};

union chassis_referee{
    uint8_t data[2];
    uint16_t referee;
};

union com{
    uint8_t data[2];
    int8_t value;
};

union ctrl_ch_s{
    uint8_t data;
    char value;
};
extern void Send_Control(int32_t W, int32_t A, int32_t S, int32_t D,
                         fp32 relative_angle, fp32 yaw_gyro, fp32 prone_vw_cmd);
extern void Send_CTRL(int32_t CTRL);
extern void Send_Chassis_Speed(int16_t ch1, int16_t ch0, int16_t ch2, int16_t mode1);
extern void Send_keyboard(int16_t gimbal_mode, int16_t launcher_mode, uint8_t s0,uint8_t s1, int16_t relative_up_angle);

extern void Send_pitch_down(fp32 absolute_angle);
extern void Send_referee(uint16_t buffer_energy,
                         uint16_t power_limit,
                         uint8_t robot_id,
                         uint16_t shooter_barrel_cooling_value);
extern void Send_bullet_speed(fp32 bullet_speed,
                              uint16_t shooter_heat,
                              uint8_t game_progress,
                              uint8_t shoot_seq,
                              uint16_t projectile_allowance_42mm);
extern void Send_UI_Telemetry(uint8_t up_pitch_temp, fp32 distance);

#endif //INC_2024_HERO_THREE_CAN_SEND_H
