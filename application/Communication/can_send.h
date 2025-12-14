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

extern void Send_Control(int32_t W, int32_t A, int32_t S, int32_t D, fp32 relative_angle);
extern void Send_CTRL(int32_t CTRL);
extern void Send_Chassis_Speed(int16_t ch1, int16_t ch0, int16_t ch2, int16_t mode1);
extern void Send_keyboard(int16_t gimbal_mode, int16_t launcher_mode, int16_t trigger_mode, int16_t relative_up_angle);

extern void Send_pitch_down(fp32 absolute_angle);
extern void Send_referee(uint16_t buffer_energy, uint16_t power_limit, uint8_t robot_id);
extern void Send_bullet_speed(fp32 bullet_speed, fp32 shooter_heat);

#endif //INC_2024_HERO_THREE_CAN_SEND_H
