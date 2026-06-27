//
// Created by Gularx on 2024/12/10.
//
#include "can_send.h"

#include "Gimbal.h"

/******************include******************/
static CAN_TxHeaderTypeDef  tx_message;
/* 0x110 0x111 0x117*/
void Send_Chassis_Speed(int16_t ch1, int16_t ch0, int16_t ch2, int16_t mode1){
    uint32_t send_mail_box;
    uint8_t Send_data[8];
    union ctrl_ch temp;
    union mode temp1;
    tx_message.StdId = 0x110;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;

    temp.value = ch1;
    Send_data[0] = temp.data[0];
    Send_data[1] = temp.data[1];
    temp.value = ch0;
    Send_data[2] = temp.data[0];
    Send_data[3] = temp.data[1];
    temp.value = ch2;
    Send_data[4] = temp.data[0];
    Send_data[5] = temp.data[1];
    temp1.mode = mode1;
    Send_data[6] = temp1.data[0];
    Send_data[7] = temp1.data[1];
    // while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);
    HAL_CAN_AddTxMessage(&hcan1, &tx_message, Send_data, &send_mail_box);
}

void Send_Control(int32_t W, int32_t A, int32_t S, int32_t D,
                  fp32 relative_angle, fp32 yaw_gyro, fp32 prone_vw_cmd) {
    uint32_t send_mail_box;
    uint8_t Send_data[8];
    union ctrl_ch temp;
    int16_t relative_angle_x100;
    int16_t yaw_gyro_x10;
    int16_t prone_vw_x1000;
    tx_message.StdId = 0x111;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;
    Send_data[0] = (uint8_t)(((W != 0) << 0) |
                             ((A != 0) << 1) |
                             ((S != 0) << 2) |
                             ((D != 0) << 3));
    relative_angle_x100 = (int16_t)(relative_angle * 100.0f);
    yaw_gyro_x10 = (int16_t)(yaw_gyro * 10.0f);
    temp.value = relative_angle_x100;
    Send_data[1] = temp.data[0];
    Send_data[2] = temp.data[1];
    temp.value = yaw_gyro_x10;
    Send_data[3] = temp.data[0];
    Send_data[4] = temp.data[1];
    prone_vw_x1000 = (int16_t)(prone_vw_cmd * 1000.0f);
    temp.value = prone_vw_x1000;
    Send_data[5] = temp.data[0];
    Send_data[6] = temp.data[1];
    Send_data[7] = 0;
    // while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);
    HAL_CAN_AddTxMessage(&hcan1, &tx_message, Send_data, &send_mail_box);
}

void Send_CTRL(int32_t CTRL) {
    uint32_t send_mail_box;
    uint8_t Send_data[6];
    tx_message.StdId = 0x112;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x06;
    Send_data[0] = (uint8_t)(CTRL & 0xFF);
    // while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);
    HAL_CAN_AddTxMessage(&hcan1, &tx_message, Send_data, &send_mail_box);
}


void Send_keyboard(int16_t gimbal_mode, int16_t launcher_mode, uint8_t s0,uint8_t s1, int16_t relative_up_angle) {
    uint32_t send_mail_box;
    union mode gimbal;
    union mode launcher;
    // union mode trigger;
    // union ctrl_ch_s;
    union mode angle1;
    tx_message.StdId = 0x117;
    uint8_t Send_data[8];
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;

    gimbal.mode = gimbal_mode;
    Send_data[0] = gimbal.data[0];
    Send_data[1] = gimbal.data[1];
    launcher.mode = launcher_mode;
    Send_data[2] = launcher.data[0];
    Send_data[3] = launcher.data[1];

    Send_data[4] = s0;
    Send_data[5] = s1;

    angle1.mode = relative_up_angle;
    Send_data[6] = angle1.data[0];
    Send_data[7] = angle1.data[1];

    // while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);
    HAL_CAN_AddTxMessage(&hcan1, &tx_message, Send_data, &send_mail_box);
}


/* 0x113 0x114 0x115*/
void Send_pitch_down(fp32 absolute_angle) {
    uint32_t send_mail_box;
    uint8_t Send_data[8];
    union angle angle;
    tx_message.StdId = 0x113;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    angle.value = absolute_angle;
    Send_data[0] = angle.data[0];
    Send_data[1] = angle.data[1];
    Send_data[2] = angle.data[2];
    Send_data[3] = angle.data[3];
    // while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);
    HAL_CAN_AddTxMessage(&hcan1, &tx_message, Send_data, &send_mail_box);
}

void Send_referee(uint16_t buffer_energy,
                  uint16_t power_limit,
                  uint8_t robot_id,
                  uint16_t shooter_barrel_heat_limit) {
    uint32_t send_mail_box;
    uint8_t Send_data[8];
    union chassis_referee temp;
    union chassis_referee limit;
    union chassis_referee cooling;
    tx_message.StdId = 0x114;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;

    temp.referee = buffer_energy;
    Send_data[0] = temp.data[0];
    Send_data[1] = temp.data[1];
    limit.referee = power_limit;
    Send_data[2] = limit.data[0];
    Send_data[3] = limit.data[1];
    Send_data[4] = robot_id;
    cooling.referee = shooter_barrel_heat_limit;
    Send_data[5] = cooling.data[0];
    Send_data[6] = cooling.data[1];
    Send_data[7] = 0U;
    // while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);
    HAL_CAN_AddTxMessage(&hcan1, &tx_message, Send_data, &send_mail_box);
}

void Send_bullet_speed(fp32 bullet_speed,
                       uint16_t shooter_heat,
                       uint8_t game_progress,
                       uint8_t shoot_seq,
                       uint16_t projectile_allowance_42mm) {
    uint32_t send_mail_box;
    uint8_t Send_data[8];
    uint16_t bullet_speed_x100;
    tx_message.StdId = 0x115;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;

    if (bullet_speed < 0.0f) {
        bullet_speed = 0.0f;
    }
    if (bullet_speed > 655.35f) {
        bullet_speed = 655.35f;
    }
    bullet_speed_x100 = (uint16_t)(bullet_speed * 100.0f + 0.5f);

    Send_data[0] = (uint8_t)(bullet_speed_x100 & 0xFFU);
    Send_data[1] = (uint8_t)(bullet_speed_x100 >> 8U);
    Send_data[2] = (uint8_t)(shooter_heat & 0xFFU);
    Send_data[3] = (uint8_t)(shooter_heat >> 8U);
    Send_data[4] = game_progress;
    Send_data[5] = shoot_seq;
    Send_data[6] = (uint8_t)(projectile_allowance_42mm & 0xFFU);
    Send_data[7] = (uint8_t)(projectile_allowance_42mm >> 8U);
    // while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);
    HAL_CAN_AddTxMessage(&hcan1, &tx_message, Send_data, &send_mail_box);
}

void Send_UI_Telemetry(uint8_t up_pitch_temp, fp32 distance) {
    uint32_t send_mail_box;
    uint8_t Send_data[8] = {0};
    union referee distance_data;
    tx_message.StdId = 0x116;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;

    distance_data.referee = distance;
    Send_data[0] = up_pitch_temp;
    Send_data[1] = distance_data.data[0];
    Send_data[2] = distance_data.data[1];
    Send_data[3] = distance_data.data[2];
    Send_data[4] = distance_data.data[3];

    HAL_CAN_AddTxMessage(&hcan1, &tx_message, Send_data, &send_mail_box);
}
