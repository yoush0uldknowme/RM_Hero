//
// Created by Pazfic on 2024/10/26.
//

#ifndef DHSENTRY_CHASSIS_DJI_MOTOR_H
#define DHSENTRY_CHASSIS_DJI_MOTOR_H

#include "struct_typedef.h"
#include "bsp_can.h"
#include "cmsis_os.h"

// DJI电机的CAN_ID
typedef enum {
    CAN_DJI_MOTOR_0x200_ID = 0x200, // C620/C610 id=1~4 (0x201~0x204)
    CAN_DJI_MOTOR_0x1FF_ID = 0x1FF, // C620/C610 id=5~8 (0x205~0x208);GM6020 id=1~4 (0x205~0x208)
    CAN_DJI_MOTOR_0x2FF_ID = 0x2FF, // GM6020 id=5~7 (0x209~0x20B)
}DJI_MOTOR_ID_e;

// DJI电机的CAN_ID
typedef enum {
    /* 底盘电机接收模块 */
    CAN_CHASSIS_MOTOR_RF_DRIVE=0x201,          //1     前右
    CAN_CHASSIS_MOTOR_LF_DRIVE=0x202,          //2     前左
    CAN_CHASSIS_MOTOR_LB_DRIVE=0x203,          //2     后左
    CAN_CHASSIS_MOTOR_RB_DRIVE=0x204,          //1     后右

    CAN_CHASSIS_MOTOR_RF_STEERING=0x205,       //1     前右
    CAN_CHASSIS_MOTOR_LF_STEERING=0x206,       //2     前左
    CAN_CHASSIS_MOTOR_LB_STEERING=0x207,       //2     后左
    CAN_CHASSIS_MOTOR_RB_STEERING=0x208,       //1     后右

    /* 拨盘电机接收模块(下板) */
    CAN_LAUNCHER_TRIGGER=0X204,                //2     拨盘

    /* 发射机构电机接收模块 */
    CAN_LAUNCHER_FIRE_FORWARD_LEFT=0x201,      //2     前左摩擦轮
    CAN_LAUNCHER_FIRE_FORWARD_RIGHT=0X202,     //2     前右摩擦轮
    CAN_LAUNCHER_FIRE_BACK_LEFT=0X203,         //2     后左摩擦轮
    CAN_LAUNCHER_FIRE_BACK_RIGHT=0X204,        //2     后右摩擦轮

}DJI_CAN_ID_e;

//电机的数据
typedef struct {
    /* 实际电机反馈数据 */
    int16_t last_ecd;       //上一次的电机编码器计数值
    uint16_t ecd;           //转子机械角度, 电机编码器计数值
    int16_t speed_rpm;      //转子转速, 电机转速（每分钟转数，RPM）
    int16_t given_current;  //实际扭矩电流
    uint8_t temperate;      //电机温度

    /* 自定义数据 */
    int32_t total_ecd;      //电机旋转的总编码器数值
    uint16_t offset_ecd;    //电机的校准编码值
    int32_t round_cnt;      //电机旋转的总圈数
} DJI_Motor_t;

void DJI_Motor_Decode(DJI_Motor_t *motor, uint8_t *data);
void DJI_Round_Count(DJI_Motor_t *motor);
fp32 DJI_Encoder_Limit(int16_t ecd);
HAL_StatusTypeDef DJI_Send_Motor_Mapping(CAN_TYPE hcan, uint32_t can_id, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
fp32 Motor_Ecd_To_Angle_Change(uint16_t ecd, uint16_t offset_ecd);


#endif //DHSENTRY_CHASSIS_DJI_MOTOR_H
