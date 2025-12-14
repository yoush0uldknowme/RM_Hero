//
// Created by Pazfic on 2024/10/26.
//

#ifndef DHSENTRY_CHASSIS_DM_MOTOR_H
#define DHSENTRY_CHASSIS_DM_MOTOR_H

#include "struct_typedef.h"
#include "bsp_can.h"
//
// Created by Pazfic on 2024/10/26.
//

#ifndef DHSENTRY_CHASSIS_MOTOR_RELATED_PARAM_H
#define DHSENTRY_CHASSIS_MOTOR_RELATED_PARAM_H

/**
 * @brief 电机控制相关参数的定义
 */

/* -----------------电机PID参数-------------------- */

// 弧度转角度
#define RAD2DEG 57.29577951308238f
// 角度转弧度
#define DEG2RAD 0.01745329251994329576923690768489f
// 编码值转角度
#define ECD2DEG 0.0439453125f
// 编码值转弧度
#define ECD2RAD 0.000766990394f
// 角度转编码值
#define DEG2ECD 22.755555555f
// 弧度转编码值
#define RAD2ECD 1303.797316049191101f

// 各种死区
#define GYRO_MOTOR_ECD_DEAD_AREA 5.0f
// 陀螺时速度死区
#define GYRO_SPEED_DEAD_AREA 22
// 陀螺XY速度死区
#define GYRO_XY_SPEED_DEAD_AREA 3
// XY速度死区
#define STR_XY_SPEED_MINIMUM_LIMIT 2
//驱动轮速度死区
#define TRA_SPEED_DEAD_AREA 16

// 电机编码值与其对应角度
#define ECD360 8192
#define ECD180 4096
#define ECD90 2048
#define ECD45 1024

// 舵向电机PID参数
#define STR_MOTOR_SPEED_KP 20.0f
#define STR_MOTOR_SPEED_KI 5.0f
#define STR_MOTOR_SPEED_KD 10.f//100.0f
#define STR_MOTOR_SPEED_MAX_INTEGRAL 8000.0f
#define STR_MOTOR_SPEED_MAX_OUTPUT 28000.0f

#define STR_MOTOR_ANGLE_KP 0.3f
#define STR_MOTOR_ANGLE_KI 0.0f
#define STR_MOTOR_ANGLE_KD 3.f//3.0f
#define STR_MOTOR_ANGLE_MAX_INTEGRAL 30.0f
#define STR_MOTOR_ANGLE_MAX_OUTPUT 150.0f

// 驱动电机PID参数
#define TRA_MOTOR_SPEED_KP 10.0f
#define TRA_MOTOR_SPEED_KI 0.f//0.2f
#define TRA_MOTOR_SPEED_KD 0.f//3.0f
#define TRA_MOTOR_SPEED_MAX_INTEGRAL 4000.0f
#define TRA_MOTOR_SPEED_MAX_OUTPUT 11000.0f

// 大YAW电机PID参数
#define YAW_MOTOR_SPEED_KP 1.5f
#define YAW_MOTOR_SPEED_KI 0.0015f
#define YAW_MOTOR_SPEED_KD 7.0f
#define YAW_MOTOR_SPEED_MAX_INTEGRAL 10.0f
#define YAW_MOTOR_SPEED_MAX_OUTPUT 25.0f

#define YAW_MOTOR_ANGLE_KP 0.35f
#define YAW_MOTOR_ANGLE_KI 0.00f
#define YAW_MOTOR_ANGLE_KD 12.0f
#define YAW_MOTOR_ANGLE_MAX_INTEGRAL 1.0f
#define YAW_MOTOR_ANGLE_MAX_OUTPUT 15.0f

// DM电机指令长度
#define DM_CMD_LEN 8

// 舵向电机转向角度计算中间量，实际上就是陀螺时默认的角度
#define STR_LF_OFFSET 4511//431//4562//3339
#define STR_RF_OFFSET 966//980//3369
#define STR_RB_OFFSET 6506//1819//5390
#define STR_LB_OFFSET 4374//282//4458//7577

// PID前馈增益系数
#define PID_FORWARD_GAIN_1 0.1
#define PID_FORWARD_GAIN_2 0.1
#define PID_FORWARD_GAIN_3 0.1
#define PID_FORWARD_GAIN_4 0.1

#endif //DHSENTRY_CHASSIS_MOTOR_RELATED_PARAM_H

// 电机反馈电流mA转A
#define CURRENT_CONVERT 0.001220703125f
// 功率拟合曲线相关系数
#define POWER_R0 0.0021f        // 电机扭矩系数
#define POWER_K0 (-0.0303f)     // 电机电阻系数？？怎么是负的？？
#define POWER_P0 10.3523f       // 静息功率


// MIT控制帧相关常量定义
#define P_MIN 8192
#define P_MAX 0
#define V_MIN -45
#define V_MAX 45
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -18
#define T_MAX 18

// DM电机的CAN_ID
typedef enum {
    /* 云台电机 */
    CAN_DM_MOTOR_YAW_ID = 0x01,             //1     yaw轴
    CAN_DM_MOTOR_PITCH_ID = 0x02,           //1     pitch轴
}DM_Motor_CAN_ID_e;

// DM电机的MASTER_ID
typedef enum {
    /* 云台电机 */
    MASTER_DM_MOTOR_YAW_ID=0x05,            //1     yaw轴
    MASTER_DM_MOTOR_PITCH_ID=0x06,          //1     pitch轴
}DM_Motor_MASTER_ID_e;

typedef struct {

    int p_int, v_int, t_int;                // 整型的电机位置、速度、扭矩数据
    fp32 position, velocity, torque;        // 浮点型的电机位置、速度、扭矩数据
    uint8_t  Tx_Data[8];					// 数据发送存储
    uint8_t  RxData[8];						// 数据接收存储
    CAN_RxHeaderTypeDef Rx_pHeader;         // 接收帧头定义

    /* 自定义数据 */
    fp32 offset_position;                        // yaw轴电机角度设定值
    float kd;
}DM_Motor_t;

// extern DM_Motor_t YAW_Motor;
extern DM_Motor_t can_1;
extern fp32 DM_Velocity;

extern uint8_t DM_Enable_CMD[DM_CMD_LEN];
// DM电机失能
extern uint8_t DM_Disable_CMD[DM_CMD_LEN];
// DM电机保存零点
extern uint8_t DM_Save_ZeroPoint_CMD[DM_CMD_LEN];
// DM电机清错
extern uint8_t DM_Clear_Error_CMD[DM_CMD_LEN];

// extern int fp32_to_uint(fp32 x, fp32 x_min, fp32 x_max, int bits);

extern void DM_Send_CMD(CAN_TYPE hcan, DM_Motor_CAN_ID_e motor_id, uint8_t *cmd);
// extern void Gimbal_DM_Enable_Init(void);
extern void DM_MIT_Ctrl_Motor(CAN_HandleTypeDef *hcan, uint16_t id, fp32 _pos, fp32 _vel, fp32 _KP, fp32 _KD, fp32 _troq);
extern void DM_Motor_Decode(DM_Motor_t *motor, CAN_TYPE hcan, uint32_t can_id, uint8_t *data);

extern float DM_YAW_Nearest_Circle(float now_position, float target_position, float circle_position);
extern float DM_YAW_Nearest_Mid_Position(float now_position);
#endif //DHSENTRY_CHASSIS_DM_MOTOR_H