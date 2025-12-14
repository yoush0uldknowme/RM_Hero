//
// Created by xhuanc on 2021/10/10.
//

#ifndef CHASSIS_H
#define CHASSIS_H

/*********************************************************************************************************
*                                              包含头文件
*********************************************************************************************************/
#include "Hero.h"
#include "PID.h"
#include "filter.h"
#include "DJI_Motor.h"
#include "key_board.h"
#include "power_limit.h"

/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/
/* 任务开始空闲一段时间 */
#define CHASSIS_TASK_INIT_TIME 357

/* 单位为ms 底盘任务运行周期 */
#define CHASSIS_PERIOD 1

/* 底盘旋转跟随PID */
#define CHASSIS_FOLLOW_GIMBAL_PID_KP        0.2f//0.4f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI        0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD        0.5f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT  0
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT   5.f

/* 底盘独立PID */
#define CHASSIS_3508_PID_KP             5.0f//6.0f
#define CHASSIS_3508_PID_KI             0.f
#define CHASSIS_3508_PID_KD             0.0f
#define CHASSIS_3508_PID_MAX_IOUT       5000.f
#define CHASSIS_3508_PID_MAX_OUT        20000.0f

/* 遥控器控制底盘移动 */
#define RC_TO_VX (MAX_CHASSIS_VX_SPEED/660)
#define RC_TO_VY (MAX_CHASSIS_VY_SPEED/660)
#define RC_TO_VW (MAX_CHASSIS_VW_SPEED/660)

/* 小陀螺的速度 */
#define CHASSIS_WZ_SET_SPEED 180

/* m3508转化成底盘速度(m/s)的比例，要根据轮子大小算 */
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f

/* 底盘机械信息 m */
#define WHEELBASE 0.47f         //H  轴距
#define WHEELTRACK 0.412f       //W  轮距
#define GIMBAL_OFFSET 0
#define PERIMETER 0.478f        //轮子周长
#define M3508_DECELE_RATIO (1.0f/19.0f) // M3508 减速比
#define M3508_MAX_RPM 8000

/* 机械信息 */
#define rotate_ratio_f  ((WHEELBASE+WHEELTRACK) / 2.0f - GIMBAL_OFFSET)   // / RADIAN_COEF;
#define rotate_ratio_b  ((WHEELBASE + WHEELTRACK) / 2.0f + GIMBAL_OFFSET) // / RADIAN_COEF;
#define wheel_rpm_ratio (60.0f / (PERIMETER * M3508_DECELE_RATIO))

/*********************************************************************************************************
*                                              枚举结构体定义
*********************************************************************************************************/
typedef struct {
    DJI_Motor_t motor_measure;     // 电机的真实信息

    fp32 rpm_set;                   //设定的转速值
    pid_type_def speed_p;           // 轮子速度环PID
    pid_type_def angle_p;           // 轮子角度环PID

    int16_t give_current;           //给定的电流值
}Motor_Chassis_t;

/* 底盘的信息集合 */
typedef struct {
    /* 电机信息 */
    Motor_Chassis_t motor_chassis[4];

    /* 轮子状态 */
    Chassis_Mode_e mode;
    Chassis_Mode_e last_mode;

    pid_type_def chassis_vw_pid;

    /* 滤波后的速度（用于平滑变化）是滤波数据 */
    first_order_filter_type_t vx_slow;
    first_order_filter_type_t vy_slow;
    first_order_filter_type_t vw_slow;

    ramp_function_source_t chassis_ramp[4];

    /* 最终速度（用于控制电机） */
    fp32 vx;
    fp32 vy;
    fp32 vw;
    /* 目标速度（原始控制命令） */
    fp32 vx_set;
    fp32 vy_set;
    fp32 vw_set;
    /* 速度偏移量（计算机或外部控制输入） */
    fp32 vx_pc;
    fp32 vy_pc;
    fp32 vw_pc;

    Chassis_Power_Limit_t chassis_power_limit;

    /* 要传给上板的C板pitch数据 */
    fp32 board_pitch;

    /* 从上板拿下来的云台数据 */
    fp32 board_yaw_angle;
} chassis_t;


/*********************************************************************************************************
*                                              对外允许调用文件
*********************************************************************************************************/
extern chassis_t chassis;
extern void Chassis_Can_Msg(uint32_t can_id, uint8_t *can_msg);
extern void Gimbal_to_Chassis_Can(uint32_t can_id, const uint8_t *rx_data);

#endif //CHASSIS_H