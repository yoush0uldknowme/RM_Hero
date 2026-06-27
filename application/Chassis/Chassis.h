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
#include "PowerLimit_Steering.h"
#include "power_limit.h"

/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/
/* 任务开始空闲一段时间 */
#define CHASSIS_TASK_INIT_TIME 357

/* 单位为ms 底盘任务运行周期 */
#define CHASSIS_PERIOD 1

/* 底盘旋转跟随PID */
#define CHASSIS_FOLLOW_GIMBAL_PID_KP        0.06f//0.1f //0.20f//0.22f//0.1f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI        0.04f//0.05f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD        0.08f//0.2f//0.25f //1.5f//0.3f//0.5f//0.5f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT  0.1f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT   2.0f//1.9f//2.0f//1.8f

/*这套参数有点正常，但不多*/
// #define CHASSIS_FOLLOW_GIMBAL_PID_KP        0.20f//0.22f//0.1f
// #define CHASSIS_FOLLOW_GIMBAL_PID_KI        0.05f
// #define CHASSIS_FOLLOW_GIMBAL_PID_KD        1.5f//0.3f//0.5f//0.5f
// #define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT  0.1f
// #define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT   2.0f//1.9f//2.0f//1.8f


/*这套参数可以漂移甩尾*/
// #define CHASSIS_FOLLOW_GIMBAL_PID_KP        0.20f//0.22f//0.1f
// #define CHASSIS_FOLLOW_GIMBAL_PID_KI        0.05f
// #define CHASSIS_FOLLOW_GIMBAL_PID_KD        1.5f//0.3f//0.5f//0.5f
// #define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT  0.1f
// #define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT   1.9f//2.0f//1.8f

/* 底盘独立PID */

/*驱动轮速度环PID*/
/* 3508 */
#define CHASSIS_3508_SPEED_PID_KP             15.0f//5.0f
#define CHASSIS_3508_SPEED_PID_KI             0.05f
#define CHASSIS_3508_SPEED_PID_KD             1.0f//10.0f//15.0f
#define CHASSIS_3508_SPEED_PID_MAX_IOUT       5000.f
#define CHASSIS_3508_SPEED_PID_MAX_OUT        15000.0f

/*转向轮速度环PID*/
/* 6020 */
#define CHASSIS_6020_SPEED_PID_KP             15.0f//20.0f//15.0f//25.0f//6.0f
#define CHASSIS_6020_SPEED_PID_KI             0.0f//0.05f
#define CHASSIS_6020_SPEED_PID_KD             10.0f//10.0f
#define CHASSIS_6020_SPEED_PID_MAX_IOUT       5000.f
#define CHASSIS_6020_SPEED_PID_MAX_OUT        20000.0f

/* 转向轮角度环PID */
#define CHASSIS_6020_ANGLE_PID_KP             30.0f//45.f//100.f
#define CHASSIS_6020_ANGLE_PID_KI             0.05f//0.05f
#define CHASSIS_6020_ANGLE_PID_KD             5.0f//10.0f//50.0f//750.f
#define CHASSIS_6020_ANGLE_MAX_IOUT           5000.f
#define CHASSIS_6020_ANGLE_MAX_OUT            20000.0f

/* 遥控器控制底盘移动 */
#define RC_TO_VX (MAX_CHASSIS_VX_SPEED/660)
#define RC_TO_VY (MAX_CHASSIS_VY_SPEED/660)
#define RC_TO_VW (MAX_CHASSIS_VW_SPEED/660)
#define CHASSIS_SHIFT_SLOW_FACTOR 0.4f
#define CHASSIS_SHIFT_PC_SPEED_CHANGE 0.0035f
#define CHASSIS_RESET_HOLD_MS 1000U
#define CHASSIS_RESET_REQUEST_TIMEOUT_MS 100U


/*移动速度最大值*/
#define MAX_CHASSIS_AUTO_VX_SPEED 3.0f
#define MAX_CHASSIS_AUTO_VY_SPEED 3.0f
// #define MAX_CHASSIS_AUTO_VX_SPEED 4.0f
// #define MAX_CHASSIS_AUTO_VY_SPEED 4.0f

#define MAX_CHASSIS_AUTO_VXY_SPEED 2.0f

/* 小陀螺转速 */
#define CHASSIS_WZ_SET_SPEED 4.0f

#define WZ_MAX 4.f
#define WZ_MIN 2.f
#define AMP   ((WZ_MAX - WZ_MIN) / 2.0f)   // 2.5
#define BIAS  ((WZ_MAX + WZ_MIN) / 2.0f)   // 4.5
#define SPIN_MODE_HOLD_S 5
#define SPIN_SMOOTH_S 0.3f


// /* m3508转化成底盘速度(m/s)的比例，要根据轮子大小算 */
// #define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f

/*轮子参数*/
#define WHEEL_DIAMETER_M        (0.121f)                // 轮径121mm
#define WHEEL_RADIUS_M          (WHEEL_DIAMETER_M / 2.0f)//半径
#define WHEEL_CIRCUMFERENCE_M   (PI * WHEEL_DIAMETER_M) // 周长 ≈ 0.3801 m

// 减速比：电机转
#define MOTOR_REDUCTION_RATIO   (3591.0f /  187.0f)      // ≈ 19.2032

// 线速度（m/s）to 电机 RPM 的转换系数
#define LINEAR_SPEED_TO_RPM     (60.0f * MOTOR_REDUCTION_RATIO / WHEEL_CIRCUMFERENCE_M)

// 反向：电机 RPM to 线速度（m/s）
#define RPM_TO_LINEAR_SPEED     (WHEEL_CIRCUMFERENCE_M / (60.0f * MOTOR_REDUCTION_RATIO))

/* 底盘机械信息 m */
#define WHEELBASE 0.41212f         //H  轴距
#define WHEELTRACK 0.4046f       //W  轮距
#define GIMBAL_OFFSET 0
#define M3508_DECELE_RATIO (187.0f/3591.0f) // M3508 传动比
#define M3508_MAX_RPM 8000


/*********************************************************************************************************
*                                              枚举结构体定义
*********************************************************************************************************/
typedef enum {
    RF=0,
    LF=1,
    LB=2,
    RB=3,
}MOTOR_CHASSIS;

typedef enum {
    MODE_1,
    MODE_2,
    MODE_3,
}SPIN_MODE;

typedef struct {
    DJI_Motor_t motor_measure;     // 电机的真实信息

    fp32 rpm_set;                   //设定的转速值
    pid_type_def speed_p;           // 轮子速度环PID
    pid_type_def angle_p;           // 轮子角度环PID

    int16_t give_current;           //给定的电流值

    fp32 Direction;
}Motor_Chassis_t;

typedef struct {
    DJI_Motor_t motor_measure;     // 电机的真实信息

    fp32 rpm_set;                   //设定的转速值
    pid_type_def speed_p;           // 轮子速度环PID
    pid_type_def angle_p;           // 轮子角度环PID

    fp32 gyro_set;                  //转速设置

    int16_t give_current;           //给定的电流值


    fp32 relative_angle_set; //°    设定
    fp32 relative_angle_get; //°    获取

}Steering_Motor_Chassis_t;


/* 底盘的信息集合 */
typedef struct {
    /* 电机信息 */
    Motor_Chassis_t drive_motor_chassis[4];
    Steering_Motor_Chassis_t Steering_motor_chassis[4];


    Chassis_Power_Limit_t chassis_power_limit;//老功控，已弃用

    /* 功控信息 */
    Chassis_Power_Limit_Steering_t power_limit;

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

    fp32 vxy;

    /* 目标速度（原始控制命令） */
    fp32 vx_set;
    fp32 vy_set;
    fp32 vw_set;
    uint8_t shift_slow_mode;
    /* 速度偏移量（计算机或外部控制输入） */
    fp32 vx_pc;
    fp32 vy_pc;
    fp32 vw_pc;

    /* 要传给上板的C板pitch数据 */
    fp32 board_pitch;

    /* 从上板拿下来的云台数据 */
    fp32 board_yaw_angle;
} chassis_t;

/*********************************************************************************************************
*                                              对外允许调用文件
*********************************************************************************************************/
extern uint8_t cap_is_on;
extern uint8_t power_boost_on;
extern chassis_t chassis;
extern uint8_t board_reset_request;
extern void Chassis_Can_Msg(uint8_t can, uint32_t can_id, uint8_t *can_msg);
extern void Gimbal_to_Chassis_Can(uint32_t can_id, const uint8_t *rx_data);

#endif //CHASSIS_H
