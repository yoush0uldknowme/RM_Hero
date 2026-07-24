//
// Created by xhuanc on 2021/10/13.
//

#ifndef HERO_GIMBAL_H
#define HERO_GIMBAL_H

/*********************************************************************************************************
*                                              包含头文件
*********************************************************************************************************/
#include "Hero.h"
#include "Feedforward_PID.h"
#include "PID.h"
#include "filter.h"
#include "DJI_Motor.h"
#include "DM_motor.h"
#include "key_board.h"

/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/
/* 云台任务初始化时间 */
#define GIMBAL_TASK_INIT_TIME 1500

/* 云台任务运行周期 */
#define GIMBAL_PERIOD 1

/* pitch轴 PID */
// #define GIMBAL_PITCH_ANGLE_PID_KP           4.f//6.f//0.17f         //85.f//50
// #define GIMBAL_PITCH_ANGLE_PID_KI           0.02f//0.0015f        //0.2f
// #define GIMBAL_PITCH_ANGLE_PID_KD           45.f//50.f       //500.f//600.f//200.f//600.f//500.f
// #define GIMBAL_PITCH_ANGLE_MAX_IOUT         15.f
// #define GIMBAL_PITCH_ANGLE_MAX_OUT          35.f

#define GIMBAL_PITCH_ANGLE_PID_KP           5.0f
#define GIMBAL_PITCH_ANGLE_PID_KI           0.03f
#define GIMBAL_PITCH_ANGLE_PID_KD           85.0f
#define GIMBAL_PITCH_ANGLE_MAX_IOUT         30.f
#define GIMBAL_PITCH_ANGLE_MAX_OUT          50.f


/* yaw轴PID */
// #define GIMBAL_YAW_ANGLE_PID_KP             10.f//20.f
// #define GIMBAL_YAW_ANGLE_PID_KI             0.25f
// #define GIMBAL_YAW_ANGLE_PID_KD             250.f//490.0f
// #define GIMBAL_YAW_ANGLE_MAX_IOUT           80.f
// #define GIMBAL_YAW_ANGLE_MAX_OUT            3000.f

// #define GIMBAL_YAW_ANGLE_PID_KP     15.2f
// #define GIMBAL_YAW_ANGLE_PID_KI     0.0f
// #define GIMBAL_YAW_ANGLE_PID_KD     300.0f
// #define GIMBAL_YAW_ANGLE_MAX_IOUT   1.0f
// #define GIMBAL_YAW_ANGLE_MAX_OUT    45.0f


//不抖，偏软
// #define GIMBAL_YAW_ANGLE_PID_KP     8.2f //12.0f
// #define GIMBAL_YAW_ANGLE_PID_KI     0.005f//0.0f
// #define GIMBAL_YAW_ANGLE_PID_KD     88.f//110.f//160.0f //150.0f//200.0f//250.f
// #define GIMBAL_YAW_ANGLE_MAX_IOUT   2.0f
// #define GIMBAL_YAW_ANGLE_MAX_OUT    45.0f

// #define GIMBAL_YAW_ANGLE_PID_KP     9.0f//9.0f //12.0f
// #define GIMBAL_YAW_ANGLE_PID_KI     0.f//0.0f
// #define GIMBAL_YAW_ANGLE_PID_KD     70.0f//90.0f//110.f//160.0f //150.0f//200.0f//250.f
// #define GIMBAL_YAW_ANGLE_MAX_IOUT   2.0f
// #define GIMBAL_YAW_ANGLE_MAX_OUT    45.0f


// #define GIMBAL_YAW_ANGLE_PID_KP     15.0f//12.0f
// #define GIMBAL_YAW_ANGLE_PID_KI     0.0f//0.0f
// #define GIMBAL_YAW_ANGLE_PID_KD     650.0f//150.0f//200.0f//250.f
// #define GIMBAL_YAW_ANGLE_MAX_IOUT   2.0f
// #define GIMBAL_YAW_ANGLE_MAX_OUT    45.0f

#define GIMBAL_YAW_ANGLE_PID_KP     9.f//11.f//12.0f
#define GIMBAL_YAW_ANGLE_PID_KI     0.0f//0.0f
#define GIMBAL_YAW_ANGLE_PID_KD     160.0f//170.f //150.0f//200.0f//250.f
#define GIMBAL_YAW_ANGLE_MAX_IOUT   2.0f
#define GIMBAL_YAW_ANGLE_MAX_OUT    45.0f

#define GIMBAL_YAW_SPEED_PID_KP             20.f//200.f//140.f//100.0f
#define GIMBAL_YAW_SPEED_PID_KI             0.f//0.f
#define GIMBAL_YAW_SPEED_PID_KD             490.0f//100.f//0.f//100.0f
#define GIMBAL_YAW_SPEED_MAX_IOUT           80.f//3000.f
#define GIMBAL_YAW_SPEED_MAX_OUT            3000.f//20000.f

/* 对 pitch 进行动态限位 41.5*/
#define MAX_ABS_ANGLE 28.f//30.5
#define MIN_ABS_ANGLE (-8.5)
#define SPIN_MIN_ABS_ANGLE -3.0f

#define PITCH_POSITION_MAX 4975
#define PITCH_POSITION_MIN 4755

#define ANGLE_TO_POSITION_RATE 6.f//5.754189f//6.226415f//6.0416667f

#define PITCH_POS_CTRL_RATE 1.f//0.85f
#define PITCH_POS_FIX_RATE 3.f
#define PITCH_G_FF_RATE 0.f

/* ECD 回中值 */
#define PITCH_OFFSET_POSITION 4803
#define YAW_OFFSET_POSITION 179

#define PITCH_KP 3.f//60.0f//4.5f
#define PITCH_KD 1.0f//0.5f

#define YAW_KP 0.f
#define YAW_KD 0.005f

#define PITCH_POSITION_CHANGE_SPEED 10 //pitch位置改变系数
#define YAW_ANGLE_CHANGE_POSITION   22.7555555//yaw位置改变系数

#define PITCH_SPEED_CHANGE_SPEED    0.1f //pitch移动速度系数
#define YAW_SPEED_CHANGE_SPEED      0.01f //yaw移动速度系数
/*********************************************************************************************************
*                                              枚举结构体定义
*********************************************************************************************************/
/* 电机的获取值和计算值 */
typedef struct {
    DM_Motor_t motor_measure;    // 电机的真实信息

    pid_type_def speed_p;   //速度环 PID 控制参数
    pid_type_def angle_p;   //角度环 PID 控制参数
    fp32 gyro_set;          //转速设置
    int16_t give_current;   //设置电流值

    /*达妙电机*/
    fp32 DM_MIT_speed;
    fp32 DM_MIT_position;
    fp32 DM_MIT_torque;
    float DM_kd;
    float DM_kp;

    fp32 relative_angle_set; //°    设定
    fp32 relative_angle_get; //°    获取
    fp32 absolute_angle_set; //      rad
    fp32 absolute_angle_get; //      云台角
    fp32 absolute_angle_get_down;//  底盘角
    /* 云台底盘相对角度 */
    fp32 relative_up_down_set;
    fp32 relative_up_down_get;
}Motor_Gimbal_t;

typedef struct {
    /* 电机信息 */
    Motor_Gimbal_t yaw;
    Motor_Gimbal_t pitch;

    /* 云台状态信息 */
    Gimbal_Mode_e mode;
    Gimbal_Mode_e last_mode;

    /* 在云台任务发送给底盘的模式 */
    Chassis_Mode_e chassis_mode;

    /* 姿态角 */
    fp32 absolute_gyro_yaw;
    fp32 absolute_gyro_pitch;

    first_order_filter_type_t mouse_in_y;
    first_order_filter_type_t mouse_in_x;

    first_order_filter_type_t auto_pitch;
    first_order_filter_type_t auto_yaw[2];
    first_kalman_filter_t filter_autoYaw;

    first_order_filter_type_t filter_pitch_gyro_in;
    first_order_filter_type_t filter_yaw_gyro_in;

    first_order_filter_type_t pitch_first_order_set;
    first_order_filter_type_t pitch_current_first_order_set;

}gimbal_t;


/*********************************************************************************************************
*                                              对外允许调用文件
*********************************************************************************************************/
extern gimbal_t gimbal;
extern void Gimbal_Can_Msg(uint8_t can,uint32_t can_id, uint8_t *can_msg);
extern void Chassis_to_Gimbal_Can(uint32_t can_id, const uint8_t *rx_data);

#endif //HERO_GIMBAL_H
