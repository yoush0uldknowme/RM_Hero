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
#define GIMBAL_UP_PITCH_ANGLE_PID_KP             1.8f//0.17f         //85.f//50
#define GIMBAL_UP_PITCH_ANGLE_PID_KI             0.0f//0.0015f        //0.2f
#define GIMBAL_UP_PITCH_ANGLE_PID_KD             1.0f//2.5f       //500.f//600.f//200.f//600.f//500.f
#define GIMBAL_UP_PITCH_ANGLE_MAX_IOUT           20.f
#define GIMBAL_UP_PITCH_ANGLE_MAX_OUT            120.f

#define GIMBAL_UP_PITCH_FIX_KP                   2.4f
#define GIMBAL_UP_PITCH_FIX_KI                   0.0010f
#define GIMBAL_UP_PITCH_FIX_KD                   0.0f
#define GIMBAL_UP_PITCH_FIX_IOUT                 2.0f
#define GIMBAL_UP_PITCH_FIX_MAX_OUT              8.0f
#define GIMBAL_UP_PITCH_FIX_FILTER_NUM           45.0f
#define PITCH_FIX_DEADBAND                       0.12f
#define PITCH_LIMIT_GUARD                        0.3f
#define UP_PITCH_ANGLE_FIX_ENABLE                0
#define UP_PITCH_INPUT_FILTER_NUM                12.0f
#define UP_PITCH_FIX_INPUT_DEADBAND              0.015f
#define UP_PITCH_FIX_GYRO_DEADBAND               4.0f
#define UP_PITCH_BACKLASH_INPUT_DEADBAND         0.006f
#define UP_PITCH_UP_BACKLASH_COMP_POS            22.0f
#define UP_PITCH_BACKLASH_MIN_ANGLE              2.0f

#define GIMBAL_UP_PITCH_SPEED_PID_KP             5.f//70.0f
#define GIMBAL_UP_PITCH_SPEED_PID_KI             0.0f
#define GIMBAL_UP_PITCH_SPEED_PID_KD             500.f//800.f//500.f//50.0f
#define GIMBAL_UP_PITCH_SPEED_MAX_IOUT           20.f
#define GIMBAL_UP_PITCH_SPEED_MAX_OUT            120.f

#define GIMBAL_DOWN_PITCH_ANGLE_PID_KP           6.f//0.17f         //85.f//50
#define GIMBAL_DOWN_PITCH_ANGLE_PID_KI           0.002f//0.0015f        //0.2f
#define GIMBAL_DOWN_PITCH_ANGLE_PID_KD           50.f//2.5f       //500.f//600.f//200.f//600.f//500.f
#define GIMBAL_DOWN_PITCH_ANGLE_MAX_IOUT         20.f
#define GIMBAL_DOWN_PITCH_ANGLE_MAX_OUT          120.f

#define GIMBAL_DOWN_PITCH_SPEED_PID_KP           5.f//70.0f
#define GIMBAL_DOWN_PITCH_SPEED_PID_KI           0.0f
#define GIMBAL_DOWN_PITCH_SPEED_PID_KD           500.f//800.f//500.f//50.0f
#define GIMBAL_DOWN_PITCH_SPEED_MAX_IOUT         20.f
#define GIMBAL_DOWN_PITCH_SPEED_MAX_OUT          120.f

/* yaw轴 PID */
#define GIMBAL_YAW_ANGLE_PID_KP     15.0f//12.0f
#define GIMBAL_YAW_ANGLE_PID_KI     0.0f//0.0f
#define GIMBAL_YAW_ANGLE_PID_KD     70.0f//150.0f//200.0f//250.f
#define GIMBAL_YAW_ANGLE_MAX_IOUT   2.0f
#define GIMBAL_YAW_ANGLE_MAX_OUT    45.0f

#define GIMBAL_YAW_SPEED_PID_KP             2.f//200.f//140.f//100.0f
#define GIMBAL_YAW_SPEED_PID_KI             0.f//0.f
#define GIMBAL_YAW_SPEED_PID_KD             10.0f//100.f//0.f//100.0f
#define GIMBAL_YAW_SPEED_MAX_IOUT           80.f//3000.f
#define GIMBAL_YAW_SPEED_MAX_OUT            3000.f//20000.f

/* yaw轴前馈系数 */
#define GIMBAL_YAW_FEEDFORWARD_KF              0.2f


/* Low-cost adaptive yaw impedance controller. */
#define GIMBAL_YAW_SERVO_DT                    0.001f
#define GIMBAL_YAW_PLAN_KP                     4.8f
#define GIMBAL_YAW_PLAN_SET_FF                 0.23f
#define GIMBAL_YAW_PLAN_SMALL_ERR              6.0f
#define GIMBAL_YAW_PLAN_SMALL_FF               0.080f
#define GIMBAL_YAW_PLAN_SMALL_ACCEL_SCALE      0.75f
#define GIMBAL_YAW_PLAN_REVERSE_FF             0.11f
#define GIMBAL_YAW_PLAN_REVERSE_ACCEL_SCALE    0.55f
#define GIMBAL_YAW_PLAN_VEL_LPF                0.18f
#define GIMBAL_YAW_PLAN_MAX_VEL                190.0f
#define GIMBAL_YAW_PLAN_MAX_ACCEL              24.0f
#define GIMBAL_DOWN_YAW_PLAN_KP                4.20f
#define GIMBAL_DOWN_YAW_PLAN_SET_FF            0.000f
#define GIMBAL_DOWN_YAW_PLAN_SMALL_FF          0.000f
#define GIMBAL_DOWN_YAW_PLAN_REVERSE_FF        0.000f
#define GIMBAL_DOWN_YAW_PLAN_VEL_LPF           0.105f
#define GIMBAL_DOWN_YAW_PLAN_MAX_VEL           160.0f
#define GIMBAL_DOWN_YAW_PLAN_MAX_ACCEL         8.50f
#define GIMBAL_YAW_IMP_KP                      0.96f
#define GIMBAL_YAW_IMP_KD                      0.040f
#define GIMBAL_YAW_IMP_KA                      0.000f
#define GIMBAL_YAW_IMP_KF                      0.12f
#define GIMBAL_YAW_IMP_FRICTION_ERR            2.0f
#define GIMBAL_YAW_IMP_KV_ASSIST               0.006f
#define GIMBAL_YAW_IMP_ASSIST_ERR              10.0f
#define GIMBAL_YAW_IMP_MAX_ASSIST_TORQUE       0.65f
#define GIMBAL_YAW_IMP_KB                      0.0180f
#define GIMBAL_YAW_IMP_BRAKE_ERR               3.0f
#define GIMBAL_YAW_IMP_MAX_BRAKE_TORQUE        1.60f
#define GIMBAL_YAW_IMP_MAX_TORQUE_STEP         0.50f
#define GIMBAL_YAW_IMP_SETTLE_KD               0.000f
#define GIMBAL_YAW_IMP_SETTLE_VEL_DEADBAND     4.0f
#define GIMBAL_YAW_IMP_MAX_SETTLE_TORQUE       0.80f
#define GIMBAL_YAW_IMP_FRICTION_MIN_VEL        6.0f
#define GIMBAL_YAW_IMP_MAX_TORQUE              8.0f
#define GIMBAL_YAW_IMP_FULL_MAX_TORQUE         10.6f
#define GIMBAL_YAW_IMP_MAX_VEL_ERR             110.0f
#define GIMBAL_YAW_IMP_MAX_ACC_FF              25.0f
#define GIMBAL_YAW_IMP_DM_KD                   0.05f
#define GIMBAL_DOWN_YAW_IMP_DM_KD              0.05f
#define GIMBAL_YAW_IMP_GYRO_LPF                0.08f
#define GIMBAL_YAW_IMP_DOWN_BIAS               1.00f
#define GIMBAL_YAW_IMP_UP_BIAS                 1.00f
#define GIMBAL_DOWN_YAW_IMP_KP                 0.78f
#define GIMBAL_DOWN_YAW_IMP_KD                 0.035f
#define GIMBAL_DOWN_YAW_IMP_KB                 0.020f
#define GIMBAL_DOWN_YAW_IMP_MAX_BRAKE_TORQUE   1.20f
#define GIMBAL_DOWN_YAW_IMP_MAX_TORQUE         5.6f
#define GIMBAL_DOWN_YAW_IMP_MAX_TORQUE_STEP    0.07f
#define GIMBAL_DOWN_YAW_IMP_MAX_VEL_ERR        45.0f
#define GIMBAL_DOWN_YAW_IMP_GYRO_LPF           0.045f
#define GIMBAL_YAW_RC_DEAD_BAND                25
#define GIMBAL_YAW_MOUSE_DEAD_BAND             4.0f
#define GIMBAL_YAW_LOAD_ADAPT_MIN              0.78f
#define GIMBAL_YAW_LOAD_ADAPT_MAX              1.80f
#define GIMBAL_YAW_LOAD_ADAPT_INC              0.0010f
#define GIMBAL_YAW_LOAD_ADAPT_DEC              0.995f
#define GIMBAL_YAW_LOAD_ADAPT_TRACK_ERR        6.0f
#define GIMBAL_YAW_LOAD_ADAPT_SPEED_ERR        18.0f
#define GIMBAL_YAW_LOAD_ADAPT_OSC_GYRO         45.0f
#define GIMBAL_YAW_LOAD_ADAPT_PERIOD           5U
#define GIMBAL_DOWN_YAW_FOLLOW_SLOW_ERR        0.78f
#define GIMBAL_DOWN_YAW_FOLLOW_STOP_ERR        0.98f
#define GIMBAL_DOWN_YAW_FOLLOW_MIN_SCALE       0.20f
#define GIMBAL_DOWN_YAW_FOLLOW_FF_MAX          0.0f
#define GIMBAL_DOWN_YAW_FOLLOW_FF_STEP         0.0f
#define GIMBAL_DOWN_YAW_FOLLOW_FF_LPF          0.02f
#define GIMBAL_DOWN_YAW_INPUT_MAX_DELTA        0.0550f
#define GIMBAL_DOWN_YAW_INPUT_MAX_STEP         0.00900f
#define GIMBAL_DOWN_YAW_INPUT_DECAY_STEP       0.010f
#define GIMBAL_DOWN_YAW_INPUT_HARD_ERR         1.05f
#define GIMBAL_DOWN_YAW_INPUT_HARD_DECAY       0.018f
#define GIMBAL_PRONE_ENTER_YAW_ERR             6.0f
#define GIMBAL_PRONE_ENTER_STABLE_MS           120U
#define GIMBAL_PRONE_VW_CMD_MAX                2.4f
#define GIMBAL_PRONE_RC_TO_VW                  0.0024f
#define GIMBAL_PRONE_MOUSE_TO_VW               0.0080f
#define CRAZY_CYCLE_EXIT_YAW_ERR               4.0f
#define CRAZY_CYCLE_EXIT_STABLE_MS             120U

/* Ammo/load compensation.
 * Normal match: load ratio is estimated from referee shoot events.
 * Bench test: set TEST_ENABLE = 1 and edit TEST_RATIO, where 1.0 = full ammo, 0.0 = empty ammo.
 * If TEST_OUTSIDE_MATCH_ONLY = 1, bench test is automatically ignored during countdown/running.
 */
#define GIMBAL_YAW_AMMO_ENABLE                 1
#define GIMBAL_YAW_AMMO_PROGRESS_COUNTDOWN     3U
#define GIMBAL_YAW_AMMO_PROGRESS_RUNNING       4U
#define GIMBAL_YAW_AMMO_TEST_ENABLE            0
#define GIMBAL_YAW_AMMO_TEST_OUTSIDE_MATCH_ONLY 1
#define GIMBAL_YAW_AMMO_TEST_RATIO             0.5f
#define GIMBAL_YAW_AMMO_TEST_GAME_PROGRESS     GIMBAL_YAW_AMMO_PROGRESS_RUNNING
#define GIMBAL_YAW_AMMO_TEST_SHOOT_SEQ         0U
#define GIMBAL_YAW_AMMO_SHOT_DEC               0.025f
#define GIMBAL_YAW_AMMO_FULL_DRIVE_SCALE       1.10f
#define GIMBAL_YAW_AMMO_EMPTY_DRIVE_SCALE      0.98f
#define GIMBAL_YAW_AMMO_FULL_DAMP_SCALE        1.15f
#define GIMBAL_YAW_AMMO_EMPTY_DAMP_SCALE       1.65f


/* 对 pitch 进行动态限位 */
#define MAX_UP_ANGLE 45.0
#define MIN_UP_ANGLE (-11)

#define MAX_MID_ANGLE 45.0
#define MIN_MID_ANGLE (-12)

//TODO:这部分不要了，趴下后pitch不用动
#define MAX_DOWN_ANGLE 30.5
#define MIN_DOWN_ANGLE (-11)

#define MAX_SPIN_ANGLE 45.0
#define MIN_SPIN_ANGLE 5.0f

#define UP_PITCH_UP_POSITION_MAX 6920
#define UP_PITCH_UP_POSITION_MIN 5325

//TODO:这部分不要了，趴下后pitch不用动
#define UP_PITCH_DOWN_POSITION_MAX 7600
#define UP_PITCH_DOWN_POSITION_MIN 6200

#define UP_PITCH_MIDDLE_POSITION_MAX 7200
#define UP_PITCH_MIDDLE_POSITION_MIN 6000

#define UP_PITCH_SWITCH_POSITION_MAX 7600
#define UP_PITCH_SWITCH_POSITION_MIN 5325

#define ANGLE_TO_POSITION_RATIO_UP 23.40f
#define ANGLE_TO_POSITION_RATIO_DOWN 6.f
#define ANGLE_TO_POSITION_RATIO_MIDDLE 6.f

/* 特殊位置 POSITION 值 */
#define UP_PITCH_OFFSET_POSITION                    5758   // 上pitch电机在抬头时的编码值（回中编码值）
#define UP_PITCH_DOWN_POSITION                      7122   // 上pitch电机在低头时的编码值
#define UP_PITCH_MIDDLE_POSITION                    6603   // 云台呈45度时的编码值（无敌战斗模式）

#define DOWN_PITCH_OFFSET_POSITION                  4414   // 下pitch电机在抬头时的编码值（回中编码值）
#define DOWN_PITCH_DOWN_POSITION                    3053   // 下pitch电机在低头时的编码值
#define DOWN_PITCH_MIDDLE_POSITION                  3391   // 云台呈45度时的编码值（无敌战斗模式）

#define YAW_OFFSET_POSITION                         3803   // yaw电机在云台正前方时的编码值（回中编码值）


/* DM电机参数 */
#define UP_PITCH_KP                            5.0f
#define UP_PITCH_KD                            0.05f
#define UP_PITCH_MOVE_KP                       1.5f

#define DOWN_PITCH_KP                          8.2f
#define DOWN_PITCH_KD                          0.3f

#define YAW_KP                                 1.5f
#define YAW_KD                                 0.005f

#define UP_PITCH_GRAVITY_COMPENSATION_ENABLE   1
#define UP_PITCH_GRAVITY_COMPENSATION_SIGN     (-1.0f)
#define UP_PITCH_HOLD_TORQUE                   2.5f
#define UP_PITCH_ANGLE_TORQUE_KP               0.0f
#define UP_PITCH_ANGLE_TORQUE_MAX              0.0f
#define UP_PITCH_TORQUE_MAX                    4.0f

#define PITCH_GRAVITY_COMPENSATION_UP          2.0f   // 上pitch重力补偿扭矩
#define PITCH_GRAVITY_COMPENSATION_DOWN        5.0f   // 下pitch重力补偿扭矩

/*********************************************************************************************************
*                                              枚举结构体定义
*********************************************************************************************************/
/* 电机的获取值和计算值 */
/*********************************************************************************************************
*                                              Pitch posture switch parameters
*********************************************************************************************************/
/* Pitch pose smooth parameters */
#define PITCH_SMOOTH_THRESHOLD_1               220.0f
#define PITCH_SMOOTH_THRESHOLD_2               60.0f
#define PITCH_SMOOTH_THRESHOLD_3               12.0f
#define PITCH_SMOOTH_FAST_SPEED                3.0f
#define PITCH_SMOOTH_RECOVER_FAST_SPEED        3.0f
#define PITCH_SMOOTH_MIN_STEP                  1.0f
#define PITCH_SMOOTH_RECOVER_MIN_STEP          1.0f
#define PITCH_SMOOTH_RATIO_1                   0.18f
#define PITCH_SMOOTH_RATIO_2                   0.10f
#define PITCH_SMOOTH_ACCEL_STEP                0.35f
#define PITCH_SMOOTH_DT_MAX_MS                 5U
#define PITCH_CHASSIS_READY_DELAY_MS           180U

/* Down pitch hold parameters */
#define DOWN_PITCH_MOVE_KP                     3.0f
#define DOWN_PITCH_MOVE_KD                     0.15f
#define DOWN_PITCH_HOLD_KP                     14.0f
#define DOWN_PITCH_HOLD_KD                     0.40f
#define DOWN_PITCH_HOLD_TORQUE                 4.2f

/* Dual pitch switch parameters */
#define PITCH_SWITCH_LEVEL_ANGLE_SET           0.0f
#define PITCH_SWITCH_LEVEL_FIX_LIMIT           0.0f
#define PITCH_SWITCH_ABS_HOLD_TARGET           1.5f
#define PITCH_SWITCH_ABS_HOLD_KP               1.8f
#define PITCH_SWITCH_ABS_HOLD_MAX_POS          320.0f
#define PITCH_SWITCH_RECOVER_ABS_HOLD_KP       0.45f
#define PITCH_SWITCH_RECOVER_ABS_HOLD_MAX_POS  80.0f
#define PITCH_SWITCH_LEVEL_DOWN_FAST_SPEED     1.0f
#define PITCH_SWITCH_LEVEL_DOWN_MIN_STEP       0.45f
#define PITCH_SWITCH_LEVEL_UP_TORQUE           2.0f
#define PITCH_SWITCH_LEVEL_RELEASE_PROGRESS    0.82f
#define PITCH_SWITCH_UP_RELEASE_ERR            120.0f
#define PITCH_SWITCH_DOWN_RELEASE_ERR          160.0f
#define PITCH_SWITCH_ABS_ZERO_SPEED            8.0f
#define PITCH_SWITCH_ABS_ZERO_DT_MAX_MS        20U
#define PITCH_SWITCH_ABS_ZERO_RELEASE_ERR      0.2f
#define PITCH_SWITCH_RECOVER_EXIT_HOLD_ANGLE   1.2f
#define PITCH_SWITCH_RECOVER_EXIT_HOLD_PROGRESS 0.40f
#define PITCH_SWITCH_RECOVER_EXIT_HOLD_KP      1.6f
#define PITCH_SWITCH_RECOVER_EXIT_HOLD_MAX_POS 120.0f

#define GIMBAL_UP_DISABLE_ENTER_TEMP           90U
#define GIMBAL_UP_DISABLE_EXIT_TEMP            75U
#define GIMBAL_UP_DISABLE_TRANSITION_SETTLE_MS 360U
#define GIMBAL_UP_DISABLE_DOWN_TO_DISABLE_EXTRA_MS 120U
#define GIMBAL_UP_DISABLE_ENTER_HOLD_ANGLE     1.0f
#define GIMBAL_UP_DISABLE_ENTER_HOLD_KP        2.2f
#define GIMBAL_UP_DISABLE_ENTER_HOLD_MAX_POS   120.0f

/*********************************************************************************************************
*                                              Enum and struct definitions
*********************************************************************************************************/
typedef enum {
    PITCH_SWITCH_IDLE = 0,
    PITCH_SWITCH_JOINT_PLAN,
} pitch_switch_state_e;

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
    fp32 last_absolute_angle_set;// 上一次的目标角度，用于计算前馈

    /* 云台底盘相对角度 */
    fp32 relative_up_down_set;
    fp32 relative_up_down_get;

    fp32 smooth_position;
    fp32 smooth_target;
    fp32 smooth_step;
    fp32 smooth_fast_speed;
    fp32 smooth_min_step;
    uint32_t smooth_last_tick;
    uint8_t smooth_arrived;
}Motor_Gimbal_t;

typedef struct {
    fp32 angle_set;
    fp32 vel_ff;
    fp32 acc_ff;
} yaw_cmd_t;

typedef struct {
    fp32 err;
    fp32 target_err;
    fp32 plan_angle;
    fp32 set_vel;
    fp32 vel_ff;
    fp32 vel_ref;
    fp32 vel_err;
    fp32 brake_torque;
    fp32 friction_torque;
    fp32 out_torque;
    fp32 load_scale;
    fp32 pose_scale;
    fp32 ammo_ratio;
} yaw_ctrl_debug_t;

typedef struct {
    /* 电机信息 */
    Motor_Gimbal_t yaw;
    Motor_Gimbal_t up_pitch;
    Motor_Gimbal_t down_pitch;

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

    first_order_filter_type_t up_pitch_first_order_set;
    first_order_filter_type_t up_pitch_current_first_order_set;
    first_order_filter_type_t down_pitch_first_order_set;
    first_order_filter_type_t down_pitch_current_first_order_set;

}gimbal_t;

/*********************************************************************************************************
*                                              对外允许调用文件
*********************************************************************************************************/
extern gimbal_t gimbal;
extern yaw_ctrl_debug_t yaw_ctrl_debug;
extern void Gimbal_Can_Msg(uint8_t can,uint32_t can_id, uint8_t *can_msg);
extern void Chassis_to_Gimbal_Can(uint32_t can_id, const uint8_t *rx_data);
void Gimbal_DM_Ctrl_Loop_Cal(void);

#endif //HERO_GIMBAL_H
