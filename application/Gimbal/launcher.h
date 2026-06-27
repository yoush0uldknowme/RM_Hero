#ifndef LAUNCHER_H
#define LAUNCHER_H

/*********************************************************************************************************
*                                              包含头文件
*********************************************************************************************************/
#include "Hero.h"
#include "Feedforward_PID.h"
#include "PID.h"
#include "DJI_Motor.h"
#include "SMC.h"


/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/
//// 摩擦轮转速
// #define FIRE_SPEED_L  4710//4910//5000//4600//5000
// #define FIRE_SPEED_R  FIRE_SPEED_L
// #define FIRE_SPEED_ON 4760//4960//4650//5050

//// 摩擦轮转速16
// #define FIRE_SPEED_L  4910//5000//4600//5000
// #define FIRE_SPEED_R  FIRE_SPEED_L
// #define FIRE_SPEED_ON 4960//4650//5050


//// 摩擦轮转速12
#define FIRE_SPEED_L  3995//3855//4910//5000//4600//5000
#define FIRE_SPEED_R  FIRE_SPEED_L
#define FIRE_SPEED_ON 4265//4055//4960//4650//5050

//test
// #define FIRE_SPEED_L  100//3855//4910//5000//4600//5000
// #define FIRE_SPEED_R  100//3855
// #define FIRE_SPEED_ON 100//4055

//// 3508编码器转一圈编码值加8192  减速比1:19  编码器转19圈输出轴才转一圈  19×8192
// #define DEGREE_60_TO_ENCODER  68500.44f//26209.f//26212.f
// #define DEGREE_120_TO_ENCODER 143500.f
#define DEGREE_60_TO_ENCODER  26225.f//26259.f//26209.f//26212.f
#define DEGREE_120_TO_ENCODER 52450.f//52418.f

/*********************************************************************************************************
*                                              枚举结构体定义
*********************************************************************************************************///
/* 电机的获取值和计算值 */
typedef struct {
    DJI_Motor_t motor_measure;    // 电机的真实信息

    fp32 speed;                   // 摩擦轮转速设定值
    VSP_PID_t feedforward_speed_p;// 摩擦轮加前馈速度环pid
    pid_type_def speed_p;         // 拨盘速度环pid
    pid_type_def angle_p;         // 拨盘角度环pid
    int16_t give_current;         // 给定的电流值
}Motor_Launcher_t;

/* 发射机构的信息集合 */
typedef struct {
    /* 电机信息 */
    Motor_Launcher_t fire_l;
    Motor_Launcher_t fire_r;
    Motor_Launcher_t trigger;

    /* 摩擦轮状态 */
    Fire_Mode_e fire_mode;
    Fire_Mode_e fire_last_mode;

    /* 拨盘状态 */
    Shoot_Cmd_e shoot_last_cmd;
    Shoot_Cmd_e shoot_cmd;

    first_order_filter_type_t filter_fire;
    first_order_filter_type_t filter_trigger;
}launcher_t;

/*********************************************************************************************************
*                                              对外允许调用文件
*********************************************************************************************************/
extern launcher_t launcher;
extern void Launcher_Init(void);
extern void Launcher_Mode_Set(void);
extern void Launcher_Control(void);
extern void Launcher_Relax_Handle(void);

#endif //LAUNCHER_H