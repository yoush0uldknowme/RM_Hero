/******************define******************/

/*轮子电机id：          前                         舵：              前                      解算坐标：            x(前)
            ****              ****                     ****               ****                           |
           *  2 *            * 1  *                   * LF *             * RF *                          |
            ****              ****                     ****               ****                           |
                                                                                                         |
           左                   右                                                        y(左)-----------z--------------
                                                                                                         |
            ****              ****                     ****               ****                           |
           *  3 *            * 4  *                   * LB *             * RB *                          |
            ****              ****                     ****               ****                           |

*/
/*********************************************************************************************************
*                                              包含头文件
*********************************************************************************************************/
#include "Chassis.h"
#include "Gimbal.h"
#include "launcher.h"
#include "Steering_Wheel_Solving.h"
#include "power_limit.h"
#include "can_send.h"
#include "usart.h"
#include "Atti.h"
#include "Cap.h"
#include "VTM.h"

/*********************************************************************************************************
*                                              内部变量
*********************************************************************************************************/
chassis_t chassis;
/* 功率控制的比值 */
fp32 k;
fp32 buffer_limit;

uint8_t cap_is_on = 0;
uint8_t power_boost_on = 0;
uint8_t spin_mode = 0;
uint8_t board_reset_request = 0;

HAL_StatusTypeDef ret1;
HAL_StatusTypeDef ret2;
extern robot_ctrl_info_t robot_ctrl;

static uint8_t board_reset_hold_active = 0;
static uint32_t board_reset_hold_start_tick = 0;
static uint32_t board_reset_request_last_tick = 0;

ramp_function_source_speed_t chassis_vx_ramp;
ramp_function_source_speed_t chassis_vy_ramp;
ramp_function_source_speed_t chassis_vw_ramp;

/*********************************************************************************************************
*                                              枚举结构体定义
*********************************************************************************************************/

/*********************************************************************************************************
*                                              内部函数声明
*********************************************************************************************************/
_Noreturn void Chassis_task(void const *pvParameters);

static void Chassis_Init(void);

static void Chassis_Ctrl_Info_Get(void);

static void Chassis_Pc_Ctrl(void);

static void Chassis_Control(void);

static void Chassis_Device_Offline_Handle(void);

static void LORA_testing(void);

static void Send_Chassis_Data(void);

static void Chassis_Test_Data(void);

static void Chassis_Follow_Gimbal_Handle(void);

static void Chassis_Relax_Handle(void);

static void Chassis_Spin_Handle(void);

static fp32 Dynamic_vw_Calc();
static void Chassis_Software_Reset_Handle(void);

/*********************************************************************************************************
*                                              内部函数实现
*********************************************************************************************************/
//TODO:非必要不要调换延迟和通讯顺序
_Noreturn void Chassis_task(void const *pvParameters) {
    vTaskDelay(CHASSIS_TASK_INIT_TIME);

    /* 底盘初始化 */
    Chassis_Init();
    Launcher_Chassis_Init();

    PowerLimit_Steering_init();
    while (1) {
        /* 遥控器获取底盘方向矢量 */
        Chassis_Ctrl_Info_Get();


        // if(switch_is_down(rc_ctrl.rc.s[RC_s_L]) && switch_is_down(rc_ctrl.rc.s[RC_s_R])) {
        //     gimbal.chassis_mode=CHASSIS_RELAX;
        // }
        // //左不下 右下： 底盘独立CHASSIS_ONLY
        // else if(!switch_is_down(rc_ctrl.rc.s[RC_s_L]) && switch_is_down(rc_ctrl.rc.s[RC_s_R])) {
        //     gimbal.chassis_mode=CHASSIS_ONLY;
        // }

        /* 底盘控制 */
        Chassis_Software_Reset_Handle();
        Chassis_Control();
        Launcher_Mode_Set();

        /* 远程通讯 */
        // LORA_testing();

        /* 更新板间传输信息 */
        Send_Chassis_Data();

        // if(Referee.GameRobotStat.power_management_chassis_output == 0) {
        //     chassis.mode = CHASSIS_RELAX;
        // }

        /* 不是失能模式且底盘供电 */

        /*调试代码*/
        // chassis.mode = CHASSIS_ONLY;
        // chassis.mode = CHASSIS_FOLLOW_GIMBAL;
        // gimbal.mode = GIMBAL_FIRE;
        power_boost_on = 1;
        // cap_is_on = 0;

        Launcher_Chassis_Control();
        Chassis_Device_Offline_Handle();

        if (chassis.mode != CHASSIS_RELAX) {
            /* 舵轮解算 */
            Chassis_Steering_Wheel_Cal();

            /*新功控超电相关逻辑*/
            if (feedback_frame.voltage > 500) {
                cap_is_on = 1;
            } else {
                cap_is_on = 0;
            }

            /* 驱电机闭环 */
            Chassis_Wheel_Loop_Cal();

            /* 检测电机、电源是否断线 */
            Chassis_Device_Offline_Handle();

            /* 控制电机 */

            /* 左侧轮组 */
            DJI_Send_Motor_Mapping(CAN_2,
                                   CAN_DJI_MOTOR_0x200_ID,
                                   0, //201
                                   chassis.drive_motor_chassis[LF].give_current, //202
                                   chassis.drive_motor_chassis[LB].give_current, //203
                                   launcher.trigger.give_current //204
            );
            DJI_Send_Motor_Mapping(CAN_2,
                                   CAN_DJI_MOTOR_0x1FF_ID,
                                   0, //205
                                   chassis.Steering_motor_chassis[LF].give_current, //206
                                   chassis.Steering_motor_chassis[LB].give_current, //207
                                   0 //208
            );


            /* 右侧轮组 */
            DJI_Send_Motor_Mapping(CAN_1,
                                   CAN_DJI_MOTOR_0x200_ID,
                                   chassis.drive_motor_chassis[RF].give_current, //201
                                   0, //202
                                   0, //203
                                   chassis.drive_motor_chassis[RB].give_current //204
            );
            DJI_Send_Motor_Mapping(CAN_1,
                                   CAN_DJI_MOTOR_0x1FF_ID,
                                   chassis.Steering_motor_chassis[RF].give_current, //205
                                   0, //206
                                   0, //207
                                   chassis.Steering_motor_chassis[RB].give_current //208
            );

            /* 底盘各个方向的速度 */
            Chassis_Test_Data();
        }
        vTaskDelay(CHASSIS_PERIOD);
    }
}


/**
  * @brief          底盘初始化
  * @param[in]      pvParameters
  * 获取底盘四个轮子、trigger 电机数据
  * 初始化底盘四个轮子、底盘跟随云台模式、和 trigger 电机的角度环和速度环 PID
  * 将底盘和 trigger 电机的电流给定为 0，即失能
  * 设置功率限制时的缓冲能量和功率的警告线
  * 初始化主动单发的 total_ecd、offset_ecd 和 ecd
  * @retval         none
  */
static void Chassis_Init(void) {
    //舵轮初始化
    Chassis_Steering_Wheel_Init();

    chassis.mode = CHASSIS_RELAX;

    /* 底盘电机速度环PID初始化 */
    for (int i = 0; i < 4; i++) {
        pid_init(&chassis.drive_motor_chassis[i].speed_p,
                 CHASSIS_3508_SPEED_PID_MAX_OUT, CHASSIS_3508_SPEED_PID_MAX_IOUT,
                 CHASSIS_3508_SPEED_PID_KP, CHASSIS_3508_SPEED_PID_KI, CHASSIS_3508_SPEED_PID_KD);
    }

    for (int i = 0; i < 4; i++) {
        pid_init(&chassis.Steering_motor_chassis[i].speed_p,
                 CHASSIS_6020_SPEED_PID_MAX_OUT, CHASSIS_6020_SPEED_PID_MAX_IOUT,
                 CHASSIS_6020_SPEED_PID_KP, CHASSIS_6020_SPEED_PID_KI, CHASSIS_6020_SPEED_PID_KD);
    }

    /* 底盘电机角度环PID初始化 */
    for (int i = 0; i < 4; i++) {
        pid_init(&chassis.Steering_motor_chassis[i].angle_p, CHASSIS_6020_ANGLE_MAX_OUT,
                 CHASSIS_6020_ANGLE_MAX_IOUT, CHASSIS_6020_ANGLE_PID_KP,
                 CHASSIS_6020_ANGLE_PID_KI, CHASSIS_6020_ANGLE_PID_KD);
    }

    /* 底盘跟随云台初始化 */
    pid_init(&chassis.chassis_vw_pid,
             CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT,
             CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD);

    for (int i = 0; i < 4; i++) {
        chassis.drive_motor_chassis[i].give_current = 0;
        chassis.Steering_motor_chassis[i].give_current = 0;
        // chassis.Steering_motor_chassis[i].relative_angle_get = 0.f;
        // chassis.Steering_motor_chassis[i].relative_angle_set = 0.f;
        // chassis.Steering_motor_chassis[i].motor_measure.speed_rpm = 0;
    }


    //低通滤波初始化
    first_order_filter_init(&chassis.vx_slow, 1, 8);
    first_order_filter_init(&chassis.vy_slow, 1, 15);
    first_order_filter_init(&chassis.vw_slow, 1, 40);

    ramp_init(&chassis.chassis_ramp[LF], 0.0001f, M3508_MAX_RPM, -M3508_MAX_RPM);
    ramp_init(&chassis.chassis_ramp[RF], 0.0001f, M3508_MAX_RPM, -M3508_MAX_RPM);
    ramp_init(&chassis.chassis_ramp[RB], 0.0001f, M3508_MAX_RPM, -M3508_MAX_RPM);
    ramp_init(&chassis.chassis_ramp[LB], 0.0001f, M3508_MAX_RPM, -M3508_MAX_RPM);
    ramp_speed_init(&chassis_vx_ramp, 0.1f, MAX_CHASSIS_AUTO_VX_SPEED, -MAX_CHASSIS_AUTO_VX_SPEED, 0.013f);
    ramp_speed_init(&chassis_vy_ramp, 0.1f, MAX_CHASSIS_AUTO_VY_SPEED, -MAX_CHASSIS_AUTO_VY_SPEED, 0.013f);

    // ramp_speed_init(&chassis_vw_ramp, 0.095f, MAX_CHASSIS_AUTO_VY_SPEED, -MAX_CHASSIS_AUTO_VY_SPEED,0.013f);

    // chassis_power_limit;功率限制时的缓冲能量和功率的警告线;比赛根据裁判系统反馈动态设定
    chassis.chassis_power_limit.warning_buff = 35.f;
    chassis.chassis_power_limit.warning_power = 40.f;
    chassis.chassis_power_limit.power_set = 50;
}


// /**
//   * @brief          电机上电判断
//   * @param[in]      none
//   * WSAD底盘移动
//   * @retval         返回空
//   */
// static uint8_t Chassis_Steering_Feedback_Ready(void) {
//
//
//
//     return steering_feedback_ready[RF] && steering_feedback_ready[LF] &&
//            steering_feedback_ready[LB] && steering_feedback_ready[RB];
// }


/**
  * @brief          键盘控制
  * @param[in]      none
  * WSAD底盘移动
  * @retval         返回空
  */
float speed_change = 0; //0.03f;
static void Chassis_Pc_Ctrl(void) {
    /* 获取加速度 */
    chassis.shift_slow_mode = (KeyBoard.SHIFT.status == KEY_PRESS);
    speed_change = CHASSIS_SHIFT_PC_SPEED_CHANGE;
    fp32 vx_pc_limit = MAX_CHASSIS_AUTO_VX_SPEED * (chassis.shift_slow_mode ? CHASSIS_SHIFT_SLOW_FACTOR : 1.0f);
    fp32 vy_pc_limit = MAX_CHASSIS_AUTO_VY_SPEED * (chassis.shift_slow_mode ? CHASSIS_SHIFT_SLOW_FACTOR : 1.0f);
    /* 优先处理组合键 */
    if (KeyBoard.W.status == KEY_PRESS && KeyBoard.A.status == KEY_PRESS) {
        if (abs(chassis.vx_pc) > abs(chassis.vy_pc)) {
            chassis.vy_pc = -chassis.vx_pc;
        } else {
            chassis.vx_pc = -chassis.vy_pc;
        }
        chassis.vx_pc += speed_change;
        chassis.vy_pc -= speed_change;
    } else if (KeyBoard.W.status == KEY_PRESS && KeyBoard.D.status == KEY_PRESS) {
        if (abs(chassis.vx_pc) > abs(chassis.vy_pc)) {
            chassis.vy_pc = chassis.vx_pc;
        } else {
            chassis.vx_pc = chassis.vy_pc;
        }
        chassis.vx_pc += speed_change;
        chassis.vy_pc += speed_change;
    } else if (KeyBoard.S.status == KEY_PRESS && KeyBoard.A.status == KEY_PRESS) {
        if (abs(chassis.vx_pc) > abs(chassis.vy_pc)) {
            chassis.vy_pc = chassis.vx_pc;
        } else {
            chassis.vx_pc = chassis.vy_pc;
        }
        chassis.vx_pc -= speed_change;
        chassis.vy_pc -= speed_change;
    } else if (KeyBoard.S.status == KEY_PRESS && KeyBoard.D.status == KEY_PRESS) {
        if (abs(chassis.vx_pc) > abs(chassis.vy_pc)) {
            chassis.vy_pc = -chassis.vx_pc;
        } else {
            chassis.vx_pc = -chassis.vy_pc;
        }
        chassis.vx_pc -= speed_change;
        chassis.vy_pc += speed_change;
    }
    // 处理独立按键
    else if (KeyBoard.W.status == KEY_PRESS) {
        if (chassis.vx_pc < 0) {
            chassis.vx_pc = 0; // 立即停止向前运动
        }
        chassis.vy_pc = 0;
        chassis.vx_pc += speed_change;
    } else if (KeyBoard.S.status == KEY_PRESS) {
        if (chassis.vx_pc > 0) {
            chassis.vx_pc = 0; // 立即停止向后运动
        }
        chassis.vy_pc = 0;
        chassis.vx_pc -= speed_change;
    } else if (KeyBoard.A.status == KEY_PRESS) {
        if (chassis.vy_pc > 0) {
            chassis.vy_pc = 0; // 立即停止向左运动
        }
        chassis.vx_pc = 0;
        chassis.vy_pc -= speed_change;
    } else if (KeyBoard.D.status == KEY_PRESS) {
        if (chassis.vy_pc < 0) {
            chassis.vy_pc = 0; // 立即停止向右运动
        }
        chassis.vx_pc = 0;
        chassis.vy_pc += speed_change;
    }
    // 无按键时归零
    else {
        chassis.vx_pc = 0;
        chassis.vy_pc = 0;
    }
    VAL_LIMIT(chassis.vx_pc, -vx_pc_limit, vx_pc_limit);
    VAL_LIMIT(chassis.vy_pc, -vy_pc_limit, vy_pc_limit);
}

/**
  * @brief          初始化键盘和遥控器控制底盘
  * @param[in]      none
  * 初始化键盘信息，通过键盘信息和遥控器信息控制底盘移动
  * @retval         返回空
  */
static void Chassis_Ctrl_Info_Get(void) {
    Chassis_Pc_Ctrl(); //根据键盘信息 获取vx_pc,vy_pc

    // chassis.vx_set = (float) (rc_ctrl.rc.ch[CHASSIS_X_CHANNEL]) * RC_TO_VX
    //                  + chassis.vx_pc;
    // chassis.vy_set = (float) (rc_ctrl.rc.ch[CHASSIS_Y_CHANNEL]) * RC_TO_VY
    //                  + chassis.vy_pc;
    fp32 chassis_speed_factor = chassis.shift_slow_mode ? CHASSIS_SHIFT_SLOW_FACTOR : 1.0f;
    fp32 vx_limit = MAX_CHASSIS_AUTO_VX_SPEED * chassis_speed_factor;
    fp32 vy_limit = MAX_CHASSIS_AUTO_VY_SPEED * chassis_speed_factor;
    fp32 vxy_limit = MAX_CHASSIS_AUTO_VXY_SPEED * chassis_speed_factor;
    fp32 vx_input = (float)(rc_ctrl.rc.ch[CHASSIS_X_CHANNEL]) * RC_TO_VX + chassis.vx_pc;
    fp32 vy_input = (float)(rc_ctrl.rc.ch[CHASSIS_Y_CHANNEL]) * RC_TO_VY + chassis.vy_pc;

    chassis.vw_set = (float) (rc_ctrl.rc.ch[CHASSIS_Z_CHANNEL]) * RC_TO_VW
                     + chassis.vw_pc;

    //TODO:怕翻车时打开
    /*斜坡，防止翻车*/
    chassis_vx_ramp.max_value = vx_limit;
    chassis_vx_ramp.min_value = -vx_limit;
    chassis_vy_ramp.max_value = vy_limit;
    chassis_vy_ramp.min_value = -vy_limit;
    VAL_LIMIT(vx_input, -vx_limit, vx_limit);
    VAL_LIMIT(vy_input, -vy_limit, vy_limit);
    ramp_speed_calc(&chassis_vx_ramp, vx_input);
    chassis.vx_set = chassis_vx_ramp.out;
    ramp_speed_calc(&chassis_vy_ramp, vy_input);
    chassis.vy_set = chassis_vy_ramp.out;

    chassis.vxy = sqrt(chassis.vx_set*chassis.vx_set+chassis.vy_set*chassis.vy_set);

    static fp32 coefficient = 0;

    if (chassis.vxy > vxy_limit && fabsf(chassis.vx_set) > 0.2f && fabsf(chassis.vy_set) > 0.2f)
    {
        coefficient = vxy_limit / chassis.vxy;
        chassis.vx_set = chassis.vx_set * coefficient;
        chassis.vy_set = chassis.vy_set * coefficient;
    }
    //
    // chassis.vw_set=(float)(rc_ctrl.rc.ch[CHASSIS_Z_CHANNEL])*RC_TO_VW
    //                +chassis.vw_pc;
    /**/


    // ramp_speed_calc(&chassis_vy_ramp,(float)(rc_ctrl.rc.ch[CHASSIS_Z_CHANNEL])*RC_TO_VW+chassis.vw_pc);
    // chassis.vy_set = chassis_vy_ramp.out;

    // 将 chassis.vx_set 进行平滑处理，生成一个平滑的输出 chassis.vx_slow.out，该输出是底盘的实际控制速度。
    first_order_filter_cali(&chassis.vx_slow, chassis.vx_set);
    first_order_filter_cali(&chassis.vy_slow, chassis.vy_set); //缓慢上升不要太快
    first_order_filter_cali(&chassis.vw_slow, chassis.vw_set); //缓慢上升不要太快

    //TODO:开启速度输入斜坡
    // chassis.vx = chassis.vx_slow.out;
    // chassis.vy = chassis.vy_slow.out;
    // chassis.vw = chassis.vw_slow.out;

    //TODO:关闭斜坡
    chassis.vx = chassis.vx_set;
    chassis.vy = chassis.vy_set;
    chassis.vw = chassis.vw_set;
}


/**
  * @brief          底盘模式选择，yaw轴模式选择
  * @param[in]      pvParameters
  * 底盘五种模式：底盘失能，跟随，独立，小陀螺，刹车
  * 云台一种模式：trigger失能
  * @retval         none
  */
static void Chassis_Control(void) {
    switch (chassis.mode) {
        case CHASSIS_RELAX: //底盘失能
            Chassis_Relax_Handle();
            break;
        case CHASSIS_ONLY: //底盘独立(以初始化数据控制)
            break;
        case CHASSIS_FOLLOW_GIMBAL: //跟随环 覆盖底盘 vw;
            Chassis_Follow_Gimbal_Handle();
            // Chassis_Relax_Handle();
            break;
        case CHASSIS_SPIN_R: //小陀螺 vw设置常量(150)
        case CHASSIS_SPIN_L: //小陀螺 vw设置常量(150)
            Chassis_Spin_Handle();
            break;
        // case CHASSIS_FIRE: //发射
        //     chassis.vx = chassis.vy = chassis.vw = 0;
        //     break;
        default:
            break;
    }
}


static void Chassis_Device_Offline_Handle(void) {
    if (detect_list[DETECT_CHASSIS_3508_RF_DRIVE].status == OFFLINE) {
        chassis.drive_motor_chassis[RF].give_current = 0;
    }
    if (detect_list[DETECT_CHASSIS_3508_LF_DRIVE].status == OFFLINE) {
        chassis.drive_motor_chassis[LF].give_current = 0;
    }
    if (detect_list[DETECT_CHASSIS_3508_LB_DRIVE].status == OFFLINE) {
        chassis.drive_motor_chassis[LB].give_current = 0;
    }
    if (detect_list[DETECT_CHASSIS_3508_RB_DRIVE].status == OFFLINE) {
        chassis.drive_motor_chassis[RB].give_current = 0;
    }

    if (detect_list[DETECT_CHASSIS_6020_RF_STEERING].status == OFFLINE) {
        chassis.Steering_motor_chassis[RF].give_current = 0;
    }
    if (detect_list[DETECT_CHASSIS_6020_LF_STEERING].status == OFFLINE) {
        chassis.Steering_motor_chassis[LF].give_current = 0;
    }
    if (detect_list[DETECT_CHASSIS_6020_LB_STEERING].status == OFFLINE) {
        chassis.Steering_motor_chassis[LB].give_current = 0;
    }
    if (detect_list[DETECT_CHASSIS_6020_RB_STEERING].status == OFFLINE) {
        chassis.Steering_motor_chassis[RB].give_current = 0;
    }

    if (detect_list[DETECT_LAUNCHER_3508_TRIGGER].status == OFFLINE) {
        Launcher_Chassis_Relax_Handle();
    }
}

static void Chassis_Software_Reset_Handle(void) {
    uint32_t now = HAL_GetTick();
    uint8_t reset_request_active = 0U;

    if (board_reset_request != 0U &&
        board_reset_request_last_tick != 0U &&
        (now - board_reset_request_last_tick) <= CHASSIS_RESET_REQUEST_TIMEOUT_MS) {
        reset_request_active = 1U;
    }

    if (reset_request_active == 0U) {
        board_reset_hold_active = 0U;
        board_reset_hold_start_tick = 0U;
        return;
    }

    if (board_reset_hold_active == 0U) {
        board_reset_hold_active = 1U;
        board_reset_hold_start_tick = now;
        return;
    }

    if ((now - board_reset_hold_start_tick) >= CHASSIS_RESET_HOLD_MS) {
        HAL_NVIC_SystemReset();
    }
}


static void LORA_testing(void) {
    uint8_t tx_data[7];
    tx_data[0] = 0x50;
    tx_data[6] = 0x88;
    union referee LORA_tx;
    LORA_tx.referee = Referee.PowerHeatData.buffer_energy;
    tx_data[1] = LORA_tx.data[0];
    tx_data[2] = LORA_tx.data[1];
    tx_data[3] = LORA_tx.data[2];
    tx_data[4] = LORA_tx.data[3];
    tx_data[5] = (uint8_t) Referee.GameRobotStat.chassis_power_limit;
    HAL_UART_Transmit(&huart1, tx_data, 7, 100);
}


static void Send_Chassis_Data(void) {
    /* 只有误差超过一定值才会被判定为正在进行爬坡模式 */
    chassis.board_pitch = INS_angle[2] * MOTOR_RAD_TO_ANGLE;
    if (abs(chassis.board_pitch) > 5) Send_pitch_down(chassis.board_pitch);
    else Send_pitch_down(0.0f);
    vTaskDelay(CHASSIS_PERIOD);

    Send_referee(feedback_frame.voltage, Referee.GameRobotStat.chassis_power_limit, Referee.GameRobotStat.robot_id,Referee.GameRobotStat.shooter_barrel_heat_limit);
    Send_bullet_speed(Referee.ShootData.bullet_speed, Referee.PowerHeatData.shooter_42mm_barrel_heat);
    vTaskDelay(CHASSIS_PERIOD);
}


/* 仅为了读取数据，不作为计算 */
float chassis_vx, chassis_vy, chassis_vw;
double test_y = 0, test_w = 0, test_x = 0;
uint32_t time;
uint32_t test_om = 0;
uint32_t time_last_time;

static void Chassis_Test_Data(void) {
    // //计算底盘在方向上的线速度
    // chassis_vx = ((float) (chassis.motor_chassis[1].motor_measure.speed_rpm
    //                        + chassis.motor_chassis[2].motor_measure.speed_rpm
    //                        - chassis.motor_chassis[0].motor_measure.speed_rpm
    //                        - chassis.motor_chassis[3].motor_measure.speed_rpm
    // ) * (M3508_MOTOR_RPM_TO_VECTOR)) / 4;
    // chassis_vy = ((float) (chassis.motor_chassis[2].motor_measure.speed_rpm
    //                        - chassis.motor_chassis[1].motor_measure.speed_rpm
    //                        - chassis.motor_chassis[0].motor_measure.speed_rpm
    //                        + chassis.motor_chassis[3].motor_measure.speed_rpm
    // ) * (M3508_MOTOR_RPM_TO_VECTOR)) / 4;
    // chassis_vw = ((float) (-chassis.motor_chassis[2].motor_measure.speed_rpm
    //                        - chassis.motor_chassis[1].motor_measure.speed_rpm
    //                        - chassis.motor_chassis[0].motor_measure.speed_rpm
    //                        - chassis.motor_chassis[3].motor_measure.speed_rpm
    // ) * (M3508_MOTOR_RPM_TO_VECTOR)) / 4;
    //
    // time = HAL_GetTick();              //获取当前系统时间
    // test_om = time - time_last_time;   //计算时间间隔
    //
    // //计算在各个方向上的位移
    // test_x = test_x + ((double) test_om * chassis_vx * 0.001);
    // test_y = test_y + ((double) test_om * chassis_vy * 0.001);
    // test_w = test_w + ((double) test_om * chassis_vw * 0.001);
    // time_last_time = time;
}


/**
  * @brief          底盘失能实现
  * @param[in]      pvParameters
  * 将四个轮子的电流信号给定为0
  * @retval         none
  */
static void Chassis_Relax_Handle(void) {
    ret1 = DJI_Send_Motor_Mapping(CAN_1, CAN_DJI_MOTOR_0x200_ID, 0, 0, 0, 0);
    ret2 = DJI_Send_Motor_Mapping(CAN_1, CAN_DJI_MOTOR_0x1FF_ID, 0, 0, 0, 0);
    DJI_Send_Motor_Mapping(CAN_2, CAN_DJI_MOTOR_0x200_ID, 0, 0, 0, 0);
    DJI_Send_Motor_Mapping(CAN_2, CAN_DJI_MOTOR_0x1FF_ID, 0, 0, 0, 0);

    chassis.drive_motor_chassis[RF].give_current = 0;
    chassis.drive_motor_chassis[LF].give_current = 0;
    chassis.drive_motor_chassis[LB].give_current = 0;
    chassis.drive_motor_chassis[RB].give_current = 0;

    chassis.Steering_motor_chassis[RF].give_current = 0;
    chassis.Steering_motor_chassis[LF].give_current = 0;
    chassis.Steering_motor_chassis[LB].give_current = 0;
    chassis.Steering_motor_chassis[RB].give_current = 0;

    launcher.trigger.give_current = 0;
}


/**
  * @brief          底盘跟随云台模式的代码实现
  * @param[in]      pvParameters
  * 获取云台偏航角的相对角度，并转换为弧度 计算云台偏航角 保存底盘速度
  * 保存的底盘速度在云台坐标系下进行分解，得到在该坐标系下的底盘速度
  * 控制底盘的角速度，使其与云台的偏航角保持稳定，使用PID控制器计算底盘的角速度
  * @retval         none
  */
static float test;

static void Chassis_Follow_Gimbal_Handle(void) {
    //获取云台偏航角的相对角度，并转换为弧度
    // if (abs(chassis.board_yaw_angle)<1)
    // {
    //     chassis.board_yaw_angle = 0;
    // }

    fp32 yaw_relative_radian = chassis.board_yaw_angle * ANGLE_TO_RAD;
    fp32 sin_yaw, cos_yaw;

    // 计算云台偏航角
    sin_yaw = -(fp32) sin(yaw_relative_radian);
    cos_yaw = (fp32) cos(yaw_relative_radian);

    // 保存底盘速度
    fp32 vx_temp = chassis.vx;
    fp32 vy_temp = chassis.vy;

    // 保存的底盘速度在云台坐标系下进行分解，得到在该坐标系下的底盘速度
    chassis.vx = (cos_yaw * vx_temp - sin_yaw * vy_temp);
    chassis.vy = (sin_yaw * vx_temp + cos_yaw * vy_temp);

    //控制底盘的角速度，使其与云台的偏航角保持稳定，使用PID控制器计算底盘的角速度
    // if (abs(chassis.board_yaw_angle)<0.5f)
    // {
    //     chassis.vw = -pid_calc(&chassis.chassis_vw_pid, chassis.board_yaw_angle, 0);
    // }

    chassis.vw = -pid_calc(&chassis.chassis_vw_pid, chassis.board_yaw_angle, 0);

    if (abs(chassis.vw) < 0.12f) {
        chassis.vw = 0;
    }
    // 确保变量val的值在指定的范围内，即不小于最小值min且不大于最大值max
    fp32 chassis_speed_factor = chassis.shift_slow_mode ? CHASSIS_SHIFT_SLOW_FACTOR : 1.0f;
    fp32 vx_limit = MAX_CHASSIS_VX_SPEED * chassis_speed_factor;
    fp32 vy_limit = MAX_CHASSIS_VY_SPEED * chassis_speed_factor;
    VAL_LIMIT(chassis.vw, -MAX_CHASSIS_VW_SPEED, MAX_CHASSIS_VW_SPEED);
    VAL_LIMIT(chassis.vx, -vx_limit * (1 - abs(chassis.vw / 4.4f)), vx_limit * (1 - abs(chassis.vw / 4.4f)));
    VAL_LIMIT(chassis.vy, -vy_limit * (1 - abs(chassis.vw / 4.4f)), vy_limit * (1 - abs(chassis.vw / 4.4f)));
}


/**
  * @brief          小陀螺
  * @param[in]      pvParameters
  * 计算云台坐标系下的底盘速度，给定固定的vw最大速度(150)
  * @retval         none
  */
static void Chassis_Spin_Handle(void) {
    //获取云台偏航角的相对角度，并转换为弧度
    fp32 yaw_relative_radian = chassis.board_yaw_angle * ANGLE_TO_RAD;
    fp32 sin_yaw, cos_yaw;
    sin_yaw = -(fp32) sin(yaw_relative_radian);
    cos_yaw = (fp32) cos(yaw_relative_radian);

    // 保存底盘速度
    fp32 vx_temp = chassis.vx;
    fp32 vy_temp = chassis.vy;

    // 保存的底盘速度在云台坐标系下进行分解，得到在该坐标系下的底盘速度
    chassis.vx = (cos_yaw * vx_temp + sin_yaw * vy_temp) * 0.8f;
    chassis.vy = -(sin_yaw * vx_temp - cos_yaw * vy_temp) * 0.8f;

    /* 给定vw转速 */
    if (chassis.mode == CHASSIS_SPIN_L) {
        chassis.vw = CHASSIS_WZ_SET_SPEED;
        // chassis.vw = Dynamic_vw_Calc();
    } else if (chassis.mode == CHASSIS_SPIN_R) {
        chassis.vw = -CHASSIS_WZ_SET_SPEED;
        // chassis.vw = Dynamic_vw_Calc();
    }
}


uint16_t last_HP;
static fp32 Dynamic_vw_Calc() {
    fp32 t = (fp32)HAL_GetTick() / 1000.0f;
    fp32 raw;
    fp32 target;
    static fp32 vw_output = 4.f;
    static fp32 hit_tick = 0;
    static fp32 smooth_tick = 0;
    static uint8_t switch_flag = 0;

    /* 模式切换：受击后先切一次，保留一段时间后再切下一模式 */
    if (Referee.GameRobotStat.current_HP != last_HP) {
        spin_mode = (spin_mode + 1) % 3;

        if (!switch_flag) {
            hit_tick = t;
            switch_flag = 1;
        }
        smooth_tick = t + SPIN_SMOOTH_S;
    }
    if (switch_flag && (t - hit_tick >= SPIN_MODE_HOLD_S)) {
        spin_mode = (spin_mode + 1) % 3;

        switch_flag = 0;
        smooth_tick = t + SPIN_SMOOTH_S;
    }
    last_HP = Referee.GameRobotStat.current_HP;

    /* 转速计算 */
    switch (spin_mode)
    {
        case MODE_1:
            raw = sinf(2.0f * PI * 1.2f * t +
                       0.5f * sinf(2.0f * PI * 0.3f * t));
            break;

        case MODE_2:
            raw = tanhf(2.0f * sinf(2.0f * PI * 2.0f * t));
            break;

        case MODE_3:
            raw = sinf(2.0f * PI * 3.1f * t)
                + 0.6f * sinf(2.0f * PI * 4.7f * t);
            raw = raw / 1.6f;
            break;

        default:
            raw = 0.0f;
            break;
    }

    /* 速度输出 */
    target = AMP * raw + BIAS;

    if (t >= smooth_tick) {  //非切换阶段不走平滑，直接目标输出
        vw_output = target;
        return target;
    }

    vw_output += (target - vw_output) * 0.08f;
    return vw_output;
}


void Chassis_Can_Msg(uint8_t can, uint32_t can_id, uint8_t *can_msg) {
    if (can == CAN_1) {
        switch (can_id) {
            case CAN_CHASSIS_MOTOR_RF_DRIVE: //201
                DJI_Motor_Decode(&chassis.drive_motor_chassis[RF].motor_measure, can_msg);
                detect_handle(DETECT_CHASSIS_3508_RF_DRIVE);
                break;
            case CAN_CHASSIS_MOTOR_RB_DRIVE: //204
                DJI_Motor_Decode(&chassis.drive_motor_chassis[RB].motor_measure, can_msg);
                detect_handle(DETECT_CHASSIS_3508_RB_DRIVE);
                break;
            case CAN_CHASSIS_MOTOR_RF_STEERING: //205
                DJI_Motor_Decode(&chassis.Steering_motor_chassis[RF].motor_measure, can_msg);
                detect_handle(DETECT_CHASSIS_6020_RF_STEERING);
                break;
            case CAN_CHASSIS_MOTOR_RB_STEERING: //208
                DJI_Motor_Decode(&chassis.Steering_motor_chassis[RB].motor_measure, can_msg);
                detect_handle(DETECT_CHASSIS_6020_RB_STEERING);
                break;
            default:
                break;
        }
    } else if (can == CAN_2) {
        switch (can_id) {
            case CAN_CHASSIS_MOTOR_LF_DRIVE: //202
                DJI_Motor_Decode(&chassis.drive_motor_chassis[LF].motor_measure, can_msg);
                detect_handle(DETECT_CHASSIS_3508_LF_DRIVE);
                break;
            case CAN_CHASSIS_MOTOR_LB_DRIVE: //203
                DJI_Motor_Decode(&chassis.drive_motor_chassis[LB].motor_measure, can_msg);
                detect_handle(DETECT_CHASSIS_3508_LB_DRIVE);
                break;
            case CAN_CHASSIS_MOTOR_LF_STEERING: //206
                DJI_Motor_Decode(&chassis.Steering_motor_chassis[LF].motor_measure, can_msg);
                detect_handle(DETECT_CHASSIS_6020_LF_STEERING);
                break;
            case CAN_CHASSIS_MOTOR_LB_STEERING: //207
                DJI_Motor_Decode(&chassis.Steering_motor_chassis[LB].motor_measure, can_msg);
                detect_handle(DETECT_CHASSIS_6020_LB_STEERING);
                break;
            case CAN_LAUNCHER_TRIGGER: //204
                DJI_Motor_Decode(&launcher.trigger.motor_measure, can_msg);
                DJI_Round_Count(&launcher.trigger.motor_measure); //获取转动拨轮电机转动圈数和总编码值
                detect_handle(DETECT_LAUNCHER_3508_TRIGGER);
                break;
            default:
                break;
        }
    }
}


void Gimbal_to_Chassis_Can(uint32_t can_id, const uint8_t *rx_data) {
    switch (can_id) {
        case 0x110: {
            union ctrl_ch ch1;
            union ctrl_ch ch0;
            union ctrl_ch ch2;
            union mode mode1;
            {
                ch1.data[0] = rx_data[0];
                ch1.data[1] = rx_data[1];
                ch0.data[0] = rx_data[2];
                ch0.data[1] = rx_data[3];
                ch2.data[0] = rx_data[4];
                ch2.data[1] = rx_data[5];
                mode1.data[0] = rx_data[6];
                mode1.data[1] = rx_data[7];
            }
            rc_ctrl.rc.ch[1] = ch1.value;
            rc_ctrl.rc.ch[0] = ch0.value;
            rc_ctrl.rc.ch[2] = ch2.value;
            chassis.mode = mode1.mode;
        }
        break;

        case 0x111: {
            union angle angle1;
            {
                KeyBoard.W.status = rx_data[0];
                KeyBoard.A.status = rx_data[1];
                KeyBoard.S.status = rx_data[2];
                KeyBoard.D.status = rx_data[3];
                angle1.data[0] = rx_data[4];
                angle1.data[1] = rx_data[5];
                angle1.data[2] = rx_data[6];
                angle1.data[3] = rx_data[7];
            }
            chassis.board_yaw_angle = angle1.value;
        }
        break;

        case 0x112: {
            uint8_t ctrl_flags;
            {
                KeyBoard.SHIFT.status = rx_data[0];
                KeyBoard.Mouse_l.status = rx_data[1];
                KeyBoard.Q.click_flag = rx_data[2];
                KeyBoard.Mouse_r.status = rx_data[3];
                ctrl_flags = rx_data[4];
            }
            vtm_rx_data.trigger = ((ctrl_flags & CTRL_FLAG_TRIGGER) != 0U) ? 1U : 0U;
            robot_ctrl.fire_command = ((ctrl_flags & CTRL_FLAG_FIRE_COMMAND) != 0U) ? 1 : 0;
            board_reset_request = ((ctrl_flags & CTRL_FLAG_RESET_REQUEST) != 0U) ? 1U : 0U;
            board_reset_request_last_tick = HAL_GetTick();
        }
        break;

        case 0x117: {
            union mode mode1;
            union mode mode2;
            union mode mode3;
            union mode angle1;
            {
                mode1.data[0] = rx_data[0];
                mode1.data[1] = rx_data[1];
                mode2.data[0] = rx_data[2];
                mode2.data[1] = rx_data[3];
                rc_ctrl.rc.s[RC_s_R] = rx_data[4];
                rc_ctrl.rc.s[RC_s_L] = rx_data[5];
                angle1.data[0] = rx_data[6];
                angle1.data[1] = rx_data[7];
            }
            gimbal.mode = mode1.mode;
            launcher.fire_mode = mode2.mode;
            // launcher.shoot_cmd = mode3.mode;
            gimbal.pitch.relative_up_down_get = angle1.mode;
        }
        break;

        // case 0x130:{
        //         union ctrl_ch_s s0;
        //         union ctrl_ch_s s1;
        //
        //         {
        //             s0.data = rx_data[0];
        //             s1.data = rx_data[1];
        //         }
        //         rc_ctrl.rc.s[0] = s0.value;
        //         rc_ctrl.rc.s[1] = s1.value;
        //
        //         gimbal.mode = rx_data[2];
        // } break;

        default:
            break;
    }
}