
/*遥控器键位：

        S1                                                                         S0
                    |                                                   |
                    |                                                   |
                    |                                                   |
                    |                                                   |
           ------------------ 2                                ------------------ 0
                    |                                                   |
                    |                                                   |
                    |                                                   |
                    3                                                   1

*/

/*********************************************************************************************************
*                                              包含头文件
*********************************************************************************************************/
#include "Gimbal.h"
#include "launcher.h"
#include "protocol_hero.h"
#include "Atti.h"
#include "packet.h"
#include "can_send.h"
#include "DM_motor.h"
#include "VTM.h"

/*********************************************************************************************************
*                                              内部变量
*********************************************************************************************************/
gimbal_t gimbal;
yaw_ctrl_debug_t yaw_ctrl_debug;
vision_t vision_data;   // 给视觉传信息
extern robot_ctrl_info_t robot_ctrl;    // 获取视觉信息
//todo: 图传的
extern uint8_t control_flag;        // 通过状态判断是什么链路

fp32 gyro_pitch = 0.0f;
fp32 gyro_yaw = 0.0f;

uint8_t camera_flag = 0;

//pitch移动速度系数
float pitch_scale = 0.0f;
static fp32 up_pitch_backlash_comp = 0.0f;

// Mechanical posture is kept separately; gimbal.mode may temporarily be AUTO.
static Gimbal_Mode_e gimbal_pose_mode = GIMBAL_UP;
static Gimbal_Mode_e yaw_last_ctrl_mode = GIMBAL_RELAX;
static fp32 yaw_vel_ref = 0.0f;
static fp32 yaw_down_load_scale = 1.0f;
static fp32 yaw_up_load_scale = 1.0f;
static fp32 yaw_angle_err_last = 0.0f;
static fp32 yaw_last_angle_set = 0.0f;
static fp32 yaw_set_vel_filter = 0.0f;
static fp32 yaw_follow_rate_ff = 0.0f;
static fp32 yaw_down_input_slew = 0.0f;
static fp32 gimbal_prone_vw_cmd = 0.0f;
static fp32 yaw_gyro_filter = 0.0f;
static fp32 yaw_plan_angle = 0.0f;
static fp32 yaw_plan_vel = 0.0f;
static fp32 yaw_plan_acc = 0.0f;
static fp32 yaw_torque_last = 0.0f;
static fp32 yaw_ammo_load_ratio = 1.0f;
static uint8_t yaw_adapt_tick = 0;
static uint8_t yaw_referee_last_game_progress = 0U;
static uint8_t yaw_referee_last_shoot_seq = 0U;
static uint8_t yaw_referee_inited = 0U;
static uint8_t pitch_recover_from_relax = 0;
static uint8_t gimbal_up_disable_thermal_lock = 0U;
static Gimbal_Mode_e gimbal_up_disable_saved_pose = GIMBAL_UP;
static uint8_t gimbal_up_disable_enter_pending = 0U;
static uint32_t gimbal_up_disable_down_ready_tick = 0U;
static uint8_t gimbal_up_disable_enter_from_down_like = 0U;
static uint8_t gimbal_up_disable_exit_pending = 0U;
static Gimbal_Mode_e gimbal_up_disable_pending_pose = GIMBAL_UP;
static uint8_t gimbal_prone_enter_pending = 0U;
static Gimbal_Mode_e gimbal_prone_pending_pose = GIMBAL_DOWN;
static uint32_t gimbal_prone_center_ready_tick = 0U;
static uint8_t crazy_cycle_exit_pending = 0U;
static Gimbal_Mode_e crazy_cycle_exit_target_pose = GIMBAL_UP;
static uint32_t crazy_cycle_exit_ready_tick = 0U;
static pid_type_def up_pitch_position_fix;

static pitch_switch_state_e pitch_switch_state = PITCH_SWITCH_IDLE;
static fp32 pitch_switch_up_start_position = 0.0f;
static fp32 pitch_switch_down_start_position = 0.0f;
static fp32 pitch_switch_up_target_position = 0.0f;
static fp32 pitch_switch_down_target_position = 0.0f;
static fp32 pitch_switch_angle_set = PITCH_SWITCH_LEVEL_ANGLE_SET;
static fp32 pitch_switch_abs_angle_set = 0.0f;
static uint32_t pitch_switch_abs_angle_tick = 0U;
static Gimbal_Mode_e pitch_switch_start_mode = GIMBAL_UP;
static Gimbal_Mode_e pitch_switch_target_mode = GIMBAL_UP;

#define GIMBAL_FIRE_CAN1_BASE_HZ        1000U
#define GIMBAL_FIRE_CAN1_ACTIVE_HZ      700U
#define GIMBAL_FIRE_CAN1_RELAX_HZ       500U

uint32_t test1221 = 0;

/*********************************************************************************************************
*                                              内部函数声明
*********************************************************************************************************/
_Noreturn void Gimbal_task(void const*pvParameters);
static void Gimbal_Init(void);
static void Gimbal_Angle_Update(void);
static void Send_Vision_Data(void);
static void Gimbal_Device_Offline_Handle(void);
static void Send_Gimbal_Data(void);

static void Chassis_Mode_Set(void);
static void Gimbal_Mode_Set(void);
static void Camera_Mode_Set(void);

static void Gimbal_Pose_Handle(Gimbal_Mode_e pose);
static void Gimbal_Control(void);

static void Gimbal_Relax_Handle(void);
static void Gimbal_Active_Handle(void);
static void Gimbal_Up_Handle(void);
static void Gimbal_Crazy_Cycle_Handle(void);
static void Gimbal_Down_Handle(void);
static void Gimbal_Auto_Handle(void);
static void Gimbal_Ctrl_Loop_Cal(void);

static float angle_to_position(float angle);
static fp32 Gimbal_Yaw_Loop_Error(fp32 set, fp32 get);
static void Gimbal_Yaw_Reset(void);
static fp32 Gimbal_Yaw_Pose_Scale(void);
static fp32 Gimbal_Yaw_Get_Load_Scale(void);
static void Gimbal_Yaw_Set_Load_Scale(fp32 scale);
static void Gimbal_Yaw_Load_Adapt_Update(fp32 yaw_err, fp32 vel_err, fp32 plan_vel, fp32 gyro_feedback);
static void Gimbal_Yaw_Ammo_Update_From_Referee(uint8_t game_progress, uint8_t shoot_seq);
static fp32 Gimbal_Yaw_Ammo_Ratio(void);
static fp32 Gimbal_Yaw_Ammo_Drive_Scale(void);
static fp32 Gimbal_Yaw_Ammo_Damp_Scale(void);
static fp32 Gimbal_Yaw_Ammo_Max_Torque(void);
static uint8_t Gimbal_Should_Send_Fire_CAN1(void);
static void Gimbal_DM_Send_One(fp32 up_pos, fp32 up_kp, fp32 up_kd, fp32 up_torque,
                               fp32 down_pos, fp32 down_kp, fp32 down_kd, fp32 down_torque,
                               fp32 yaw_kd, fp32 yaw_torque);
static void Gimbal_Up_Pitch_Position_Fix_Reset(void);
static fp32 Gimbal_Up_Pitch_Position_Fix(fp32 target_angle);
static fp32 Gimbal_Up_Pitch_Hold_Torque(fp32 target_angle, uint8_t pitch_switching);
static void Gimbal_Pitch_Build_Cmd(fp32 *up_pos, fp32 *up_kp, fp32 *up_kd, fp32 *up_torque,
                                   fp32 *down_pos, fp32 *down_kp, fp32 *down_kd, fp32 *down_torque);
static void Pitch_Smooth_Init(Motor_Gimbal_t *pitch, fp32 target);
static void Pitch_Smooth_Start_From_Feedback(Motor_Gimbal_t *pitch, fp32 target);
static void Pitch_Smooth_Set_Target(Motor_Gimbal_t *pitch, fp32 target);
static fp32 Pitch_Smooth_Position_Update(Motor_Gimbal_t *pitch);
static uint8_t Gimbal_Pitch_Pose_Busy(void);
static uint8_t Gimbal_Pitch_Chassis_Follow_Ready(void);
static uint8_t Gimbal_Is_Crazy_Cycle_Pose(Gimbal_Mode_e mode);
static uint8_t Gimbal_Is_Up_Like_Pose(Gimbal_Mode_e mode);
static uint8_t Gimbal_Pose_Control_Enabled(void);
static uint8_t Gimbal_Is_Pitch_Pose_Mode(Gimbal_Mode_e mode);
static uint8_t Gimbal_Is_Down_Like_Pose(Gimbal_Mode_e mode);
static uint8_t Gimbal_Is_Same_Mechanical_Pose(Gimbal_Mode_e lhs, Gimbal_Mode_e rhs);
static uint8_t Gimbal_Prone_Enter_Ready(void);
static uint8_t Gimbal_Crazy_Cycle_Exit_Ready(void);
static uint8_t Gimbal_Yaw_Disabled_In_Current_Mode(void);
static void Gimbal_Yaw_Disable_Reset(void);
static void Gimbal_Crazy_Cycle_Exit_Reset(void);
static fp32 Gimbal_Yaw_Get_Center_Target(void);
static Gimbal_Mode_e Pitch_Switch_Infer_Pose_From_Feedback(void);
static fp32 Pitch_Switch_Update_Abs_Angle_Set(void);
static void Pitch_Switch_Reset(void);
static void Pitch_Switch_Start(Gimbal_Mode_e last_mode, Gimbal_Mode_e mode);
static void Pitch_Switch_Update_Targets(fp32 up_target, fp32 down_target);

/**
  * @brief          云台任务
  * @param[in]      pvParameters
  * 初始化，设置模式，实现功能，接受 CAN 信号，云台的具体实现逻辑
  * @retval         返回指针
  */
void Gimbal_task(void const*pvParameters) {
    /* 任务初始化时间 */
    vTaskDelay(GIMBAL_TASK_INIT_TIME);

    /* 云台初始化 */
    Gimbal_Init();
    /* 发射机构初始化 */
    Launcher_Init();

    while(1) {
        test1221++;
        /* 更新遥控器和键鼠信息 */
        update_pc_info();

        Gimbal_Angle_Update();  // 接收更新绝对相对角度值
        Send_Vision_Data();     // 给视觉发信息

        /* 更新板间传输信息 */
        Send_Gimbal_Data();

        /* 模式设置 */
        Chassis_Mode_Set();
        Gimbal_Mode_Set();
        Launcher_Mode_Set();

        /*测试代码*/
        // gimbal.mode = GIMBAL_ACTIVE;

        /* 检测遥控器、电机、电源是否断线 */
        Gimbal_Device_Offline_Handle();

        /* 云台发射机构控制 */
        Gimbal_Control();
        Launcher_Control();
        // Servo_Control();

        /* 检测遥控器、电机、电源是否断线 */
        Gimbal_Device_Offline_Handle();

        /* 控制电机 */
        if(Gimbal_Should_Send_Fire_CAN1()) {
            DJI_Send_Motor_Mapping(CAN_1,
                                   CAN_DJI_MOTOR_0x200_ID,
                                   0,  //201
                                   0,                              //202
                                   launcher.fire_l.give_current,   //203
                                   launcher.fire_r.give_current    //204

            );
        }
        DJI_Send_Motor_Mapping(CAN_2,
                               CAN_DJI_MOTOR_0x200_ID,
                               launcher.trigger.give_current,  //201
                               0,                              //202
                               0,   //203
                               0    //204

        );

        vTaskDelay(GIMBAL_PERIOD);
    }
}

static uint8_t Gimbal_Should_Send_Fire_CAN1(void)
{
    static uint16_t send_acc = GIMBAL_FIRE_CAN1_BASE_HZ;
    static uint8_t last_relax = 0xFFU;
    uint8_t is_relax = (gimbal.mode == GIMBAL_RELAX);
    uint16_t target_hz = is_relax ? GIMBAL_FIRE_CAN1_RELAX_HZ : GIMBAL_FIRE_CAN1_ACTIVE_HZ;

    if(is_relax != last_relax) {
        send_acc = GIMBAL_FIRE_CAN1_BASE_HZ;
        last_relax = is_relax;
    }

    send_acc += target_hz;
    if(send_acc >= GIMBAL_FIRE_CAN1_BASE_HZ) {
        send_acc -= GIMBAL_FIRE_CAN1_BASE_HZ;
        return 1U;
    }

    return 0U;
}

/******************/
/**
  * @brief          云台初始化
  * @param[in]      none
  * 初始化英雄 mode 和 last_mode 的模式为失能，获取 pitch 电机数据
  * 初始化 pitch 轴和 yaw 轴电机的角度环和速度环 PID (包括自瞄)
  * 将 pitch 轴和 yaw 轴电机的电流给定为 0，即失能
  * 初始化时，云台设为未回中状态，校准 pitch 轴和 yaw 轴电机的编码值
  * @retval         返回空
  */
static void Gimbal_Init(void) {
    gimbal.mode = gimbal.last_mode = GIMBAL_RELAX;
    gimbal_pose_mode = GIMBAL_UP;
    gimbal.chassis_mode = CHASSIS_RELAX;

    /* pit 轴电机角度环和速度环PID初始化 */
    pid_init(&gimbal.up_pitch.speed_p, GIMBAL_UP_PITCH_SPEED_MAX_OUT,
             GIMBAL_UP_PITCH_SPEED_MAX_IOUT, GIMBAL_UP_PITCH_SPEED_PID_KP,
             GIMBAL_UP_PITCH_SPEED_PID_KI, GIMBAL_UP_PITCH_SPEED_PID_KD);
    pid_init(&gimbal.up_pitch.angle_p, GIMBAL_UP_PITCH_ANGLE_MAX_OUT,
             GIMBAL_UP_PITCH_ANGLE_MAX_IOUT, GIMBAL_UP_PITCH_ANGLE_PID_KP,
             GIMBAL_UP_PITCH_ANGLE_PID_KI, GIMBAL_UP_PITCH_ANGLE_PID_KD);
    pid_init(&up_pitch_position_fix, GIMBAL_UP_PITCH_FIX_MAX_OUT,
             GIMBAL_UP_PITCH_FIX_IOUT, GIMBAL_UP_PITCH_FIX_KP,
             GIMBAL_UP_PITCH_FIX_KI, GIMBAL_UP_PITCH_FIX_KD);

    pid_init(&gimbal.down_pitch.speed_p, GIMBAL_DOWN_PITCH_SPEED_MAX_OUT,
             GIMBAL_DOWN_PITCH_SPEED_MAX_IOUT, GIMBAL_DOWN_PITCH_SPEED_PID_KP,
             GIMBAL_DOWN_PITCH_SPEED_PID_KI, GIMBAL_DOWN_PITCH_SPEED_PID_KD);
    pid_init(&gimbal.down_pitch.angle_p, GIMBAL_DOWN_PITCH_ANGLE_MAX_OUT,
             GIMBAL_DOWN_PITCH_ANGLE_MAX_IOUT, GIMBAL_DOWN_PITCH_ANGLE_PID_KP,
             GIMBAL_DOWN_PITCH_ANGLE_PID_KI, GIMBAL_DOWN_PITCH_ANGLE_PID_KD);

    /* yaw 轴电机角度环和速度环PID初始化 */
    pid_init(&gimbal.yaw.speed_p, GIMBAL_YAW_SPEED_MAX_OUT,
             GIMBAL_YAW_SPEED_MAX_IOUT, GIMBAL_YAW_SPEED_PID_KP,
             GIMBAL_YAW_SPEED_PID_KI, GIMBAL_YAW_SPEED_PID_KD);
    pid_init(&gimbal.yaw.angle_p, GIMBAL_YAW_ANGLE_MAX_OUT,
             GIMBAL_YAW_ANGLE_MAX_IOUT, GIMBAL_YAW_ANGLE_PID_KP,
             GIMBAL_YAW_ANGLE_PID_KI, GIMBAL_YAW_ANGLE_PID_KD);

    gimbal.yaw.motor_measure.offset_position = YAW_OFFSET_POSITION;
    gimbal.up_pitch.motor_measure.offset_position = UP_PITCH_OFFSET_POSITION;
    gimbal.down_pitch.motor_measure.offset_position = DOWN_PITCH_OFFSET_POSITION;
    Pitch_Smooth_Init(&gimbal.up_pitch, UP_PITCH_OFFSET_POSITION);
    Pitch_Smooth_Init(&gimbal.down_pitch, DOWN_PITCH_OFFSET_POSITION);

    /* 使pitch失能保持当前状态 */
    // gimbal.pitch.relative_up_down_set = gimbal.pitch.relative_up_down_get;

    gimbal.up_pitch.DM_kd = UP_PITCH_KD;
    gimbal.up_pitch.DM_kp = UP_PITCH_KP;

    gimbal.down_pitch.DM_kd = DOWN_PITCH_KD;
    gimbal.down_pitch.DM_kp = DOWN_PITCH_KP;

    gimbal.yaw.DM_kd = YAW_KD;
    gimbal.yaw.DM_kp = YAW_KP;

    gimbal.yaw.absolute_angle_get = gimbal.yaw.absolute_angle_set;

    //低通滤波初始化
    first_order_filter_init(&gimbal.mouse_in_x, 1, 40);
    first_order_filter_init(&gimbal.mouse_in_y, 1, 10);

    first_order_filter_init(&gimbal.auto_pitch, 1, 15);
    first_order_filter_init(&gimbal.auto_yaw[0], 1, 15);
    first_order_filter_init(&gimbal.auto_yaw[1], 1, 15);
    first_Kalman_Create(&gimbal.filter_autoYaw,1,20);  //卡尔曼滤波
    first_order_filter_init(&gimbal.filter_pitch_gyro_in, 1, 20);
    first_order_filter_init(&gimbal.filter_yaw_gyro_in, 5, 30);

    first_order_filter_init(&gimbal.up_pitch_first_order_set, 1, GIMBAL_UP_PITCH_FIX_FILTER_NUM);
    first_order_filter_init(&gimbal.up_pitch_current_first_order_set, 1, UP_PITCH_INPUT_FILTER_NUM);
    first_order_filter_init(&gimbal.down_pitch_first_order_set, 0.0f, 500);
    first_order_filter_init(&gimbal.down_pitch_current_first_order_set, 5, 30);

}




/**
  * @brief          云台角度更新，与视觉通讯
  * @param[in]      none
  * 根据惯性导航系统的数据更新云台的角度信息，并将相应的信息发送到视觉系统。
  * 其中，角度信息通过陀螺仪测量、电机编码器数据以及惯性导航系统的角度信息等获得。
  * @retval         返回空
  */
static void Gimbal_Angle_Update(void) {
    // 更新云台俯仰角度信息
    gimbal.up_pitch.absolute_angle_get=INS_angle[2]*MOTOR_RAD_TO_ANGLE;
    gimbal.up_pitch.relative_angle_get = Motor_Ecd_To_Angle_Change((uint16_t)gimbal.up_pitch.motor_measure.position,
                                                             (uint16_t)gimbal.up_pitch.motor_measure.offset_position);
    gimbal.yaw.absolute_angle_get=INS_angle[0]*MOTOR_RAD_TO_ANGLE;
    gimbal.yaw.relative_angle_get = Motor_Ecd_To_Angle_Change((uint16_t)gimbal.yaw.motor_measure.position,
                                                             (uint16_t)gimbal.yaw.motor_measure.offset_position);

    // 更新云台底盘相对角 | get
    // gimbal.pitch.relative_up_down_get = gimbal.pitch.absolute_angle_get + gimbal.pitch.absolute_angle_get_down;

    // 更新云台偏航角度信息,通过陀螺仪测量的偏航和俯仰角速度
    gyro_yaw = INS_gyro[2]*MOTOR_RAD_TO_ANGLE;
    gyro_pitch = -INS_gyro[0]*MOTOR_RAD_TO_ANGLE;

    gimbal.absolute_gyro_yaw = (fp32) INS_gyro[2];
    gimbal.absolute_gyro_pitch = (fp32) INS_gyro[0];
}


static void Send_Vision_Data(void) {
    // 7:蓝 107:红(注意:ID的7和107均为队内定义，与裁判系统定义逻辑相反)
    if (Referee.GameRobotStat.robot_id < 10) vision_data.id = 7;
    else vision_data.id = 107;
    /* 给视觉发开自瞄 */
    if (gimbal.mode == GIMBAL_AUTO) vision_data.mode = 0x21;
    else vision_data.mode = 0;

    vision_data.pitch = gimbal.up_pitch.absolute_angle_get;
    vision_data.yaw   = gimbal.yaw.absolute_angle_get;
    vision_data.roll = (fp32) INS_angle[1] * MOTOR_RAD_TO_ANGLE;

    for (int i = 0; i < 4; ++i) {
        vision_data.quaternion[i] = INS_quat[i];
    }

    vision_data.shoot_speed = Referee.ShootData.bullet_speed;
    rm_queue_data(VISION_ID, &vision_data, sizeof(vision_t));
}


static void Gimbal_Device_Offline_Handle(void) {
    if(detect_list[DETECT_REMOTE].status == OFFLINE &&
       detect_list[DETECT_VIDEO_TRANSIMITTER].status == OFFLINE) {
        gimbal.mode = GIMBAL_RELAX;
        gimbal.last_mode = GIMBAL_RELAX;
        Launcher_Relax_Handle();
        Gimbal_Relax_Handle();
        gimbal.chassis_mode = CHASSIS_RELAX;
    }

    if (detect_list[DETECT_GIMBAL_UP_PITCH].status == OFFLINE) {
        gimbal.up_pitch.DM_MIT_position = gimbal.up_pitch.motor_measure.offset_position;
    }

    if (detect_list[DETECT_GIMBAL_DOWN_PITCH].status == OFFLINE) {
        gimbal.down_pitch.DM_MIT_position = gimbal.down_pitch.motor_measure.offset_position;
    }

    if (detect_list[DETECT_GIMBAL_YAW].status == OFFLINE) {
        gimbal.yaw.DM_MIT_torque = 0.0f;
    }

    if (detect_list[DETECT_LAUNCHER_3508_FIRE_L].status == OFFLINE) {
        launcher.fire_l.give_current = 0;
    }
    if (detect_list[DETECT_LAUNCHER_3508_FIRE_R].status == OFFLINE) {
        launcher.fire_r.give_current = 0;
    }
    if (detect_list[DETECT_LAUNCHER_3508_TRIGGER].status == OFFLINE) {
        launcher.trigger.give_current = 0;
    }

}


static void Send_Gimbal_Data(void) {
    Chassis_Mode_e chassis_mode = gimbal.chassis_mode;
    int16_t chassis_z_cmd = rc_ctrl.rc.ch[2];
    fp32 chassis_vw_pc_cmd = gimbal_prone_vw_cmd;
    uint8_t down_pose_req = (KeyBoard.CTRL.click_flag == 1) ||
                            (switch_is_up(rc_ctrl.rc.s[RC_s_R]) &&
                             switch_is_mid(rc_ctrl.rc.s[RC_s_L]));

    if ((Gimbal_Is_Down_Like_Pose(gimbal_pose_mode) ||
         (down_pose_req && !Gimbal_Is_Crazy_Cycle_Pose(gimbal_pose_mode))) &&
        !switch_is_down(rc_ctrl.rc.s[RC_s_R])) {
        chassis_mode = CHASSIS_FOLLOW_GIMBAL;
        gimbal.chassis_mode = CHASSIS_FOLLOW_GIMBAL;
    }

    if (Gimbal_Is_Crazy_Cycle_Pose(gimbal_pose_mode) || crazy_cycle_exit_pending) {
        chassis_mode = CHASSIS_ONLY;
        gimbal.chassis_mode = CHASSIS_ONLY;
        chassis_z_cmd = 0;
        chassis_vw_pc_cmd = 0.0f;
    }

    if (!Gimbal_Is_Down_Like_Pose(gimbal_pose_mode) &&
        (chassis_mode == CHASSIS_FOLLOW_GIMBAL ||
         chassis_mode == CHASSIS_SPIN_R ||
         chassis_mode == CHASSIS_SPIN_L) &&
        !Gimbal_Pitch_Chassis_Follow_Ready()) {
        chassis_mode = CHASSIS_ONLY;
    }

    Send_Chassis_Speed(rc_ctrl.rc.ch[1], rc_ctrl.rc.ch[0], chassis_z_cmd, chassis_mode);
    Send_keyboard(gimbal.mode, launcher.fire_mode, rc_ctrl.rc.s[RC_s_R],rc_ctrl.rc.s[RC_s_L],(int16_t)gimbal.up_pitch.relative_angle_get);
    vTaskDelay(GIMBAL_PERIOD);
    Send_Control(KeyBoard.W.status, KeyBoard.A.status, KeyBoard.S.status, KeyBoard.D.status,
                 gimbal.yaw.relative_angle_get,
                 yaw_follow_rate_ff,
                 chassis_vw_pc_cmd);

    // uint8_t safe_fire_command = (robot_ctrl.fire_command != 0) ? 1 : 0;
    // Send_CTRL(KeyBoard.SHIFT.status,KeyBoard.Mouse_l.status,KeyBoard.Q.click_flag,KeyBoard.Mouse_r.status, vtm_rx_data.trigger, robot_ctrl.fire_command);
    Send_CTRL(KeyBoard.SHIFT.status);
    Send_UI_Telemetry(DM_Up_Pitch_T_Rotor, robot_ctrl.distance);
    vTaskDelay(GIMBAL_PERIOD);
}

/**
  * @brief          设置底盘模式
  * @param[in]      none
  * 左下 右下底盘失能CHASSIS_RELAX   左不下 右下底盘独立CHASSIS_ONLY
  * 右中底盘跟随云台CHASSIS_FOLLOW_GIMBAL 右上 E小陀螺CHASSIS_SPIN
  * 按下 Q 键，底盘刹车   UI更新底盘模式
  * @retval         返回空
  */
static void Chassis_Mode_Set(void) {
    //根据遥控器设置底盘模式
    uint8_t sw_l = rc_ctrl.rc.s[RC_s_L];
    uint8_t sw_r = rc_ctrl.rc.s[RC_s_R];

    if (Gimbal_Is_Crazy_Cycle_Pose(gimbal_pose_mode) || crazy_cycle_exit_pending) {
        gimbal.chassis_mode = CHASSIS_ONLY;
        return;
    }

    // 左下+右下：底盘失能
    if (switch_is_down(sw_l) && switch_is_down(sw_r)) {
        gimbal.chassis_mode = CHASSIS_RELAX;
        return;
    }

    // 左不下+右下：底盘独立
    if (!switch_is_down(sw_l) && switch_is_down(sw_r)) {
        gimbal.chassis_mode = CHASSIS_ONLY;
        return;
    }

    if (Gimbal_Is_Down_Like_Pose(gimbal_pose_mode) || Gimbal_Pitch_Pose_Busy()) {
        gimbal.chassis_mode = CHASSIS_FOLLOW_GIMBAL;
        return;
    }

    // 右中：抬头时底盘跟随云台（键鼠E可触发右小陀螺）
    if (switch_is_mid(sw_r)) {
        if (KeyBoard.E.click_flag == KEY_DOWN) gimbal.chassis_mode = CHASSIS_SPIN_R;
        else gimbal.chassis_mode = CHASSIS_FOLLOW_GIMBAL;
        return;
    }

    if (switch_is_up(sw_r) && switch_is_down(sw_l)) {
        // 右上+左下：默认是左小陀螺；但低头状态下强制不进小陀螺
        if (Gimbal_Is_Down_Like_Pose(gimbal_pose_mode)) {
            gimbal.chassis_mode = CHASSIS_FOLLOW_GIMBAL;
        } else {
            gimbal.chassis_mode = CHASSIS_SPIN_L;
        }
        return;
    }

    if (switch_is_up(sw_r) && (switch_is_mid(sw_l) || switch_is_up(sw_l))) {
        // 右上+左中/左上：低头时跟随
        gimbal.chassis_mode = CHASSIS_FOLLOW_GIMBAL;
        return;

    }
}

/**
  * @brief          云台模式设置（获取遥控器信息，判断模式）
  * @param[in]      none
  * 右边拨杆向下云台失能
  * 右边拨杆向中和向上得到遥控器对云台电机的控制 —— 云台回中，自瞄判定
  * @retval         返回空
  */
static void Gimbal_Mode_Set(void) {
    static uint8_t last_remote_down_req = 0U;
    static uint8_t remote_down_latched = 0U;
    Gimbal_Mode_e last_control_mode = gimbal.mode;
    Gimbal_Mode_e last_pose_mode = gimbal_pose_mode;
    Gimbal_Mode_e requested_pose = gimbal_pose_mode;
    uint8_t pose_enabled = Gimbal_Pose_Control_Enabled();
    uint8_t remote_down_req = pose_enabled &&
                              switch_is_up(rc_ctrl.rc.s[RC_s_R]) &&
                              switch_is_mid(rc_ctrl.rc.s[RC_s_L]);
    uint8_t auto_cond =
        ((KeyBoard.Mouse_r.status == KEY_PRESS && robot_ctrl.target_lock == 0x31 &&
          (detect_list[DETECT_AUTO_AIM].status == ONLINE)) ||
         (rc_ctrl.rc.ch[AUTO_CHANNEL] > 50 && robot_ctrl.target_lock == 0x31 &&
          (detect_list[DETECT_AUTO_AIM].status == ONLINE)));

    if (!gimbal_up_disable_thermal_lock &&
        DM_Up_Pitch_T_Rotor > GIMBAL_UP_DISABLE_ENTER_TEMP) {
        gimbal_up_disable_thermal_lock = 1U;
        gimbal_up_disable_saved_pose =
            Gimbal_Is_Pitch_Pose_Mode(gimbal_pose_mode) ? gimbal_pose_mode : GIMBAL_UP;
        gimbal_up_disable_enter_pending = !Gimbal_Is_Crazy_Cycle_Pose(gimbal_pose_mode);
        gimbal_up_disable_down_ready_tick = 0U;
        gimbal_up_disable_enter_from_down_like = Gimbal_Is_Down_Like_Pose(gimbal_pose_mode);
        gimbal_up_disable_exit_pending = 0U;
        remote_down_latched = 0U;
    } else if (gimbal_up_disable_thermal_lock &&
               DM_Up_Pitch_T_Rotor < GIMBAL_UP_DISABLE_EXIT_TEMP) {
        gimbal_up_disable_thermal_lock = 0U;
        gimbal_up_disable_enter_pending = 0U;
        gimbal_up_disable_down_ready_tick = 0U;
        gimbal_up_disable_enter_from_down_like = 0U;
        requested_pose = gimbal_up_disable_saved_pose;
    }

    if (KeyBoard.F.click_flag == 1) {
        KeyBoard.F.click_flag = 0;
        if (!gimbal_up_disable_thermal_lock) {
            remote_down_latched = 0U;
            gimbal_prone_enter_pending = 0U;
            gimbal_prone_center_ready_tick = 0U;
            requested_pose = GIMBAL_CRAZY_CYCLE;
            gimbal.chassis_mode = CHASSIS_ONLY;
            gimbal_up_disable_enter_pending = 0U;
            gimbal_up_disable_down_ready_tick = 0U;
            gimbal_up_disable_enter_from_down_like = 0U;
            Gimbal_Crazy_Cycle_Exit_Reset();
        }
    }
    else if (KeyBoard.G.click_flag == 1) {
        KeyBoard.G.click_flag = 0;
        if (!gimbal_up_disable_thermal_lock) {
            remote_down_latched = 0U;
            requested_pose = GIMBAL_UP;
            gimbal_prone_enter_pending = 0U;
            gimbal_prone_center_ready_tick = 0U;
            gimbal_up_disable_enter_pending = 0U;
            gimbal_up_disable_down_ready_tick = 0U;
            gimbal_up_disable_enter_from_down_like = 0U;
        }
    }
    else if (KeyBoard.Mouse_m.click_flag == 1) {
        KeyBoard.Mouse_m.click_flag = 0;
        if (!gimbal_up_disable_thermal_lock) {
            remote_down_latched = 0U;
            if (Gimbal_Is_Crazy_Cycle_Pose(gimbal_pose_mode) || crazy_cycle_exit_pending) {
                gimbal_prone_enter_pending = 0U;
                gimbal_prone_center_ready_tick = 0U;
                requested_pose = GIMBAL_DOWN;
            } else {
                gimbal_prone_enter_pending = 1U;
                gimbal_prone_pending_pose = GIMBAL_DOWN;
                gimbal_prone_center_ready_tick = 0U;
                requested_pose = gimbal_pose_mode;
                gimbal.chassis_mode = CHASSIS_FOLLOW_GIMBAL;
            }
            gimbal_up_disable_enter_pending = 0U;
            gimbal_up_disable_down_ready_tick = 0U;
            gimbal_up_disable_enter_from_down_like = 0U;
        }
    }
    else if (KeyBoard.CTRL.click_flag == 1) {
        KeyBoard.CTRL.click_flag = 0;
        if (!gimbal_up_disable_thermal_lock) {
            remote_down_latched = 0U;
            if (Gimbal_Is_Crazy_Cycle_Pose(gimbal_pose_mode) || crazy_cycle_exit_pending) {
                gimbal_prone_enter_pending = 0U;
                gimbal_prone_center_ready_tick = 0U;
                requested_pose = GIMBAL_UP_DISABLE;
            } else {
                gimbal_prone_enter_pending = 1U;
                gimbal_prone_pending_pose = GIMBAL_UP_DISABLE;
                gimbal_prone_center_ready_tick = 0U;
                requested_pose = gimbal_pose_mode;
                gimbal.chassis_mode = CHASSIS_FOLLOW_GIMBAL;
            }
            gimbal_up_disable_enter_pending = 0U;
            gimbal_up_disable_down_ready_tick = 0U;
            gimbal_up_disable_enter_from_down_like = Gimbal_Is_Down_Like_Pose(gimbal_pose_mode);
        }
    }
    else if (!pose_enabled) {
        Gimbal_Crazy_Cycle_Exit_Reset();
        gimbal_prone_enter_pending = 0U;
        gimbal_prone_center_ready_tick = 0U;
        gimbal_up_disable_enter_pending = 0U;
        gimbal_up_disable_down_ready_tick = 0U;
        gimbal_up_disable_enter_from_down_like = 0U;
        remote_down_latched = 0U;
    }
    else if (!gimbal_up_disable_thermal_lock &&
             remote_down_req && !last_remote_down_req) {
        remote_down_latched = 1U;
        if (Gimbal_Is_Crazy_Cycle_Pose(gimbal_pose_mode) || crazy_cycle_exit_pending) {
            gimbal_prone_enter_pending = 0U;
            gimbal_prone_center_ready_tick = 0U;
            requested_pose = GIMBAL_UP_DISABLE;
        } else {
            gimbal_prone_enter_pending = 1U;
            gimbal_prone_pending_pose = GIMBAL_UP_DISABLE;
            gimbal_prone_center_ready_tick = 0U;
            requested_pose = gimbal_pose_mode;
        }
        gimbal_up_disable_enter_pending = 0U;
        gimbal_up_disable_down_ready_tick = 0U;
        gimbal_up_disable_enter_from_down_like = Gimbal_Is_Down_Like_Pose(gimbal_pose_mode);
    }
    else if (!gimbal_up_disable_thermal_lock &&
             remote_down_latched && !switch_is_up(rc_ctrl.rc.s[RC_s_R])) {
        remote_down_latched = 0U;
        gimbal_prone_enter_pending = 0U;
        gimbal_prone_center_ready_tick = 0U;
        requested_pose = GIMBAL_UP;
        gimbal_up_disable_enter_pending = 0U;
        gimbal_up_disable_down_ready_tick = 0U;
        gimbal_up_disable_enter_from_down_like = 0U;
    }

    last_remote_down_req = remote_down_req;

    if (gimbal_up_disable_thermal_lock) {
        gimbal_prone_enter_pending = 0U;
        gimbal_prone_center_ready_tick = 0U;
        requested_pose = GIMBAL_UP_DISABLE;
    }

    if (pose_enabled &&
        Gimbal_Is_Crazy_Cycle_Pose(gimbal_pose_mode) &&
        requested_pose != GIMBAL_CRAZY_CYCLE) {
        crazy_cycle_exit_pending = 1U;
        crazy_cycle_exit_target_pose = requested_pose;
        crazy_cycle_exit_ready_tick = 0U;
        requested_pose = GIMBAL_CRAZY_CYCLE;
        gimbal_prone_enter_pending = 0U;
        gimbal_prone_center_ready_tick = 0U;
        gimbal_up_disable_enter_pending = 0U;
        gimbal_up_disable_down_ready_tick = 0U;
    }

    if (crazy_cycle_exit_pending) {
        if (!pose_enabled) {
            Gimbal_Crazy_Cycle_Exit_Reset();
        } else {
            if (requested_pose != GIMBAL_CRAZY_CYCLE &&
                !Gimbal_Is_Crazy_Cycle_Pose(requested_pose)) {
                crazy_cycle_exit_target_pose = requested_pose;
            }

            if (Gimbal_Crazy_Cycle_Exit_Ready()) {
                Gimbal_Mode_e crazy_target = crazy_cycle_exit_target_pose;
                Gimbal_Crazy_Cycle_Exit_Reset();
                if (crazy_target == GIMBAL_UP_DISABLE) {
                    gimbal_up_disable_enter_pending = 1U;
                    gimbal_up_disable_down_ready_tick = 0U;
                    gimbal_up_disable_enter_from_down_like = Gimbal_Is_Down_Like_Pose(gimbal_pose_mode);
                    requested_pose = GIMBAL_DOWN;
                } else {
                    requested_pose = crazy_target;
                }
            } else {
                requested_pose = GIMBAL_CRAZY_CYCLE;
            }
        }
    }

    if (gimbal_prone_enter_pending) {
        gimbal.chassis_mode = CHASSIS_FOLLOW_GIMBAL;
        if (Gimbal_Prone_Enter_Ready()) {
            requested_pose = gimbal_prone_pending_pose;
            if (gimbal_prone_pending_pose == GIMBAL_UP_DISABLE) {
                gimbal_up_disable_enter_pending = 1U;
                gimbal_up_disable_down_ready_tick = 0U;
                gimbal_up_disable_enter_from_down_like = Gimbal_Is_Down_Like_Pose(gimbal_pose_mode);
            }
            gimbal_prone_enter_pending = 0U;
            gimbal_prone_center_ready_tick = 0U;
        } else {
            requested_pose = gimbal_pose_mode;
        }
    }

    if (gimbal_up_disable_enter_pending) {
        uint32_t settle_ms = GIMBAL_UP_DISABLE_TRANSITION_SETTLE_MS +
                             (gimbal_up_disable_enter_from_down_like ?
                              GIMBAL_UP_DISABLE_DOWN_TO_DISABLE_EXTRA_MS : 0U);
        if (Gimbal_Pitch_Pose_Busy()) {
            gimbal_up_disable_down_ready_tick = 0U;
            requested_pose = GIMBAL_DOWN;
        } else if (gimbal_up_disable_down_ready_tick == 0U) {
            gimbal_up_disable_down_ready_tick = HAL_GetTick();
            requested_pose = GIMBAL_DOWN;
        } else if ((HAL_GetTick() - gimbal_up_disable_down_ready_tick) < settle_ms) {
            requested_pose = GIMBAL_DOWN;
        } else {
            gimbal_up_disable_enter_pending = 0U;
            gimbal_up_disable_down_ready_tick = 0U;
            gimbal_up_disable_enter_from_down_like = 0U;
            requested_pose = GIMBAL_UP_DISABLE;
        }
    }

    if (gimbal_pose_mode == GIMBAL_UP_DISABLE &&
        requested_pose != GIMBAL_UP_DISABLE &&
        !gimbal_up_disable_exit_pending) {
        gimbal_up_disable_exit_pending = 1U;
        gimbal_up_disable_pending_pose = requested_pose;
        requested_pose = GIMBAL_DOWN;
    } else if (gimbal_up_disable_exit_pending) {
        if (Gimbal_Pitch_Pose_Busy()) {
            requested_pose = GIMBAL_DOWN;
        } else {
            requested_pose = gimbal_up_disable_pending_pose;
            gimbal_up_disable_exit_pending = 0U;
        }
    }

    gimbal_pose_mode = requested_pose;

    if ((last_pose_mode != GIMBAL_UP_DISABLE && gimbal_pose_mode == GIMBAL_UP_DISABLE) ||
        (last_pose_mode == GIMBAL_UP_DISABLE && gimbal_pose_mode != GIMBAL_UP_DISABLE)) {
        Gimbal_Yaw_Disable_Reset();
    }

    Camera_Mode_Set();

    if (!pose_enabled && !crazy_cycle_exit_pending) {
        gimbal.mode = GIMBAL_RELAX;
    } else if (Gimbal_Is_Up_Like_Pose(gimbal_pose_mode) &&
               auto_cond && !gimbal_up_disable_thermal_lock) {
        gimbal.mode = GIMBAL_AUTO;
    } else {
        gimbal.mode = gimbal_pose_mode;
    }

    if (gimbal.mode == GIMBAL_AUTO) {
        gimbal.last_mode = gimbal_pose_mode;
    } else {
        gimbal.last_mode = last_control_mode;
    }

    if (!gimbal_up_disable_enter_pending &&
        last_control_mode == GIMBAL_RELAX &&
        Gimbal_Is_Pitch_Pose_Mode(gimbal_pose_mode) &&
        gimbal.mode != GIMBAL_RELAX) {
        pitch_recover_from_relax = 1;
        Gimbal_Up_Pitch_Position_Fix_Reset();
        Gimbal_Mode_e inferred_pose = Pitch_Switch_Infer_Pose_From_Feedback();
        if (!Gimbal_Is_Same_Mechanical_Pose(inferred_pose, gimbal_pose_mode)) {
            Pitch_Switch_Start(inferred_pose, gimbal_pose_mode);
        }
    } else if (!gimbal_up_disable_enter_pending &&
               Gimbal_Is_Pitch_Pose_Mode(last_pose_mode) &&
               Gimbal_Is_Pitch_Pose_Mode(gimbal_pose_mode) &&
               !Gimbal_Is_Same_Mechanical_Pose(last_pose_mode, gimbal_pose_mode) &&
               gimbal.mode != GIMBAL_RELAX) {
        Pitch_Switch_Start(last_pose_mode, gimbal_pose_mode);
    }
}


/**
  * @brief          云台模式设置实现(四种模式)
  * @param[in]      none
  * 云台失能，云台回中，云台控制，云台自瞄
  * @retval         返回空
  */
static void Camera_Mode_Set(void)
{
    static uint8_t last_auto_switch_state = 0;
    uint8_t current_auto_switch_state = (rc_ctrl.rc.ch[AUTO_CHANNEL] < -50) ? 1 : 0;

    if (current_auto_switch_state == 1 && last_auto_switch_state == 0) {
        camera_flag = !camera_flag;
    }
    last_auto_switch_state = current_auto_switch_state;

    if (control_flag == VT_ONLINE) {
        if (KeyBoard.Z.status == KEY_CLICK) {
            vision_data.camera_id = 0;
        } else if (KeyBoard.X.status == KEY_CLICK) {
            vision_data.camera_id = 1;
        }
    } else {
        vision_data.camera_id = camera_flag ? 1 : 0;
    }
}

static void Gimbal_Pose_Handle(Gimbal_Mode_e pose)
{
    switch (pose) {
    case GIMBAL_CRAZY_CYCLE:
        Gimbal_Crazy_Cycle_Handle();
        break;
    case GIMBAL_DOWN:
    case GIMBAL_UP_DISABLE:
        Gimbal_Down_Handle();
        break;
    case GIMBAL_UP:
    default:
        Gimbal_Up_Handle();
        break;
    }
}

static void Gimbal_Control(void) {
    switch (gimbal.mode) {
    case GIMBAL_RELAX:            //云台失能
        Gimbal_Relax_Handle();
        break;


    case GIMBAL_UP: {
            Gimbal_Pose_Handle(GIMBAL_UP);
            Gimbal_Active_Handle();  //得到遥控器信息云台控制目标值
            Gimbal_DM_Ctrl_Loop_Cal();  //云台闭环PID计算
    } break;

    case GIMBAL_CRAZY_CYCLE: {
            Gimbal_Pose_Handle(GIMBAL_CRAZY_CYCLE);
            Gimbal_Active_Handle();  //得到遥控器信息云台控制目标值
            Gimbal_DM_Ctrl_Loop_Cal();  //云台闭环PID计算
    }break;

    case GIMBAL_DOWN:
    case GIMBAL_UP_DISABLE: {
            Gimbal_Pose_Handle(GIMBAL_DOWN);
            Gimbal_Active_Handle();  //得到遥控器信息云台控制目标值
            Gimbal_DM_Ctrl_Loop_Cal();  //云台闭环PID计算

    } break;

    case GIMBAL_AUTO: {          //云台自瞄模式
            Gimbal_Pose_Handle(gimbal_pose_mode);
            Gimbal_Auto_Handle();
            Gimbal_DM_Ctrl_Loop_Cal();  //云台闭环PID计算
    } break;

    default:
        break;
    }
}


/**
  * @brief          云台失能模式(四个电机)
  * @param[in]      none
  * pitch 轴，左右摩擦轮，主动单发的给定电流设置为 0
  * @retval         返回空
  */
void Gimbal_Relax_Handle(void) {
    // CAN1 fire motors are rate-limited in Gimbal_task to keep CAN1 load down.
    // DJI_Send_Motor_Mapping(CAN_1,CAN_DJI_MOTOR_0x1FF_ID,0,0,0,0);
    // gimbal.pitch.relative_up_down_set = gimbal.pitch.relative_up_down_get;
    //TODO: 云台回中函数
    gimbal.yaw.absolute_angle_set = gimbal.yaw.absolute_angle_get;

    gimbal.yaw.DM_MIT_speed = 0.0f;
    gimbal.yaw.DM_MIT_torque = 0.0f;
    Gimbal_Yaw_Reset();
    Pitch_Switch_Reset();
    Gimbal_Up_Pitch_Position_Fix_Reset();

    gimbal.up_pitch.DM_MIT_position = gimbal.up_pitch.motor_measure.position;
    gimbal.up_pitch.absolute_angle_set = 0.0f;
    Pitch_Smooth_Init(&gimbal.up_pitch, gimbal.up_pitch.motor_measure.offset_position);
    Pitch_Smooth_Init(&gimbal.down_pitch, gimbal.down_pitch.motor_measure.offset_position);

    Gimbal_DM_Send_One(gimbal.up_pitch.DM_MIT_position, 0.0f, 0.0f, 0.0f,
                       gimbal.down_pitch.motor_measure.offset_position, 0.0f, 0.0f, 0.0f,
                       0.0f, 0.0f);
    Launcher_Relax_Handle();
}


//一键掉头
static void gimbal_turn_back_judge(void) {
    if(KeyBoard.R.click_flag == 1){
        KeyBoard.R.click_flag = 0;
        gimbal.yaw.absolute_angle_set+=180;
    }
}

/* Pitch posture switch helpers */
static uint8_t Gimbal_Pitch_Pose_Busy(void)
{
    return (pitch_switch_state != PITCH_SWITCH_IDLE) ||
           !gimbal.up_pitch.smooth_arrived ||
           !gimbal.down_pitch.smooth_arrived;
}

static uint8_t Gimbal_Pitch_Chassis_Follow_Ready(void)
{
    static uint32_t ready_start_tick = 0U;
    static uint8_t follow_blocked = 1U;
    static Gimbal_Mode_e last_pose_mode = GIMBAL_RELAX;
    static Gimbal_Mode_e last_gimbal_mode = GIMBAL_RELAX;
    uint8_t pitch_busy = Gimbal_Pitch_Pose_Busy();
    uint8_t pose_changed = (last_pose_mode != gimbal_pose_mode);
    uint8_t recover_from_relax = (last_gimbal_mode == GIMBAL_RELAX &&
                                  gimbal.mode != GIMBAL_RELAX);

    if (gimbal.mode == GIMBAL_RELAX) {
        follow_blocked = 1U;
        ready_start_tick = 0U;
        last_pose_mode = gimbal_pose_mode;
        last_gimbal_mode = gimbal.mode;
        return 0U;
    }

    if (pose_changed || recover_from_relax || pitch_busy) {
        follow_blocked = 1U;
        ready_start_tick = 0U;
    }

    last_pose_mode = gimbal_pose_mode;
    last_gimbal_mode = gimbal.mode;

    if (!follow_blocked) {
        return 1U;
    }

    if (pitch_busy) {
        ready_start_tick = 0U;
        return 0U;
    }

    if (ready_start_tick == 0U) {
        ready_start_tick = HAL_GetTick();
        return 0U;
    }

    if ((HAL_GetTick() - ready_start_tick) >= PITCH_CHASSIS_READY_DELAY_MS) {
        follow_blocked = 0U;
        return 1U;
    }

    return 0U;
}

static uint8_t Gimbal_Is_Crazy_Cycle_Pose(Gimbal_Mode_e mode)
{
    return mode == GIMBAL_CRAZY_CYCLE;
}

static uint8_t Gimbal_Is_Up_Like_Pose(Gimbal_Mode_e mode)
{
    return (mode == GIMBAL_UP || Gimbal_Is_Crazy_Cycle_Pose(mode));
}

static uint8_t Gimbal_Pose_Control_Enabled(void)
{
    return !switch_is_down(rc_ctrl.rc.s[RC_s_R]);
}

static uint8_t Gimbal_Is_Pitch_Pose_Mode(Gimbal_Mode_e mode)
{
    return (Gimbal_Is_Up_Like_Pose(mode) ||
            mode == GIMBAL_DOWN ||
            mode == GIMBAL_UP_DISABLE);
}

static uint8_t Gimbal_Is_Down_Like_Pose(Gimbal_Mode_e mode)
{
    return (mode == GIMBAL_DOWN || mode == GIMBAL_UP_DISABLE);
}

static uint8_t Gimbal_Is_Same_Mechanical_Pose(Gimbal_Mode_e lhs, Gimbal_Mode_e rhs)
{
    if (lhs == rhs) {
        return 1U;
    }

    if (Gimbal_Is_Down_Like_Pose(lhs) && Gimbal_Is_Down_Like_Pose(rhs)) {
        return 1U;
    }

    if (Gimbal_Is_Up_Like_Pose(lhs) && Gimbal_Is_Up_Like_Pose(rhs)) {
        return 1U;
    }

    return 0U;
}

static uint8_t Gimbal_Prone_Enter_Ready(void)
{
    fp32 yaw_abs_err = fabsf(gimbal.yaw.relative_angle_get);

    if (yaw_abs_err > GIMBAL_PRONE_ENTER_YAW_ERR) {
        gimbal_prone_center_ready_tick = 0U;
        return 0U;
    }

    if (gimbal_prone_center_ready_tick == 0U) {
        gimbal_prone_center_ready_tick = HAL_GetTick();
        return 0U;
    }

    return (HAL_GetTick() - gimbal_prone_center_ready_tick) >= GIMBAL_PRONE_ENTER_STABLE_MS;
}

static uint8_t Gimbal_Crazy_Cycle_Exit_Ready(void)
{
    fp32 yaw_abs_err = fabsf(gimbal.yaw.relative_angle_get);

    if (yaw_abs_err > CRAZY_CYCLE_EXIT_YAW_ERR) {
        crazy_cycle_exit_ready_tick = 0U;
        return 0U;
    }

    if (crazy_cycle_exit_ready_tick == 0U) {
        crazy_cycle_exit_ready_tick = HAL_GetTick();
        return 0U;
    }

    return (HAL_GetTick() - crazy_cycle_exit_ready_tick) >= CRAZY_CYCLE_EXIT_STABLE_MS;
}

static uint8_t Gimbal_Yaw_Disabled_In_Current_Mode(void)
{
    return gimbal_pose_mode == GIMBAL_UP_DISABLE;
}

static void Gimbal_Yaw_Disable_Reset(void)
{
    gimbal.yaw.absolute_angle_set = gimbal.yaw.absolute_angle_get;
    gimbal.yaw.DM_MIT_speed = 0.0f;
    gimbal.yaw.DM_MIT_torque = 0.0f;
    yaw_follow_rate_ff = 0.0f;
    yaw_down_input_slew = 0.0f;
    Gimbal_Yaw_Reset();
}

static void Gimbal_Crazy_Cycle_Exit_Reset(void)
{
    crazy_cycle_exit_pending = 0U;
    crazy_cycle_exit_target_pose = GIMBAL_UP;
    crazy_cycle_exit_ready_tick = 0U;
}

static fp32 Gimbal_Yaw_Get_Center_Target(void)
{
    fp32 target = gimbal.yaw.absolute_angle_get + gimbal.yaw.relative_angle_get;

    if (target >= 180.0f) {
        target -= 360.0f;
    } else if (target <= -180.0f) {
        target += 360.0f;
    }

    return target;
}

static Gimbal_Mode_e Pitch_Switch_Infer_Pose_From_Feedback(void)
{
    fp32 down_position = gimbal.down_pitch.motor_measure.position;
    fp32 up_err = 0.0f;
    fp32 middle_err = 0.0f;
    fp32 down_err = 0.0f;

    if (down_position <= 100.0f || down_position >= 8100.0f) {
        down_position = gimbal.down_pitch.smooth_position;
    }

    up_err = ABS(down_position - DOWN_PITCH_OFFSET_POSITION);
    middle_err = ABS(down_position - DOWN_PITCH_MIDDLE_POSITION);
    down_err = ABS(down_position - DOWN_PITCH_DOWN_POSITION);

    if (down_err <= up_err && down_err <= middle_err) {
        return GIMBAL_DOWN;
    }

    if (middle_err <= up_err && middle_err <= down_err) {
        return GIMBAL_CRAZY_CYCLE;
    }

    return GIMBAL_UP;
}

static void Pitch_Switch_Reset(void)
{
    pitch_switch_state = PITCH_SWITCH_IDLE;
    pitch_switch_up_start_position = 0.0f;
    pitch_switch_down_start_position = 0.0f;
    pitch_switch_up_target_position = 0.0f;
    pitch_switch_down_target_position = 0.0f;
    pitch_switch_angle_set = PITCH_SWITCH_LEVEL_ANGLE_SET;
    pitch_switch_abs_angle_tick = 0U;
    pitch_switch_start_mode = GIMBAL_UP;
    pitch_switch_target_mode = GIMBAL_UP;
}

static void Pitch_Switch_Start(Gimbal_Mode_e last_mode, Gimbal_Mode_e mode)
{
    fp32 up_start_position = gimbal.up_pitch.DM_MIT_position;

    Pitch_Switch_Reset();
    if (last_mode != mode) {
        if (up_start_position <= 100.0f || up_start_position >= 8100.0f) {
            up_start_position = gimbal.up_pitch.motor_measure.position;
        }
        if (up_start_position <= 100.0f || up_start_position >= 8100.0f) {
            up_start_position = gimbal.up_pitch.smooth_position +
                                gimbal.up_pitch.absolute_angle_set *
                                gimbal.up_pitch.motor_measure.offset_angle_to_position_ratio;
        }

        pitch_switch_state = PITCH_SWITCH_JOINT_PLAN;
        pitch_switch_start_mode = last_mode;
        pitch_switch_target_mode = mode;
        pitch_switch_up_start_position = up_start_position;
        pitch_switch_down_start_position = gimbal.down_pitch.smooth_position;
        pitch_switch_up_target_position = up_start_position;
        pitch_switch_down_target_position = gimbal.down_pitch.smooth_position;
        gimbal.up_pitch.smooth_position = up_start_position;
        pitch_switch_abs_angle_set = gimbal.up_pitch.absolute_angle_set;
        pitch_switch_abs_angle_tick = HAL_GetTick();
    }
}

static fp32 Pitch_Switch_Update_Abs_Angle_Set(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t dt_ms = now - pitch_switch_abs_angle_tick;
    fp32 step = 0.0f;

    pitch_switch_abs_angle_tick = now;
    if (dt_ms == 0U) {
        dt_ms = GIMBAL_PERIOD;
    } else if (dt_ms > PITCH_SWITCH_ABS_ZERO_DT_MAX_MS) {
        dt_ms = PITCH_SWITCH_ABS_ZERO_DT_MAX_MS;
    }

    step = PITCH_SWITCH_ABS_ZERO_SPEED * (fp32)dt_ms * 0.001f;
    if (pitch_switch_abs_angle_set > step) {
        pitch_switch_abs_angle_set -= step;
    } else if (pitch_switch_abs_angle_set < -step) {
        pitch_switch_abs_angle_set += step;
    } else {
        pitch_switch_abs_angle_set = 0.0f;
    }

    return pitch_switch_abs_angle_set;
}

static void Pitch_Switch_Update_Targets(fp32 up_target, fp32 down_target)
{
    if (pitch_switch_state == PITCH_SWITCH_JOINT_PLAN) {
        fp32 down_range = down_target - pitch_switch_down_start_position;
        fp32 progress = 1.0f;

        pitch_switch_up_target_position = up_target;
        pitch_switch_down_target_position = down_target;

        if (ABS(down_range) > 1.0f) {
            progress = (gimbal.down_pitch.smooth_position - pitch_switch_down_start_position) / down_range;
            VAL_LIMIT(progress, 0.0f, 1.0f)
        }

        fp32 up_plan_target = pitch_switch_up_start_position +
                              (up_target - pitch_switch_up_start_position) * progress;
        pitch_switch_angle_set = PITCH_SWITCH_LEVEL_ANGLE_SET;
        if (progress > PITCH_SWITCH_LEVEL_RELEASE_PROGRESS) {
            fp32 release_progress = (progress - PITCH_SWITCH_LEVEL_RELEASE_PROGRESS) /
                                    (1.0f - PITCH_SWITCH_LEVEL_RELEASE_PROGRESS);
            VAL_LIMIT(release_progress, 0.0f, 1.0f)
            pitch_switch_angle_set = PITCH_SWITCH_LEVEL_ANGLE_SET +
                                     (gimbal.up_pitch.absolute_angle_set -
                                      PITCH_SWITCH_LEVEL_ANGLE_SET) * release_progress;
        }

        Pitch_Smooth_Set_Target(&gimbal.up_pitch, up_plan_target);
        Pitch_Smooth_Set_Target(&gimbal.down_pitch, down_target);

        gimbal.down_pitch.smooth_fast_speed = PITCH_SWITCH_LEVEL_DOWN_FAST_SPEED;
        gimbal.down_pitch.smooth_min_step = PITCH_SWITCH_LEVEL_DOWN_MIN_STEP;

        if (ABS(gimbal.up_pitch.smooth_position - up_target) <= PITCH_SWITCH_UP_RELEASE_ERR &&
            ABS(gimbal.down_pitch.smooth_position - down_target) <= PITCH_SWITCH_DOWN_RELEASE_ERR &&
            ABS(pitch_switch_abs_angle_set) <= PITCH_SWITCH_ABS_ZERO_RELEASE_ERR) {
            Pitch_Switch_Reset();
        }
        return;
    }

    Pitch_Smooth_Set_Target(&gimbal.up_pitch, up_target);
    Pitch_Smooth_Set_Target(&gimbal.down_pitch, down_target);
}

static void Gimbal_Up_Handle(void) {
    if (pitch_recover_from_relax) {
        Pitch_Smooth_Start_From_Feedback(&gimbal.up_pitch, UP_PITCH_OFFSET_POSITION);
        Pitch_Smooth_Start_From_Feedback(&gimbal.down_pitch, DOWN_PITCH_OFFSET_POSITION);
        pitch_recover_from_relax = 0;
    }

    Pitch_Switch_Update_Targets(UP_PITCH_OFFSET_POSITION, DOWN_PITCH_OFFSET_POSITION);
    gimbal.up_pitch.motor_measure.offset_angle_to_position_ratio = ANGLE_TO_POSITION_RATIO_UP;
    gimbal.up_pitch.motor_measure.offset_position_min =  UP_PITCH_UP_POSITION_MIN;
    gimbal.up_pitch.motor_measure.offset_position_max =  UP_PITCH_UP_POSITION_MAX;
}

static void Gimbal_Crazy_Cycle_Handle(void) {
    if (pitch_recover_from_relax) {
        Pitch_Smooth_Start_From_Feedback(&gimbal.up_pitch, UP_PITCH_OFFSET_POSITION);
        Pitch_Smooth_Start_From_Feedback(&gimbal.down_pitch, DOWN_PITCH_OFFSET_POSITION);
        pitch_recover_from_relax = 0;
    }

    Pitch_Switch_Update_Targets(UP_PITCH_OFFSET_POSITION, DOWN_PITCH_OFFSET_POSITION);
    gimbal.up_pitch.motor_measure.offset_angle_to_position_ratio = ANGLE_TO_POSITION_RATIO_UP;
    gimbal.up_pitch.motor_measure.offset_position_min =  UP_PITCH_UP_POSITION_MIN;
    gimbal.up_pitch.motor_measure.offset_position_max =  UP_PITCH_UP_POSITION_MAX;
}

static void Gimbal_Down_Handle(void) {
    if (pitch_recover_from_relax) {
        Pitch_Smooth_Start_From_Feedback(&gimbal.up_pitch, UP_PITCH_DOWN_POSITION);
        Pitch_Smooth_Start_From_Feedback(&gimbal.down_pitch, DOWN_PITCH_DOWN_POSITION);
        pitch_recover_from_relax = 0;
    }

    Pitch_Switch_Update_Targets(UP_PITCH_DOWN_POSITION, DOWN_PITCH_DOWN_POSITION);
    gimbal.up_pitch.motor_measure.offset_angle_to_position_ratio = ANGLE_TO_POSITION_RATIO_DOWN;
    gimbal.up_pitch.motor_measure.offset_position_min =  UP_PITCH_DOWN_POSITION_MIN;
    gimbal.up_pitch.motor_measure.offset_position_max =  UP_PITCH_DOWN_POSITION_MAX;
}

/**
  * @brief          使能模式, 得到遥控器对云台电机的控制
  * @param[in]      none
  * 通过遥控器左摇杆或鼠标控制 pitch 轴和 yaw 轴，对角度值进行限幅
  * @retval         返回空
  */
void Gimbal_Active_Handle(void) {
    //鼠标输入滤波
    //对输入滤波器进行校准，将鼠标的x坐标作为输入
    if(control_flag == ALL_ONLINE) {
        /* 图传优先 */
        first_order_filter_cali(&gimbal.mouse_in_x,Referee.keyboard.mouse_x);
        first_order_filter_cali(&gimbal.mouse_in_y,Referee.keyboard.mouse_y);
        /* 遥控器优先 */
        // first_order_filter_cali(&gimbal.mouse_in_x,rc_ctrl.mouse.x);
        // first_order_filter_cali(&gimbal.mouse_in_y,rc_ctrl.mouse.y);
    }
    if(control_flag == RC_ONLINE) {
        first_order_filter_cali(&gimbal.mouse_in_x,rc_ctrl.mouse.x);
        first_order_filter_cali(&gimbal.mouse_in_y,rc_ctrl.mouse.y);
    }
    if(control_flag == VT_ONLINE) {
        first_order_filter_cali(&gimbal.mouse_in_x,Referee.keyboard.mouse_x);
        first_order_filter_cali(&gimbal.mouse_in_y,Referee.keyboard.mouse_y);
    }

    // //在pit期望值上,按遥控器或者鼠标进行增减
    // gimbal.pitch.relative_up_down_set +=
    //         (float)rc_ctrl.rc.ch[PITCH_CHANNEL] * RC_TO_PITCH
    //         + (float)gimbal.mouse_in_y.out * MOUSE_Y_RADIO;  // rc_ctrl.mouse.y
    //
    // //对pit期望值进行动态限幅（通过陀螺仪和编码器得到动态的限位）
    // gimbal.pitch.relative_up_down_set = fp32_constrain(gimbal.pitch.relative_up_down_set,
    //                                                    MIN_ABS_ANGLE,
    //                                                    MAX_ABS_ANGLE);

    //在pitch期望值上,按遥控器或者鼠标进行增减
    uint8_t pitch_switching = (pitch_switch_state == PITCH_SWITCH_JOINT_PLAN) ||
                              !gimbal.up_pitch.smooth_arrived ||
                              !gimbal.down_pitch.smooth_arrived;

    if (Gimbal_Is_Down_Like_Pose(gimbal_pose_mode)) {
        gimbal.up_pitch.absolute_angle_set = gimbal_up_disable_enter_pending ?
                                             GIMBAL_UP_DISABLE_ENTER_HOLD_ANGLE :
                                             0.0f;
        gimbal.up_pitch_current_first_order_set.input = 0.0f;
        gimbal.up_pitch_current_first_order_set.out = 0.0f;
        pitch_scale = 0.0f;
    } else if (pitch_switching) {
        gimbal.up_pitch.absolute_angle_set = Pitch_Switch_Update_Abs_Angle_Set();
        gimbal.up_pitch_current_first_order_set.input = 0.0f;
        gimbal.up_pitch_current_first_order_set.out = 0.0f;
        pitch_scale = 0.0f;
    } else {
        fp32 pitch_input =
                (float)rc_ctrl.rc.ch[PITCH_CHANNEL] * RC_TO_PITCH
                + (float)gimbal.mouse_in_y.out * MOUSE_Y_RADIO;
        first_order_filter_cali(&gimbal.up_pitch_current_first_order_set, pitch_input);
        gimbal.up_pitch.absolute_angle_set += gimbal.up_pitch_current_first_order_set.out;

        //pitch移动速度系数
        pitch_scale = gimbal.up_pitch_current_first_order_set.out;
    }

    //在yaw期望值上,按遥控器或者鼠标进行增减
    fp32 yaw_rc_input = (ABS(rc_ctrl.rc.ch[YAW_CHANNEL]) > GIMBAL_YAW_RC_DEAD_BAND) ?
                        (fp32)rc_ctrl.rc.ch[YAW_CHANNEL] :
                        0.0f;
    fp32 yaw_mouse_input = (ABS(gimbal.mouse_in_x.out) > GIMBAL_YAW_MOUSE_DEAD_BAND) ?
                           gimbal.mouse_in_x.out :
                           0.0f;

    fp32 yaw_input_delta = yaw_rc_input * RC_TO_YAW
                         + yaw_mouse_input * MOUSE_X_RADIO;    // rc_ctrl.mouse.x
    gimbal_prone_vw_cmd = yaw_rc_input * GIMBAL_PRONE_RC_TO_VW +
                          yaw_mouse_input * GIMBAL_PRONE_MOUSE_TO_VW;
    VAL_LIMIT(gimbal_prone_vw_cmd,
              -GIMBAL_PRONE_VW_CMD_MAX,
               GIMBAL_PRONE_VW_CMD_MAX);
    if (Gimbal_Is_Down_Like_Pose(gimbal_pose_mode)) {
        fp32 yaw_follow_err = fabsf(gimbal.yaw.relative_angle_get);
        fp32 yaw_follow_scale = 1.0f;
        fp32 yaw_follow_err_dir = gimbal.yaw.relative_angle_get * yaw_input_delta;
        uint8_t yaw_input_pushes_limit = (yaw_follow_err_dir < 0.0f);

        if (yaw_follow_err >= GIMBAL_DOWN_YAW_INPUT_HARD_ERR &&
            yaw_input_pushes_limit) {
            yaw_input_delta = 0.0f;
            yaw_down_input_slew = 0.0f;
        }
        VAL_LIMIT(yaw_input_delta,
                  -GIMBAL_DOWN_YAW_INPUT_MAX_DELTA,
                   GIMBAL_DOWN_YAW_INPUT_MAX_DELTA);
        if (yaw_input_pushes_limit &&
            yaw_follow_err >= GIMBAL_DOWN_YAW_FOLLOW_STOP_ERR) {
            yaw_follow_scale = 0.0f;
        } else if (yaw_input_pushes_limit &&
                   yaw_follow_err > GIMBAL_DOWN_YAW_FOLLOW_SLOW_ERR) {
            yaw_follow_scale = 1.0f -
                               (yaw_follow_err - GIMBAL_DOWN_YAW_FOLLOW_SLOW_ERR) /
                               (GIMBAL_DOWN_YAW_FOLLOW_STOP_ERR - GIMBAL_DOWN_YAW_FOLLOW_SLOW_ERR);
            if (yaw_follow_scale < GIMBAL_DOWN_YAW_FOLLOW_MIN_SCALE) {
                yaw_follow_scale = GIMBAL_DOWN_YAW_FOLLOW_MIN_SCALE;
            }
        }
        yaw_input_delta *= yaw_follow_scale;
        fp32 yaw_input_step = yaw_input_delta - yaw_down_input_slew;
        fp32 yaw_input_step_limit = (yaw_input_delta * yaw_down_input_slew < 0.0f ||
                                    fabsf(yaw_input_delta) < fabsf(yaw_down_input_slew)) ?
                                    GIMBAL_DOWN_YAW_INPUT_DECAY_STEP :
                                    GIMBAL_DOWN_YAW_INPUT_MAX_STEP;
        if (yaw_follow_err >= GIMBAL_DOWN_YAW_INPUT_HARD_ERR) {
            yaw_input_step_limit = GIMBAL_DOWN_YAW_INPUT_HARD_DECAY;
        }
        VAL_LIMIT(yaw_input_step, -yaw_input_step_limit, yaw_input_step_limit);
        yaw_down_input_slew += yaw_input_step;
        yaw_input_delta = yaw_down_input_slew;
    } else {
        yaw_down_input_slew = 0.0f;
    }
    fp32 yaw_rate_ff_target = -yaw_input_delta / (GIMBAL_PERIOD * 0.001f);
    if (Gimbal_Yaw_Disabled_In_Current_Mode()) {
        yaw_follow_rate_ff = 0.0f;
        yaw_down_input_slew = 0.0f;
        gimbal.yaw.absolute_angle_set = gimbal.yaw.absolute_angle_get;
    } else if (Gimbal_Is_Down_Like_Pose(gimbal_pose_mode)) {
        VAL_LIMIT(yaw_rate_ff_target,
                  -GIMBAL_DOWN_YAW_FOLLOW_FF_MAX,
                   GIMBAL_DOWN_YAW_FOLLOW_FF_MAX);
        fp32 yaw_rate_ff_delta = yaw_rate_ff_target - yaw_follow_rate_ff;
        VAL_LIMIT(yaw_rate_ff_delta,
                  -GIMBAL_DOWN_YAW_FOLLOW_FF_STEP,
                   GIMBAL_DOWN_YAW_FOLLOW_FF_STEP);
        yaw_rate_ff_target = yaw_follow_rate_ff + yaw_rate_ff_delta;
        yaw_follow_rate_ff += (yaw_rate_ff_target - yaw_follow_rate_ff) *
                              GIMBAL_DOWN_YAW_FOLLOW_FF_LPF;
    } else {
        yaw_follow_rate_ff = 0.0f;
    }
    if (crazy_cycle_exit_pending && Gimbal_Pose_Control_Enabled()) {
        gimbal.yaw.absolute_angle_set = Gimbal_Yaw_Get_Center_Target();
    } else if (!Gimbal_Yaw_Disabled_In_Current_Mode()) {
        gimbal.yaw.absolute_angle_set -= yaw_input_delta;
    }

    //云台绕圈时进行绝对角循环设置, 当yaw期望值超过180度时, 将其调整到[-180,180]的范围
    if (gimbal.yaw.absolute_angle_set >= 180) {
        gimbal.yaw.absolute_angle_set -= 360;
    }
    else if (gimbal.yaw.absolute_angle_set <= -180) {
        gimbal.yaw.absolute_angle_set += 360;
    }

    if (gimbal_pose_mode == GIMBAL_UP) {
        gimbal.up_pitch.absolute_angle_set = fp32_constrain(gimbal.up_pitch.absolute_angle_set,
                                                           MIN_UP_ANGLE,
                                                           MAX_UP_ANGLE);
    } else if (Gimbal_Is_Crazy_Cycle_Pose(gimbal_pose_mode)) {
        gimbal.up_pitch.absolute_angle_set = fp32_constrain(gimbal.up_pitch.absolute_angle_set,
                                                           MIN_SPIN_ANGLE,
                                                           MAX_SPIN_ANGLE);
    } else if (Gimbal_Is_Down_Like_Pose(gimbal_pose_mode)) {
        gimbal.up_pitch.absolute_angle_set = gimbal_up_disable_enter_pending ?
                                             GIMBAL_UP_DISABLE_ENTER_HOLD_ANGLE :
                                             0.0f;
    }
    //小陀螺限制pitch角度，避免测速撞到电池
    if (gimbal.chassis_mode == CHASSIS_SPIN_R || gimbal.chassis_mode == CHASSIS_SPIN_L) {
        gimbal.up_pitch.absolute_angle_set = fp32_constrain(gimbal.up_pitch.absolute_angle_set,
                                                           MIN_SPIN_ANGLE,
                                                           MAX_SPIN_ANGLE);
    }

    //一键掉头判断
    gimbal_turn_back_judge();
}

// void Gimbal_DM_Ctrl_Loop_Cal(void)
// {
//     /* yaw闭环控制 */
//     // 计算目标角度变化率（前馈项）
//     fp32 yaw_angle_set_delta = gimbal.yaw.absolute_angle_set - gimbal.yaw.last_absolute_angle_set;
//
//     // 处理过零点情况
//     if (yaw_angle_set_delta > 180.0f) {
//         yaw_angle_set_delta -= 360.0f;
//     } else if (yaw_angle_set_delta < -180.0f) {
//         yaw_angle_set_delta += 360.0f;
//     }
//
//     // 前馈项：目标变化率 * 前馈系数
//     fp32 yaw_feedforward = yaw_angle_set_delta * GIMBAL_YAW_FEEDFORWARD_KF;
//
//     // 更新上一次目标角度
//     gimbal.yaw.last_absolute_angle_set = gimbal.yaw.absolute_angle_set;
//
//     // PID 反馈项
//     fp32 yaw_pid_out = pid_loop_calc(&(gimbal.yaw.angle_p),
//                                       gimbal.yaw.absolute_angle_get,
//                                       gimbal.yaw.absolute_angle_set,
//                                       180,
//                                       -180);
//
//     // 前馈 + PID 输出
//     gimbal.yaw.DM_MIT_speed = yaw_pid_out + yaw_feedforward;
//
//     // 输出限幅
//     if (gimbal.yaw.DM_MIT_speed > GIMBAL_YAW_ANGLE_MAX_OUT) {
//         gimbal.yaw.DM_MIT_speed = GIMBAL_YAW_ANGLE_MAX_OUT;
//     } else if (gimbal.yaw.DM_MIT_speed < -GIMBAL_YAW_ANGLE_MAX_OUT) {
//         gimbal.yaw.DM_MIT_speed = -GIMBAL_YAW_ANGLE_MAX_OUT;
//     }
//
// }
static fp32 Gimbal_Yaw_Loop_Error(fp32 set, fp32 get)
{
    fp32 err = set - get;

    if (err > 180.0f) {
        err -= 360.0f;
    } else if (err < -180.0f) {
        err += 360.0f;
    }

    return err;
}

static void Gimbal_Yaw_Reset(void)
{
    yaw_last_ctrl_mode = gimbal.mode;
    yaw_vel_ref = 0.0f;
    yaw_angle_err_last = 0.0f;
    yaw_last_angle_set = gimbal.yaw.absolute_angle_set;
    yaw_set_vel_filter = 0.0f;
    yaw_gyro_filter = gyro_yaw;
    yaw_plan_angle = gimbal.yaw.absolute_angle_set;
    yaw_plan_vel = 0.0f;
    yaw_plan_acc = 0.0f;
    yaw_torque_last = 0.0f;
    yaw_adapt_tick = 0;
    yaw_ctrl_debug.err = 0.0f;
    yaw_ctrl_debug.target_err = 0.0f;
    yaw_ctrl_debug.plan_angle = yaw_plan_angle;
    yaw_ctrl_debug.set_vel = 0.0f;
    yaw_ctrl_debug.vel_ff = 0.0f;
    yaw_ctrl_debug.vel_ref = 0.0f;
    yaw_ctrl_debug.vel_err = 0.0f;
    yaw_ctrl_debug.brake_torque = 0.0f;
    yaw_ctrl_debug.friction_torque = 0.0f;
    yaw_ctrl_debug.out_torque = 0.0f;
    yaw_ctrl_debug.load_scale = Gimbal_Yaw_Get_Load_Scale() * Gimbal_Yaw_Ammo_Drive_Scale();
    yaw_ctrl_debug.pose_scale = Gimbal_Yaw_Pose_Scale();
    yaw_ctrl_debug.ammo_ratio = Gimbal_Yaw_Ammo_Ratio();
}

static fp32 Gimbal_Yaw_Pose_Scale(void)
{
    if (gimbal.mode == GIMBAL_RELAX) {
        return 0.0f;
    }

    if (Gimbal_Is_Down_Like_Pose(gimbal_pose_mode)) {
        return GIMBAL_YAW_IMP_DOWN_BIAS;
    }

    if (Gimbal_Is_Up_Like_Pose(gimbal_pose_mode)) {
        return GIMBAL_YAW_IMP_UP_BIAS;
    }

    return 0.0f;
}

static fp32 Gimbal_Yaw_Get_Load_Scale(void)
{
    if (Gimbal_Is_Down_Like_Pose(gimbal_pose_mode)) {
        return yaw_down_load_scale;
    }

    return yaw_up_load_scale;
}

static void Gimbal_Yaw_Set_Load_Scale(fp32 scale)
{
    if (Gimbal_Is_Down_Like_Pose(gimbal_pose_mode)) {
        yaw_down_load_scale = scale;
    } else {
        yaw_up_load_scale = scale;
    }
}

static void Gimbal_Yaw_Ammo_Update_From_Referee(uint8_t game_progress, uint8_t shoot_seq)
{
    uint8_t is_match_progress = (game_progress == GIMBAL_YAW_AMMO_PROGRESS_COUNTDOWN ||
                                 game_progress == GIMBAL_YAW_AMMO_PROGRESS_RUNNING);

#if GIMBAL_YAW_AMMO_TEST_ENABLE && GIMBAL_YAW_AMMO_TEST_OUTSIDE_MATCH_ONLY
    if (is_match_progress == 0U) {
        (void)shoot_seq;
        yaw_ammo_load_ratio = GIMBAL_YAW_AMMO_TEST_RATIO;
        VAL_LIMIT(yaw_ammo_load_ratio, 0.0f, 1.0f);
        yaw_referee_last_game_progress = GIMBAL_YAW_AMMO_TEST_GAME_PROGRESS;
        yaw_referee_last_shoot_seq = GIMBAL_YAW_AMMO_TEST_SHOOT_SEQ;
        yaw_referee_inited = 1U;
        return;
    }
#elif GIMBAL_YAW_AMMO_TEST_ENABLE
    (void)is_match_progress;
    (void)shoot_seq;
    yaw_ammo_load_ratio = GIMBAL_YAW_AMMO_TEST_RATIO;
    VAL_LIMIT(yaw_ammo_load_ratio, 0.0f, 1.0f);
    yaw_referee_last_game_progress = GIMBAL_YAW_AMMO_TEST_GAME_PROGRESS;
    yaw_referee_last_shoot_seq = GIMBAL_YAW_AMMO_TEST_SHOOT_SEQ;
    yaw_referee_inited = 1U;
    return;
#endif

#if GIMBAL_YAW_AMMO_ENABLE
    uint8_t was_match_progress = (yaw_referee_last_game_progress == GIMBAL_YAW_AMMO_PROGRESS_COUNTDOWN ||
                                  yaw_referee_last_game_progress == GIMBAL_YAW_AMMO_PROGRESS_RUNNING);

    if (yaw_referee_inited == 0U) {
        yaw_referee_last_game_progress = game_progress;
        yaw_referee_last_shoot_seq = shoot_seq;
        yaw_referee_inited = 1U;
        return;
    }

    if (is_match_progress != 0U && was_match_progress == 0U) {
        yaw_ammo_load_ratio = 1.0f;
    }

    if (is_match_progress != 0U) {
        uint8_t shot_delta = (uint8_t)(shoot_seq - yaw_referee_last_shoot_seq);
        if (shot_delta > 0U && shot_delta < 128U) {
            yaw_ammo_load_ratio -= (fp32)shot_delta * GIMBAL_YAW_AMMO_SHOT_DEC;
            VAL_LIMIT(yaw_ammo_load_ratio, 0.0f, 1.0f);
        }
    }

    yaw_referee_last_game_progress = game_progress;
    yaw_referee_last_shoot_seq = shoot_seq;
#else
    (void)game_progress;
    (void)shoot_seq;
    yaw_ammo_load_ratio = 1.0f;
#endif
}

static fp32 Gimbal_Yaw_Ammo_Ratio(void)
{
    fp32 ratio = yaw_ammo_load_ratio;
#if GIMBAL_YAW_AMMO_TEST_ENABLE && !GIMBAL_YAW_AMMO_TEST_OUTSIDE_MATCH_ONLY
    ratio = GIMBAL_YAW_AMMO_TEST_RATIO;
#endif
    VAL_LIMIT(ratio, 0.0f, 1.0f);
    return ratio;
}

static fp32 Gimbal_Yaw_Ammo_Drive_Scale(void)
{
    fp32 ratio = Gimbal_Yaw_Ammo_Ratio();
    return GIMBAL_YAW_AMMO_EMPTY_DRIVE_SCALE +
           (GIMBAL_YAW_AMMO_FULL_DRIVE_SCALE - GIMBAL_YAW_AMMO_EMPTY_DRIVE_SCALE) * ratio;
}

static fp32 Gimbal_Yaw_Ammo_Damp_Scale(void)
{
    fp32 ratio = Gimbal_Yaw_Ammo_Ratio();
    return GIMBAL_YAW_AMMO_EMPTY_DAMP_SCALE +
           (GIMBAL_YAW_AMMO_FULL_DAMP_SCALE - GIMBAL_YAW_AMMO_EMPTY_DAMP_SCALE) * ratio;
}

static fp32 Gimbal_Yaw_Ammo_Max_Torque(void)
{
    fp32 ratio = Gimbal_Yaw_Ammo_Ratio();
    return GIMBAL_YAW_IMP_MAX_TORQUE +
           (GIMBAL_YAW_IMP_FULL_MAX_TORQUE - GIMBAL_YAW_IMP_MAX_TORQUE) * ratio;
}

static yaw_cmd_t Gimbal_Yaw_Build_Cmd(void)
{
    yaw_cmd_t cmd;
    uint8_t follow_down = Gimbal_Is_Down_Like_Pose(gimbal_pose_mode);
    fp32 plan_kp = follow_down ? GIMBAL_DOWN_YAW_PLAN_KP : GIMBAL_YAW_PLAN_KP;
    fp32 plan_set_ff = follow_down ? GIMBAL_DOWN_YAW_PLAN_SET_FF : GIMBAL_YAW_PLAN_SET_FF;
    fp32 plan_small_ff = follow_down ? GIMBAL_DOWN_YAW_PLAN_SMALL_FF : GIMBAL_YAW_PLAN_SMALL_FF;
    fp32 plan_reverse_ff = follow_down ? GIMBAL_DOWN_YAW_PLAN_REVERSE_FF : GIMBAL_YAW_PLAN_REVERSE_FF;
    fp32 plan_vel_lpf = follow_down ? GIMBAL_DOWN_YAW_PLAN_VEL_LPF : GIMBAL_YAW_PLAN_VEL_LPF;
    fp32 plan_max_vel = follow_down ? GIMBAL_DOWN_YAW_PLAN_MAX_VEL : GIMBAL_YAW_PLAN_MAX_VEL;
    fp32 plan_max_accel = follow_down ? GIMBAL_DOWN_YAW_PLAN_MAX_ACCEL : GIMBAL_YAW_PLAN_MAX_ACCEL;
    fp32 yaw_track_err = Gimbal_Yaw_Loop_Error(gimbal.yaw.absolute_angle_set,
                                               gimbal.yaw.absolute_angle_get);
    fp32 yaw_set_delta = Gimbal_Yaw_Loop_Error(gimbal.yaw.absolute_angle_set,
                                               yaw_last_angle_set);
    fp32 yaw_set_vel_ff = yaw_set_delta / GIMBAL_YAW_SERVO_DT;
    VAL_LIMIT(yaw_set_vel_ff,
              -plan_max_vel,
              plan_max_vel);

    fp32 yaw_set_ff_scale = plan_set_ff;
    fp32 yaw_plan_acc_limit = plan_max_accel;

    if (ABS(yaw_track_err) < GIMBAL_YAW_PLAN_SMALL_ERR) {
        yaw_set_ff_scale = plan_small_ff;
        yaw_plan_acc_limit *= GIMBAL_YAW_PLAN_SMALL_ACCEL_SCALE;
    }

    if (yaw_set_vel_ff * yaw_plan_vel < 0.0f) {
        yaw_set_ff_scale = plan_reverse_ff;
        yaw_plan_acc_limit *= GIMBAL_YAW_PLAN_REVERSE_ACCEL_SCALE;
    }

    fp32 yaw_plan_vel_target = yaw_track_err * plan_kp +
                               yaw_set_vel_ff * yaw_set_ff_scale;
    VAL_LIMIT(yaw_plan_vel_target,
              -plan_max_vel,
              plan_max_vel);

    fp32 yaw_plan_vel_delta = yaw_plan_vel_target - yaw_plan_vel;
    VAL_LIMIT(yaw_plan_vel_delta,
              -yaw_plan_acc_limit,
              yaw_plan_acc_limit);
    yaw_plan_acc = yaw_plan_vel_delta;
    yaw_plan_vel += yaw_plan_vel_delta;

    yaw_plan_angle = gimbal.yaw.absolute_angle_set;

    yaw_set_vel_filter += (yaw_plan_vel - yaw_set_vel_filter) * plan_vel_lpf;

    cmd.angle_set = gimbal.yaw.absolute_angle_set;
    cmd.vel_ff = yaw_set_vel_filter * Gimbal_Yaw_Ammo_Drive_Scale();
    cmd.acc_ff = yaw_plan_acc * Gimbal_Yaw_Ammo_Drive_Scale();
    yaw_last_angle_set = gimbal.yaw.absolute_angle_set;

    return cmd;
}

static void Gimbal_Yaw_Load_Adapt_Update(fp32 yaw_err, fp32 vel_err, fp32 plan_vel, fp32 gyro_feedback)
{
    fp32 load_scale = Gimbal_Yaw_Get_Load_Scale();

    yaw_adapt_tick++;
    if (yaw_adapt_tick < GIMBAL_YAW_LOAD_ADAPT_PERIOD) {
        yaw_angle_err_last = yaw_err;
        return;
    }
    yaw_adapt_tick = 0;

    if (ABS(plan_vel) > GIMBAL_YAW_LOAD_ADAPT_SPEED_ERR &&
        ABS(vel_err) > GIMBAL_YAW_LOAD_ADAPT_SPEED_ERR &&
        plan_vel * vel_err > 0.0f) {
        load_scale += GIMBAL_YAW_LOAD_ADAPT_INC;
    }

    if ((yaw_err * yaw_angle_err_last < 0.0f && ABS(gyro_feedback) > GIMBAL_YAW_LOAD_ADAPT_SPEED_ERR) ||
        (ABS(gyro_feedback) > GIMBAL_YAW_LOAD_ADAPT_OSC_GYRO &&
         plan_vel * gyro_feedback < 0.0f)) {
        load_scale *= GIMBAL_YAW_LOAD_ADAPT_DEC;
    }

    VAL_LIMIT(load_scale,
              GIMBAL_YAW_LOAD_ADAPT_MIN,
              GIMBAL_YAW_LOAD_ADAPT_MAX);
    Gimbal_Yaw_Set_Load_Scale(load_scale);
    yaw_angle_err_last = yaw_err;
}

int cnt_send_yaw;

static void Gimbal_Yaw_Servo(const yaw_cmd_t *cmd)
{
    if (Gimbal_Yaw_Disabled_In_Current_Mode()) {
        Gimbal_Yaw_Disable_Reset();
        return;
    }

    if (yaw_last_ctrl_mode != gimbal.mode) {
        gimbal.yaw.absolute_angle_set = gimbal.yaw.absolute_angle_get;
        Gimbal_Yaw_Reset();
        gimbal.yaw.DM_MIT_speed = 0.0f;
        gimbal.yaw.DM_MIT_torque = 0.0f;
        return;
    }

    fp32 yaw_err = Gimbal_Yaw_Loop_Error(cmd->angle_set,
                                         gimbal.yaw.absolute_angle_get);
    uint8_t follow_down = Gimbal_Is_Down_Like_Pose(gimbal_pose_mode);
    fp32 imp_kp = follow_down ? GIMBAL_DOWN_YAW_IMP_KP : GIMBAL_YAW_IMP_KP;
    fp32 imp_kd = follow_down ? GIMBAL_DOWN_YAW_IMP_KD : GIMBAL_YAW_IMP_KD;
    fp32 imp_kb = follow_down ? GIMBAL_DOWN_YAW_IMP_KB : GIMBAL_YAW_IMP_KB;
    fp32 imp_max_brake_torque = follow_down ? GIMBAL_DOWN_YAW_IMP_MAX_BRAKE_TORQUE :
                                GIMBAL_YAW_IMP_MAX_BRAKE_TORQUE;
    fp32 imp_max_torque = follow_down ? GIMBAL_DOWN_YAW_IMP_MAX_TORQUE :
                          Gimbal_Yaw_Ammo_Max_Torque();
    fp32 imp_max_torque_step = follow_down ? GIMBAL_DOWN_YAW_IMP_MAX_TORQUE_STEP :
                               GIMBAL_YAW_IMP_MAX_TORQUE_STEP;
    fp32 imp_max_vel_err = follow_down ? GIMBAL_DOWN_YAW_IMP_MAX_VEL_ERR :
                             GIMBAL_YAW_IMP_MAX_VEL_ERR;
    fp32 imp_gyro_lpf = follow_down ? GIMBAL_DOWN_YAW_IMP_GYRO_LPF : GIMBAL_YAW_IMP_GYRO_LPF;
    fp32 pose_scale = Gimbal_Yaw_Pose_Scale();
    fp32 response_load_scale = Gimbal_Yaw_Get_Load_Scale();
    fp32 load_scale = response_load_scale * Gimbal_Yaw_Ammo_Drive_Scale();
    fp32 damp_scale = Gimbal_Yaw_Ammo_Damp_Scale();
    yaw_gyro_filter += (gyro_yaw - yaw_gyro_filter) * imp_gyro_lpf;

    yaw_vel_ref = follow_down ? 0.0f : cmd->vel_ff;
    fp32 yaw_vel_err = yaw_vel_ref - yaw_gyro_filter;
    VAL_LIMIT(yaw_vel_err,
              -imp_max_vel_err,
              imp_max_vel_err);

    fp32 yaw_acc_ff = cmd->acc_ff;
    VAL_LIMIT(yaw_acc_ff,
              -GIMBAL_YAW_IMP_MAX_ACC_FF,
              GIMBAL_YAW_IMP_MAX_ACC_FF);
    fp32 yaw_friction_torque = 0.0f;

    if (ABS(yaw_err) > GIMBAL_YAW_IMP_FRICTION_ERR) {
        yaw_friction_torque = (yaw_err > 0.0f) ?
                              GIMBAL_YAW_IMP_KF :
                              -GIMBAL_YAW_IMP_KF;
    }

    fp32 yaw_brake_torque = 0.0f;
    if (ABS(yaw_err) < GIMBAL_YAW_IMP_BRAKE_ERR &&
        yaw_err * yaw_gyro_filter > 0.0f) {
        fp32 yaw_brake_scale = (GIMBAL_YAW_IMP_BRAKE_ERR - ABS(yaw_err)) /
                               GIMBAL_YAW_IMP_BRAKE_ERR;
        yaw_brake_torque = -yaw_gyro_filter * imp_kb * yaw_brake_scale;
        VAL_LIMIT(yaw_brake_torque,
                  -imp_max_brake_torque,
                  imp_max_brake_torque);
    }

    if (ABS(yaw_err) < GIMBAL_YAW_IMP_BRAKE_ERR &&
        ABS(yaw_vel_err) > GIMBAL_YAW_IMP_SETTLE_VEL_DEADBAND) {
        fp32 yaw_settle_scale = (GIMBAL_YAW_IMP_BRAKE_ERR - ABS(yaw_err)) /
                                GIMBAL_YAW_IMP_BRAKE_ERR;
        fp32 yaw_settle_vel = yaw_vel_err -
                              ((yaw_vel_err > 0.0f) ?
                               GIMBAL_YAW_IMP_SETTLE_VEL_DEADBAND :
                               -GIMBAL_YAW_IMP_SETTLE_VEL_DEADBAND);
        fp32 yaw_settle_torque = yaw_settle_vel * GIMBAL_YAW_IMP_SETTLE_KD *
                                 damp_scale * yaw_settle_scale;
        VAL_LIMIT(yaw_settle_torque,
                  -GIMBAL_YAW_IMP_MAX_SETTLE_TORQUE,
                  GIMBAL_YAW_IMP_MAX_SETTLE_TORQUE);
        yaw_brake_torque += yaw_settle_torque;
    }

    fp32 yaw_torque = yaw_err * imp_kp * load_scale
                    + yaw_vel_err * imp_kd * damp_scale
                    + yaw_acc_ff * GIMBAL_YAW_IMP_KA * load_scale
                    + yaw_friction_torque
                    + yaw_brake_torque;
    yaw_torque *= pose_scale;

    VAL_LIMIT(yaw_torque,
              -imp_max_torque,
              imp_max_torque);

    fp32 yaw_torque_delta = yaw_torque - yaw_torque_last;
    VAL_LIMIT(yaw_torque_delta,
              -imp_max_torque_step,
              imp_max_torque_step);
    yaw_torque = yaw_torque_last + yaw_torque_delta;
    yaw_torque_last = yaw_torque;

    Gimbal_Yaw_Load_Adapt_Update(yaw_err, yaw_vel_err, cmd->vel_ff, yaw_gyro_filter);

    yaw_ctrl_debug.err = yaw_err;
    yaw_ctrl_debug.target_err = Gimbal_Yaw_Loop_Error(gimbal.yaw.absolute_angle_set,
                                                      gimbal.yaw.absolute_angle_get);
    yaw_ctrl_debug.plan_angle = yaw_plan_angle;
    yaw_ctrl_debug.set_vel = cmd->vel_ff;
    yaw_ctrl_debug.vel_ff = yaw_acc_ff;
    yaw_ctrl_debug.vel_ref = yaw_vel_ref;
    yaw_ctrl_debug.vel_err = yaw_vel_err;
    yaw_ctrl_debug.brake_torque = yaw_brake_torque * pose_scale;
    yaw_ctrl_debug.friction_torque = yaw_friction_torque;
    yaw_ctrl_debug.out_torque = yaw_torque;
    yaw_ctrl_debug.load_scale = load_scale;
    yaw_ctrl_debug.pose_scale = pose_scale;
    yaw_ctrl_debug.ammo_ratio = Gimbal_Yaw_Ammo_Ratio();

    gimbal.yaw.DM_MIT_speed = 0.0f;
    gimbal.yaw.DM_MIT_torque = yaw_torque;
}

/* Pitch pose smooth helpers */
static void Pitch_Smooth_Init(Motor_Gimbal_t *pitch, fp32 target)
{
    pitch->smooth_target = target;
    pitch->smooth_position = target;
    pitch->smooth_step = 0.0f;
    pitch->smooth_fast_speed = PITCH_SMOOTH_FAST_SPEED;
    pitch->smooth_min_step = PITCH_SMOOTH_MIN_STEP;
    pitch->smooth_last_tick = HAL_GetTick();
    pitch->smooth_arrived = 1;
}

static void Pitch_Smooth_Start_From_Feedback(Motor_Gimbal_t *pitch, fp32 target)
{
    fp32 feedback = pitch->motor_measure.position;

    pitch->smooth_target = target;
    if (feedback > 100.0f && feedback < 8100.0f) {
        pitch->smooth_position = feedback;
    } else {
        pitch->smooth_position = pitch->motor_measure.offset_position;
    }
    pitch->smooth_step = 0.0f;
    pitch->smooth_fast_speed = PITCH_SMOOTH_RECOVER_FAST_SPEED;
    pitch->smooth_min_step = PITCH_SMOOTH_RECOVER_MIN_STEP;
    pitch->smooth_last_tick = HAL_GetTick();
    pitch->smooth_arrived = 0;
}

static void Pitch_Smooth_Set_Target(Motor_Gimbal_t *pitch, fp32 target)
{
    pitch->smooth_target = target;
    if (pitch->smooth_arrived) {
        pitch->smooth_fast_speed = PITCH_SMOOTH_FAST_SPEED;
        pitch->smooth_min_step = PITCH_SMOOTH_MIN_STEP;
    }
}

static fp32 Pitch_Smooth_Position_Update(Motor_Gimbal_t *pitch)
{
    fp32 err = pitch->smooth_target - pitch->smooth_position;
    fp32 abs_err = ABS(err);
    fp32 step = 0.0f;
    uint32_t now = HAL_GetTick();
    uint32_t dt_ms = now - pitch->smooth_last_tick;
    fp32 dt_scale = 1.0f;

    pitch->smooth_last_tick = now;
    if (dt_ms == 0U) {
        dt_ms = GIMBAL_PERIOD;
    } else if (dt_ms > PITCH_SMOOTH_DT_MAX_MS) {
        dt_ms = PITCH_SMOOTH_DT_MAX_MS;
    }
    dt_scale = (fp32)dt_ms / (fp32)GIMBAL_PERIOD;

    if (abs_err <= PITCH_SMOOTH_THRESHOLD_3) {
        pitch->smooth_position = pitch->smooth_target;
        pitch->smooth_step = 0.0f;
        pitch->smooth_fast_speed = PITCH_SMOOTH_FAST_SPEED;
        pitch->smooth_min_step = PITCH_SMOOTH_MIN_STEP;
        pitch->smooth_arrived = 1;
        return pitch->smooth_position;
    }

    if (abs_err > PITCH_SMOOTH_THRESHOLD_1) {
        step = pitch->smooth_fast_speed * dt_scale;
    } else if (abs_err > PITCH_SMOOTH_THRESHOLD_2) {
        step = abs_err * PITCH_SMOOTH_RATIO_1 * dt_scale;
        if (step > pitch->smooth_fast_speed * dt_scale) {
            step = pitch->smooth_fast_speed * dt_scale;
        }
    } else {
        fp32 slow_step = abs_err * PITCH_SMOOTH_RATIO_2 * dt_scale;
        fp32 boundary_step = PITCH_SMOOTH_THRESHOLD_2 * PITCH_SMOOTH_RATIO_1 * dt_scale;
        fp32 blend = (abs_err - PITCH_SMOOTH_THRESHOLD_3) /
                     (PITCH_SMOOTH_THRESHOLD_2 - PITCH_SMOOTH_THRESHOLD_3);

        if (boundary_step > pitch->smooth_fast_speed * dt_scale) {
            boundary_step = pitch->smooth_fast_speed * dt_scale;
        }
        VAL_LIMIT(blend, 0.0f, 1.0f);
        step = slow_step + (boundary_step - slow_step) * blend;
    }

    if (step < pitch->smooth_min_step * dt_scale) {
        step = pitch->smooth_min_step * dt_scale;
    }
    if (step > pitch->smooth_step + PITCH_SMOOTH_ACCEL_STEP * dt_scale) {
        step = pitch->smooth_step + PITCH_SMOOTH_ACCEL_STEP * dt_scale;
    } else if (step + PITCH_SMOOTH_ACCEL_STEP * dt_scale < pitch->smooth_step) {
        step = pitch->smooth_step - PITCH_SMOOTH_ACCEL_STEP * dt_scale;
    }
    if (step > abs_err) {
        step = abs_err;
    }
    pitch->smooth_step = step;

    pitch->smooth_position += (err > 0.0f) ? step : -step;
    pitch->smooth_arrived = 0;
    return pitch->smooth_position;
}

fp32 angle_fix = 0;
static void Gimbal_DM_Send_One(fp32 up_pos, fp32 up_kp, fp32 up_kd, fp32 up_torque,
                               fp32 down_pos, fp32 down_kp, fp32 down_kd, fp32 down_torque,
                               fp32 yaw_kd, fp32 yaw_torque)
{
    static uint8_t dm_send_phase = 0U;

    switch (dm_send_phase) {
    case 0U:
        DM_MIT_Ctrl_Motor(&hcan2,
                          CAN_DM_MOTOR_UP_PITCH_ID,
                          up_pos,
                          0,
                          up_kp,
                          up_kd,
                          up_torque);
        break;
    case 1U:
        DM_MIT_Ctrl_Motor(&hcan2,
                          CAN_DM_MOTOR_DOWN_PITCH_ID,
                          down_pos,
                          0,
                          down_kp,
                          down_kd,
                          down_torque);
        break;
    default:
        DM_MIT_Ctrl_Motor(&hcan2,
                          CAN_DM_MOTOR_YAW_ID,
                          0,
                          0,
                          0,
                          yaw_kd,
                          yaw_torque);
        cnt_send_yaw++;
        break;
    }

    dm_send_phase++;
    if (dm_send_phase >= 3U) {
        dm_send_phase = 0U;
    }
}

static void Gimbal_Up_Pitch_Position_Fix_Reset(void)
{
    up_pitch_position_fix.iout = 0.0f;
    up_pitch_position_fix.dout = 0.0f;
    up_pitch_position_fix.out = 0.0f;
    up_pitch_position_fix.pout = 0.0f;
    up_pitch_position_fix.sum_err = 0.0f;
    up_pitch_position_fix.err[NOW] = 0.0f;
    up_pitch_position_fix.err[LAST] = 0.0f;

    gimbal.up_pitch_first_order_set.input = 0.0f;
    gimbal.up_pitch_first_order_set.out = 0.0f;
    angle_fix = 0.0f;
}

static fp32 Gimbal_Up_Pitch_Position_Fix(fp32 target_angle)
{
#if UP_PITCH_ANGLE_FIX_ENABLE
    fp32 pitch_min = MIN_UP_ANGLE;
    fp32 pitch_max = MAX_UP_ANGLE;
    fp32 pitch_fix_pid_out = 0.0f;
    fp32 pitch_angle_err = target_angle - gimbal.up_pitch.absolute_angle_get;
    fp32 pitch_angle_abs_err = ABS(pitch_angle_err);
    if (Gimbal_Is_Crazy_Cycle_Pose(gimbal_pose_mode)) {
        pitch_min = MIN_SPIN_ANGLE;
        pitch_max = MAX_SPIN_ANGLE;
    } else if (Gimbal_Is_Down_Like_Pose(gimbal_pose_mode)) {
        pitch_min = MIN_DOWN_ANGLE;
        pitch_max = MAX_DOWN_ANGLE;
    } else if (gimbal.chassis_mode == CHASSIS_SPIN_R || gimbal.chassis_mode == CHASSIS_SPIN_L) {
        pitch_min = MIN_SPIN_ANGLE;
        pitch_max = MAX_SPIN_ANGLE;
    }

    if ((gimbal.up_pitch.absolute_angle_get >= (pitch_max - PITCH_LIMIT_GUARD)) ||
        (gimbal.up_pitch.absolute_angle_get <= (pitch_min + PITCH_LIMIT_GUARD))) {
        up_pitch_position_fix.iout = 0.0f;
        up_pitch_position_fix.dout = 0.0f;
        up_pitch_position_fix.err[LAST] = 0.0f;
    } else if (pitch_angle_abs_err >= PITCH_FIX_DEADBAND) {
        fp32 soft_err = pitch_angle_err;
        if (soft_err > 0.0f) {
            soft_err -= PITCH_FIX_DEADBAND;
        } else {
            soft_err += PITCH_FIX_DEADBAND;
        }

        pitch_fix_pid_out = pid_calc(&up_pitch_position_fix, 0.0f, soft_err);
    } else {
        up_pitch_position_fix.dout = 0.0f;
        up_pitch_position_fix.err[LAST] = 0.0f;
        pitch_fix_pid_out = up_pitch_position_fix.iout;
    }

    VAL_LIMIT(pitch_fix_pid_out,
              -GIMBAL_UP_PITCH_FIX_MAX_OUT,
              GIMBAL_UP_PITCH_FIX_MAX_OUT)

    first_order_filter_cali(&gimbal.up_pitch_first_order_set, pitch_fix_pid_out);
    return gimbal.up_pitch_first_order_set.out *
           gimbal.up_pitch.motor_measure.offset_angle_to_position_ratio;
#else
    (void)target_angle;
    Gimbal_Up_Pitch_Position_Fix_Reset();
    return 0.0f;
#endif
}

static fp32 Gimbal_Up_Pitch_Hold_Torque(fp32 target_angle, uint8_t pitch_switching)
{
    fp32 pitch_err = target_angle - gimbal.up_pitch.absolute_angle_get;
    fp32 err_torque = pitch_err * UP_PITCH_ANGLE_TORQUE_KP;
    fp32 torque = UP_PITCH_HOLD_TORQUE;

    if (pitch_switching || Gimbal_Is_Down_Like_Pose(gimbal_pose_mode)) {
        return 0.0f;
    }

    VAL_LIMIT(err_torque,
              -UP_PITCH_ANGLE_TORQUE_MAX,
              UP_PITCH_ANGLE_TORQUE_MAX)

    torque += err_torque;
    VAL_LIMIT(torque, -UP_PITCH_TORQUE_MAX, UP_PITCH_TORQUE_MAX)
    return torque;
}

static void Gimbal_Pitch_Build_Cmd(fp32 *up_pos, fp32 *up_kp, fp32 *up_kd, fp32 *up_torque,
                                   fp32 *down_pos, fp32 *down_kp, fp32 *down_kd, fp32 *down_torque)
{
    fp32 up_pitch_smooth_offset = Pitch_Smooth_Position_Update(&gimbal.up_pitch);
    fp32 down_pitch_smooth_position = Pitch_Smooth_Position_Update(&gimbal.down_pitch);
    fp32 pitch_switch_progress = 1.0f;
    uint8_t pitch_switching = (!gimbal.up_pitch.smooth_arrived || !gimbal.down_pitch.smooth_arrived);
    uint8_t pitch_joint_plan = (pitch_switch_state == PITCH_SWITCH_JOINT_PLAN);
    uint8_t pitch_pose_switching = pitch_switching || pitch_joint_plan;
    uint8_t up_disable_exit_protect =
        pitch_joint_plan &&
        Gimbal_Is_Down_Like_Pose(pitch_switch_start_mode) &&
        (pitch_switch_target_mode == GIMBAL_UP);
    uint8_t pitch_static_fix_enable =
        (ABS(pitch_scale) <= UP_PITCH_FIX_INPUT_DEADBAND) &&
        (ABS(gyro_pitch) <= UP_PITCH_FIX_GYRO_DEADBAND);
    if (pitch_joint_plan) {
        fp32 down_range = pitch_switch_down_target_position - pitch_switch_down_start_position;
        fp32 abs_hold_fix = 0.0f;
        fp32 abs_hold_kp = Gimbal_Is_Down_Like_Pose(pitch_switch_target_mode) ?
                           PITCH_SWITCH_ABS_HOLD_KP :
                           PITCH_SWITCH_RECOVER_ABS_HOLD_KP;
        fp32 abs_hold_max = Gimbal_Is_Down_Like_Pose(pitch_switch_target_mode) ?
                            PITCH_SWITCH_ABS_HOLD_MAX_POS :
                            PITCH_SWITCH_RECOVER_ABS_HOLD_MAX_POS;

        if (ABS(down_range) > 1.0f) {
            pitch_switch_progress =
                (down_pitch_smooth_position - pitch_switch_down_start_position) / down_range;
            VAL_LIMIT(pitch_switch_progress, 0.0f, 1.0f)
        }

        up_pitch_smooth_offset = pitch_switch_up_start_position +
                                 (pitch_switch_up_target_position - pitch_switch_up_start_position) *
                                 pitch_switch_progress;
        abs_hold_fix = (gimbal.up_pitch.absolute_angle_get - PITCH_SWITCH_ABS_HOLD_TARGET) *
                       gimbal.up_pitch.motor_measure.offset_angle_to_position_ratio *
                       abs_hold_kp;
        VAL_LIMIT(abs_hold_fix,
                  -abs_hold_max,
                  abs_hold_max)
        up_pitch_smooth_offset += abs_hold_fix;
        gimbal.up_pitch.smooth_position = up_pitch_smooth_offset;
    }
    if (pitch_pose_switching) {
        gimbal.up_pitch.absolute_angle_set =
            Gimbal_Is_Down_Like_Pose(gimbal_pose_mode) ? 0.0f : pitch_switch_abs_angle_set;
    }
    fp32 up_pitch_angle_offset = (pitch_joint_plan || Gimbal_Is_Down_Like_Pose(gimbal_pose_mode)) ? 0.0f :
                                  gimbal.up_pitch.absolute_angle_set *
                                  gimbal.up_pitch.motor_measure.offset_angle_to_position_ratio;

    *up_kp = pitch_joint_plan ? gimbal.up_pitch.DM_kp :
             (pitch_switching ? UP_PITCH_MOVE_KP : gimbal.up_pitch.DM_kp);
    *up_kd = gimbal.up_pitch.DM_kd;
    *up_torque = pitch_joint_plan ? PITCH_SWITCH_LEVEL_UP_TORQUE :
                 Gimbal_Up_Pitch_Hold_Torque(gimbal.up_pitch.absolute_angle_set,
                                             pitch_switching);

    *down_pos = down_pitch_smooth_position;
    *down_kp = gimbal.down_pitch.smooth_arrived ? DOWN_PITCH_HOLD_KP : DOWN_PITCH_MOVE_KP;
    *down_kd = gimbal.down_pitch.smooth_arrived ? DOWN_PITCH_HOLD_KD : DOWN_PITCH_MOVE_KD;
    *down_torque = gimbal.down_pitch.smooth_arrived ? DOWN_PITCH_HOLD_TORQUE : 0.0f;

    *up_pos = up_pitch_smooth_offset + up_pitch_angle_offset;
    if (gimbal_up_disable_enter_pending) {
        fp32 up_disable_enter_err =
            GIMBAL_UP_DISABLE_ENTER_HOLD_ANGLE - gimbal.up_pitch.absolute_angle_get;
        fp32 up_disable_enter_fix =
            up_disable_enter_err *
            gimbal.up_pitch.motor_measure.offset_angle_to_position_ratio *
            GIMBAL_UP_DISABLE_ENTER_HOLD_KP;
        VAL_LIMIT(up_disable_enter_fix,
                  -GIMBAL_UP_DISABLE_ENTER_HOLD_MAX_POS,
                   GIMBAL_UP_DISABLE_ENTER_HOLD_MAX_POS)
        *up_pos += up_disable_enter_fix;
    }
    if (up_disable_exit_protect) {
        fp32 exit_hold_angle = PITCH_SWITCH_RECOVER_EXIT_HOLD_ANGLE;
        fp32 up_disable_exit_fix = 0.0f;
        if (pitch_switch_progress > PITCH_SWITCH_RECOVER_EXIT_HOLD_PROGRESS) {
            fp32 release_progress =
                (pitch_switch_progress - PITCH_SWITCH_RECOVER_EXIT_HOLD_PROGRESS) /
                (1.0f - PITCH_SWITCH_RECOVER_EXIT_HOLD_PROGRESS);
            VAL_LIMIT(release_progress, 0.0f, 1.0f)
            exit_hold_angle *= (1.0f - release_progress);
        }
        up_disable_exit_fix =
            (exit_hold_angle - gimbal.up_pitch.absolute_angle_get) *
            gimbal.up_pitch.motor_measure.offset_angle_to_position_ratio *
            PITCH_SWITCH_RECOVER_EXIT_HOLD_KP;
        VAL_LIMIT(up_disable_exit_fix,
                  -PITCH_SWITCH_RECOVER_EXIT_HOLD_MAX_POS,
                   PITCH_SWITCH_RECOVER_EXIT_HOLD_MAX_POS)
        *up_pos += up_disable_exit_fix;
    }

    if (!pitch_switching && !Gimbal_Is_Down_Like_Pose(gimbal_pose_mode)) {
        if (pitch_scale > UP_PITCH_BACKLASH_INPUT_DEADBAND) {
            up_pitch_backlash_comp = UP_PITCH_UP_BACKLASH_COMP_POS;
        } else if (pitch_scale < -UP_PITCH_BACKLASH_INPUT_DEADBAND) {
            up_pitch_backlash_comp = 0.0f;
        }

        if (gimbal.up_pitch.absolute_angle_set > UP_PITCH_BACKLASH_MIN_ANGLE) {
            *up_pos += up_pitch_backlash_comp;
        }
    } else {
        up_pitch_backlash_comp = 0.0f;
    }

    if (pitch_joint_plan) {
        angle_fix = Gimbal_Up_Pitch_Position_Fix(pitch_switch_angle_set);
        VAL_LIMIT(angle_fix, -PITCH_SWITCH_LEVEL_FIX_LIMIT, PITCH_SWITCH_LEVEL_FIX_LIMIT)
    } else if (pitch_switching || Gimbal_Is_Down_Like_Pose(gimbal_pose_mode) || !pitch_static_fix_enable) {
        angle_fix = 0.0f;
        Gimbal_Up_Pitch_Position_Fix_Reset();
    } else {
        angle_fix = Gimbal_Up_Pitch_Position_Fix(gimbal.up_pitch.absolute_angle_set);
    }

    *up_pos += angle_fix;
    if (pitch_switching) {
        VAL_LIMIT(*up_pos,
                  UP_PITCH_SWITCH_POSITION_MIN - 10,
                  UP_PITCH_SWITCH_POSITION_MAX + 50)
    } else {
        VAL_LIMIT(*up_pos,
                  gimbal.up_pitch.motor_measure.offset_position_min - 10,
                  gimbal.up_pitch.motor_measure.offset_position_max + 50)
    }

    if (gimbal_pose_mode == GIMBAL_UP_DISABLE) {
        *up_pos = 0.0f;
        *up_kp = 0.0f;
        *up_kd = 0.0f;
        *up_torque = 0.0f;
    }
}

void Gimbal_DM_Ctrl_Loop_Cal(void)
{
    yaw_cmd_t yaw_cmd = Gimbal_Yaw_Build_Cmd();
    fp32 up_pitch_position = 0.0f;
    fp32 up_pitch_kp = 0.0f;
    fp32 up_pitch_kd = 0.0f;
    fp32 up_pitch_torque = 0.0f;
    fp32 down_pitch_position = 0.0f;
    fp32 down_pitch_kp = 0.0f;
    fp32 down_pitch_kd = 0.0f;
    fp32 down_pitch_torque = 0.0f;

    Gimbal_Yaw_Servo(&yaw_cmd);

    Gimbal_Pitch_Build_Cmd(&up_pitch_position,
                           &up_pitch_kp,
                           &up_pitch_kd,
                           &up_pitch_torque,
                           &down_pitch_position,
                           &down_pitch_kp,
                           &down_pitch_kd,
                           &down_pitch_torque);

    gimbal.up_pitch.DM_MIT_position = up_pitch_position;
    gimbal.down_pitch.DM_MIT_position = down_pitch_position;

    fp32 yaw_dm_kd = Gimbal_Is_Down_Like_Pose(gimbal_pose_mode) ?
                     GIMBAL_DOWN_YAW_IMP_DM_KD :
                     GIMBAL_YAW_IMP_DM_KD;

    Gimbal_DM_Send_One(gimbal.up_pitch.DM_MIT_position,
                       up_pitch_kp,
                       up_pitch_kd,
                       up_pitch_torque,
                       gimbal.down_pitch.DM_MIT_position,
                       down_pitch_kp,
                       down_pitch_kd,
                       down_pitch_torque,
                       yaw_dm_kd,
                       gimbal.yaw.DM_MIT_torque);
}


/**
  * @brief          自瞄处理得到对云台电机的控制
  * @param[in]      none
  * 获取视觉发送的角度误差转动 pitch 轴和 yaw 轴
  * @retval         返回空
  */
float a_pitch = 0.f;
float a_yaw = 0.f;
void Gimbal_Auto_Handle(void) {
    if(KeyBoard.V.status == KEY_CLICK) a_pitch += 0.2f;
    if(KeyBoard.C.status == KEY_CLICK) a_pitch -= 0.2f;
    // if(KeyBoard.Z.status == KEY_CLICK) a_yaw += 0.2f;
    // if(KeyBoard.X.status == KEY_CLICK) a_yaw -= 0.2f;
    //获取视觉发送的角度误差
    first_order_filter_cali(&gimbal.auto_pitch, robot_ctrl.pitch);
    first_order_filter_cali(&gimbal.auto_yaw[0], sinf(robot_ctrl.yaw / 180.0f * PI));//yaw数据分解成x
    first_order_filter_cali(&gimbal.auto_yaw[1], cosf(robot_ctrl.yaw / 180.0f * PI));//yaw数据分解成y

    gimbal.yaw.absolute_angle_set = (atan2f(gimbal.auto_yaw[0].out, gimbal.auto_yaw[1].out) * 180.0f / PI) + a_yaw;//在此处做合成

    gimbal.up_pitch.relative_up_down_set = gimbal.auto_pitch.out + a_pitch;
    gimbal.up_pitch.absolute_angle_set = gimbal.auto_pitch.out + a_pitch;

    //云台绕圈时进行绝对角循环设置
    if(gimbal.yaw.absolute_angle_set>=180){
        gimbal.yaw.absolute_angle_set-=360;
    }
    else if(gimbal.yaw.absolute_angle_set<=-180){
        gimbal.yaw.absolute_angle_set+=360;
    }

    if (crazy_cycle_exit_pending && Gimbal_Pose_Control_Enabled()) {
        gimbal.yaw.absolute_angle_set = Gimbal_Yaw_Get_Center_Target();
    }

    //TODO:pitch软件限位
    // if (gimbal.mode == GIMBAL_UP) {
    //     gimbal.up_pitch.absolute_angle_set = fp32_constrain(gimbal.up_pitch.absolute_angle_set,
    //                                                        -8,
    //                                                        50);
    // }else if (gimbal.mode == GIMBAL_MIDDLE) {
    //     gimbal.up_pitch.absolute_angle_set = fp32_constrain(gimbal.up_pitch.absolute_angle_set,
    //                                                        -20,
    //                                                        45);
    // }
    gimbal.up_pitch.absolute_angle_set = fp32_constrain(gimbal.up_pitch.absolute_angle_set,
                                                           -8,
                                                           50);
    //小陀螺限制pitch角度，避免测速撞到电池
    if (Gimbal_Is_Crazy_Cycle_Pose(gimbal_pose_mode) ||
        gimbal.chassis_mode == CHASSIS_SPIN_R || gimbal.chassis_mode == CHASSIS_SPIN_L) {
        gimbal.up_pitch.absolute_angle_set = fp32_constrain(gimbal.up_pitch.absolute_angle_set,
                                                           MIN_SPIN_ANGLE,
                                                           MAX_SPIN_ANGLE);
    }
    // // 目标位置
    // gimbal.up_pitch.DM_MIT_position = gimbal.up_pitch.offset_position + gimbal.up_pitch.absolute_angle_set * MOTOR_ANGLE_TO_ECD;
    // gimbal.down_pitch.DM_MIT_position = gimbal.down_pitch.offset_position;
    // gimbal.up_pitch.DM_MIT_torque = 2.0f;//PITCH_GRAVITY_COMPENSATION_UP * sinf((gimbal.up_pitch.DM_MIT_position - UP_PITCH_OFFSET_ECD) * 0.001f);
    // gimbal.down_pitch.DM_MIT_torque = PITCH_GRAVITY_COMPENSATION_DOWN * sinf((gimbal.down_pitch.smooth_position - DOWN_PITCH_OFFSET_ECD) * 0.001f);

}


void Gimbal_Can_Msg(uint8_t can,uint32_t can_id, uint8_t *can_msg) {
    /*达妙电机*/
    if (can ==CAN_2)
    {
        switch (can_id) {

            case MASTER_DM_MOTOR_UP_PITCH_ID: // 0x11
                DM_Motor_Decode(&gimbal.up_pitch.motor_measure, CAN_2, MASTER_DM_MOTOR_UP_PITCH_ID, can_msg);
                detect_handle(DETECT_GIMBAL_UP_PITCH);
                break;
            case MASTER_DM_MOTOR_DOWN_PITCH_ID: // 0x12
                DM_Motor_Decode(&gimbal.down_pitch.motor_measure, CAN_2, MASTER_DM_MOTOR_DOWN_PITCH_ID, can_msg);
                detect_handle(DETECT_GIMBAL_DOWN_PITCH);
                break;
            case MASTER_DM_MOTOR_YAW_ID:   // 0x13
                DM_Motor_Decode(&gimbal.yaw.motor_measure, CAN_2, MASTER_DM_MOTOR_YAW_ID, can_msg);
                detect_handle(DETECT_GIMBAL_YAW);
                break;
            case CAN_LAUNCHER_TRIGGER: //201
                DJI_Motor_Decode(&launcher.trigger.motor_measure, can_msg);
                DJI_Round_Count(&launcher.trigger.motor_measure);//获取转动拨轮电机转动圈数和总编码值
                detect_handle(DETECT_LAUNCHER_3508_TRIGGER);
                break;
            default:
                break;
        }
    }

    /*DJI电机*/
    else if (can ==CAN_1)
    {
        switch (can_id) {
            case CAN_LAUNCHER_FIRE_LEFT: //203
                DJI_Motor_Decode(&launcher.fire_l.motor_measure, can_msg);
                detect_handle(DETECT_LAUNCHER_3508_FIRE_L);
                break;

            case CAN_LAUNCHER_FIRE_RIGHT: //204
                DJI_Motor_Decode(&launcher.fire_r.motor_measure, can_msg);
                detect_handle(DETECT_LAUNCHER_3508_FIRE_R);
                break;
            default:
                break;
        }
    }
}

void Chassis_to_Gimbal_Can(uint32_t can_id, const uint8_t *rx_data) {
    switch (can_id) {
        case 0x113: {
            union angle angle;
            {
                angle.data[0] = rx_data[0];
                angle.data[1] = rx_data[1];
                angle.data[2] = rx_data[2];
                angle.data[3] = rx_data[3];
            }
            gimbal.up_pitch.absolute_angle_get_down = angle.value;
        }break;

        case 0x114: {
            union chassis_referee power;
            union chassis_referee power_limit;
            union chassis_referee heat_limit;
            {
                power.data[0] = rx_data[0];
                power.data[1] = rx_data[1];

                power_limit.data[0] = rx_data[2];
                power_limit.data[1] = rx_data[3];
                Referee.GameRobotStat.robot_id = rx_data[4];
                heat_limit.data[0] = rx_data[5];
                heat_limit.data[1] = rx_data[6];
            }
            Referee.PowerHeatData.buffer_energy = power.referee;
            Referee.GameRobotStat.chassis_power_limit = power_limit.referee;
            Referee.GameRobotStat.shooter_barrel_heat_limit = heat_limit.referee;
        }
            break;

        case 0x115: {
            uint16_t bullet_speed_x100 = (uint16_t)rx_data[0] |
                                         ((uint16_t)rx_data[1] << 8U);
            uint16_t shooter_heat = (uint16_t)rx_data[2] |
                                    ((uint16_t)rx_data[3] << 8U);
            uint8_t game_progress = rx_data[4];
            uint8_t shoot_seq = rx_data[5];
            uint16_t projectile_allowance_42mm = (uint16_t)rx_data[6] |
                                                 ((uint16_t)rx_data[7] << 8U);

            Referee.ShootData.bullet_speed = (fp32)bullet_speed_x100 * 0.01f;
            Referee.PowerHeatData.shooter_42mm_barrel_heat = shooter_heat;
            Referee.GameState.game_progress = game_progress;
            Referee.BulletRemaining.projectile_allowance_42mm = projectile_allowance_42mm;
            Gimbal_Yaw_Ammo_Update_From_Referee(game_progress, shoot_seq);
        }
            break;
        default:
            break;
    }
}
