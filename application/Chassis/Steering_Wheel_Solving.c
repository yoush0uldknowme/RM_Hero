//
// Created by SyLin on 25-9-11.
//

#include "Steering_Wheel_Solving.h"
#include "Atti.h"
#include "Cap.h"
#include "Chassis.h"
#include "PowerLimit_Steering.h"

fp32 Steering_Direction[4];
static fp32 power_limit = 0.0f;
int16_t before_current[4];
int16_t after_current[4];

/**
  * @brief          舵轮方向初始化
  * @param[in]      pvParameters
  * 计算舵轮的速度
  * @retval         none
  */
//已弃用
void Chassis_Steering_Wheel_Init(void){

  	// Steering_Direction[RF] = RF_DIRECTION;
  	// Steering_Direction[LF] = LF_DIRECTION;
  	// Steering_Direction[LB] = LB_DIRECTION;
  	// Steering_Direction[RB] = RB_DIRECTION;

    chassis.Steering_motor_chassis[RF].motor_measure.offset_ecd = RF_STEERING_OFFSET_ECD;
    chassis.Steering_motor_chassis[LF].motor_measure.offset_ecd = LF_STEERING_OFFSET_ECD;
    chassis.Steering_motor_chassis[LB].motor_measure.offset_ecd = LB_STEERING_OFFSET_ECD;
    chassis.Steering_motor_chassis[RB].motor_measure.offset_ecd = RB_STEERING_OFFSET_ECD;
}

/**
  * @brief          舵轮解算
  * @param[in]      pvParameters
  * 计算舵轮的速度
  * @retval         none
  */

// void Chassis_Steering_Wheel_Cal(void) {
//     fp32 Vx[4];
//     fp32 Vy[4];
//     fp32 drive_wheel_rpm[4];
//     fp32 Steering_Wheel_Target_angle[4];
//     fp32 max = 0;
//
//
//     // 根据给定的底盘运动参数，计算四个轮子的转速
//     Vx[RF] = chassis.vx - chassis.vw * sqrt(1/2);
//     Vx[LF] = chassis.vx + chassis.vw * sqrt(1/2);
//     Vx[LB] = chassis.vx + chassis.vw * sqrt(1/2);
//     Vx[RB] = chassis.vx - chassis.vw * sqrt(1/2);
//
//     Vy[RF] = chassis.vy - chassis.vw * sqrt(1/2);
//     Vy[LF] = chassis.vy - chassis.vw * sqrt(1/2);
//     Vy[LB] = chassis.vy + chassis.vw * sqrt(1/2);
//     Vy[RB] = chassis.vy + chassis.vw * sqrt(1/2);
//
//     drive_wheel_rpm[RF] =sqrt(pow( Vx[RF] , 2) + pow( Vy[RF] , 2));
//     drive_wheel_rpm[LF] =sqrt(pow( Vx[LF] , 2) + pow( Vy[LF] , 2));
//     drive_wheel_rpm[LB] =sqrt(pow( Vx[LB] , 2) + pow( Vy[LB] , 2));
//     drive_wheel_rpm[RB] =sqrt(pow( Vx[RB] , 2) + pow( Vy[RB] , 2));
//
//     for (uint8_t i = 0; i < 4; i++){
//         Steering_Wheel_Target_angle[i] = atan2(Vy[i] , Vx[i]);
//         if (Steering_Wheel_Target_angle[i] > PI / 2.0f) {
//             Steering_Wheel_Target_angle[i] -= PI;
//             chassis.drive_motor_chassis[i].Direction = -Steering_Direction[i];
//         } else if (Steering_Wheel_Target_angle[i] < -PI / 2.0f) {
//             Steering_Wheel_Target_angle[i] += PI;
//             chassis.drive_motor_chassis[i].Direction = -Steering_Direction[i];
//         } else {
//             chassis.drive_motor_chassis[i].Direction = Steering_Direction[i];
//         }
//     }
//     for (uint8_t i = 0; i < 4; i++){
//       chassis.Steering_motor_chassis[i].relative_angle_get= Motor_Ecd_To_Angle_Change(chassis.Steering_motor_chassis[i].motor_measure.ecd,
//                                                                chassis.Steering_motor_chassis[i].motor_measure.offset_ecd);
//     }
//
//     for (uint8_t i = 0; i < 4; i++) {
//         if (ABS(drive_wheel_rpm[i]) > max) max = ABS(drive_wheel_rpm[i]);
//     }
//
//     // 如果超过最大转速，对所有轮子转速进行缩放，使其不超过最大转速
//     if (max > M3508_MAX_RPM) {
//         float rate = M3508_MAX_RPM / max;
//         for (uint8_t i = 0; i < 4; i++) drive_wheel_rpm[i] *= rate;
//     }
//
// //    //上坡功率重新分配
// //
// //    chassis.board_pitch=INS_angle[2]*MOTOR_RAD_TO_ANGLE;
// //    if(abs(chassis.board_pitch) > 8)
// //    {
// //        fp32 ramp_p = 0; //上坡功率分配系数p
// //        ramp_p = 0.6f * sinf(-INS_angle[2]);
// //        VAL_LIMIT(ramp_p,0,0.25f);
// //        wheel_rpm[0] *= (1 - 0.7f * sinf(-INS_angle[2]));
// //        wheel_rpm[1] *= (1 - 0.7f * sinf(-INS_angle[2]));
// //        wheel_rpm[2] *= (1 + ramp_p);
// //        wheel_rpm[3] *= (1 + ramp_p);
// //    }
//
//     // 设置给定驱动轮的转速
//     chassis.drive_motor_chassis[RF].rpm_set=chassis.drive_motor_chassis[RF].Direction * drive_wheel_rpm[RF]*1000;
//     chassis.drive_motor_chassis[LF].rpm_set=chassis.drive_motor_chassis[LF].Direction * drive_wheel_rpm[LF]*1000;
//     chassis.drive_motor_chassis[RB].rpm_set=chassis.drive_motor_chassis[RB].Direction * drive_wheel_rpm[RB]*1000;
//     chassis.drive_motor_chassis[LB].rpm_set=chassis.drive_motor_chassis[LB].Direction * drive_wheel_rpm[LB]*1000;
//
//     // 设置给定轮子目标角度
//     chassis.Steering_motor_chassis[RF].relative_angle_set = Steering_Wheel_Target_angle[RF] * RAD_TO_DEG;
//     chassis.Steering_motor_chassis[LF].relative_angle_set = Steering_Wheel_Target_angle[LF] * RAD_TO_DEG;
//     chassis.Steering_motor_chassis[RB].relative_angle_set = Steering_Wheel_Target_angle[RB] * RAD_TO_DEG;
//     chassis.Steering_motor_chassis[LB].relative_angle_set = Steering_Wheel_Target_angle[LB] * RAD_TO_DEG;
// }

// void Chassis_Steering_Wheel_Cal(void) {
//     fp32 Vx[4];
//     fp32 Vy[4];
//     fp32 drive_wheel_rpm[4];
//     fp32 Steering_Wheel_Target_angle[4];
//
//     const fp32 L = sqrtf(0.5f);
//
//     Vx[RF] = chassis.vx - chassis.vw * L;
//     Vx[LF] = chassis.vx + chassis.vw * L;
//     Vx[LB] = chassis.vx + chassis.vw * L;
//     Vx[RB] = chassis.vx - chassis.vw * L;
//
//     Vy[RF] = chassis.vy - chassis.vw * L;
//     Vy[LF] = chassis.vy - chassis.vw * L;
//     Vy[LB] = chassis.vy + chassis.vw * L;
//     Vy[RB] = chassis.vy + chassis.vw * L;
//
//     fp32 max_rpm = 0.0f;
//
//     for (uint8_t i = 0; i < 4; i++) {
//         fp32 speed = sqrtf(Vx[i] * Vx[i] + Vy[i] * Vy[i]);
//
//         if (speed < 1e-3f) {
//             // 保持上一次的目标角度
//             Steering_Wheel_Target_angle[i] =
//                 chassis.Steering_motor_chassis[i].relative_angle_set * DEG_TO_RAD;
//             drive_wheel_rpm[i] = 0.0f;
//             chassis.drive_motor_chassis[i].Direction = 1;
//         } else {
//             // 正常计算目标角度
//             fp32 angle = atan2f(Vy[i], Vx[i]); // 范围 (-π, π]
//
//             // 利用180°对称性：映射到 [-π/2, π/2]
//             if (angle > PI / 2.0f) {
//                 angle -= PI;
//                 chassis.drive_motor_chassis[i].Direction = -1.0f;
//             } else if (angle < -PI / 2.0f) {
//                 angle += PI;
//                 chassis.drive_motor_chassis[i].Direction = -1.0f;
//             } else {
//                 chassis.drive_motor_chassis[i].Direction = 1.0f;
//             }
//
//             //根据当前轮子实际角度校正驱动方向
//             fp32 current_angle = chassis.Steering_motor_chassis[i].relative_angle_get * DEG_TO_RAD;
//             fp32 angle_diff = angle - current_angle;
//             while (angle_diff > PI) angle_diff -= 2.0f * PI;
//             while (angle_diff < -PI) angle_diff += 2.0f * PI;
//             if (fabsf(angle_diff) > PI / 2.0f) {
//                 chassis.drive_motor_chassis[i].Direction = -chassis.drive_motor_chassis[i].Direction;
//             }
//
//             Steering_Wheel_Target_angle[i] = angle;
//             drive_wheel_rpm[i] = speed;
//         }
//
//         // 更新最大转速
//         if (drive_wheel_rpm[i] > max_rpm) {
//             max_rpm = drive_wheel_rpm[i];
//         }
//     }
//
//     //归一化
//     if (max_rpm > M3508_MAX_RPM / LINEAR_SPEED_TO_RPM) {
//         fp32 rate = (M3508_MAX_RPM / LINEAR_SPEED_TO_RPM) / max_rpm;
//         for (uint8_t i = 0; i < 4; i++) {
//             drive_wheel_rpm[i] *= rate;
//         }
//     }
//
//     //设置驱动电机目标转速
//     for (uint8_t i = 0; i < 4; i++) {
//         chassis.drive_motor_chassis[i].rpm_set =
//             chassis.drive_motor_chassis[i].Direction * drive_wheel_rpm[i] * LINEAR_SPEED_TO_RPM;
//     }
//
//     // 设置转向电机目标角度
//     for (uint8_t i = 0; i < 4; i++) {
//         chassis.Steering_motor_chassis[i].relative_angle_set =
//             Steering_Wheel_Target_angle[i] * RAD_TO_DEG;
//     }
//
//     //更新当前转向角度反馈
//     for (uint8_t i = 0; i < 4; i++) {
//         chassis.Steering_motor_chassis[i].relative_angle_get =
//             Motor_Ecd_To_Angle_Change(
//                 chassis.Steering_motor_chassis[i].motor_measure.ecd,
//                 chassis.Steering_motor_chassis[i].motor_measure.offset_ecd
//             );
//     }
//     chassis.drive_motor_chassis[RF].rpm_set *= RF_DIRECTION;
//     chassis.drive_motor_chassis[LF].rpm_set *= LF_DIRECTION;
//     chassis.drive_motor_chassis[LB].rpm_set *= LB_DIRECTION;
//     chassis.drive_motor_chassis[RB].rpm_set *= RB_DIRECTION;
// }

fp32 Steering_Wheel_Target_angle_3;

void Chassis_Steering_Wheel_Cal(void) {
    fp32 Vx[4];
    fp32 Vy[4];
    fp32 drive_wheel_rpm[4];
    fp32 Steering_Wheel_Target_angle[4];

    // 轮子位置：(±0.5, ±0.5)，所以 Lx = Ly = 0.5f

    Vx[RF] = chassis.vx - chassis.vw * 0.5f;  // -vw * y_RF, y_RF = +0.5
    Vy[RF] = chassis.vy + chassis.vw * 0.5f;  // +vw * x_RF, x_RF = +0.5

    Vx[LF] = chassis.vx + chassis.vw * 0.5f;  // -vw * y_LF, y_LF = -0.5 → -vw*(-0.5) = +vw*0.5
    Vy[LF] = chassis.vy + chassis.vw * 0.5f;  // +vw * x_LF, x_LF = +0.5

    Vx[LB] = chassis.vx + chassis.vw * 0.5f;  // -vw * y_LB, y_LB = -0.5
    Vy[LB] = chassis.vy - chassis.vw * 0.5f;  // +vw * x_LB, x_LB = -0.5

    Vx[RB] = chassis.vx - chassis.vw * 0.5f;  // -vw * y_RB, y_RB = +0.5
    Vy[RB] = chassis.vy - chassis.vw * 0.5f;  // +vw * x_RB, x_RB = -0.5

    fp32 max_rpm = 0.0f;

    for (uint8_t i = 0; i < 4; i++) {
        fp32 speed = sqrtf(Vx[i] * Vx[i] + Vy[i] * Vy[i]);

        //零速保护：速度接近0时，保持当前角度，避免抖动
        if (speed < 1e-3f) {
            // 保持上一次的目标角度
            Steering_Wheel_Target_angle[i] =
                chassis.Steering_motor_chassis[i].relative_angle_set * DEG_TO_RAD;
            drive_wheel_rpm[i] = 0.0f;
            chassis.drive_motor_chassis[i].Direction = 1;
        } else {
            // 正常计算目标角度
            fp32 angle = atan2f(Vy[i], Vx[i]); // 范围 (-π, π]

            // 利用180°对称性：映射到 [-π/2, π/2]
            if (angle > PI / 2.0f) {
                angle -= PI;
                chassis.drive_motor_chassis[i].Direction = -1.0f;
            } else if (angle < -PI / 2.0f) {
                angle += PI;
                chassis.drive_motor_chassis[i].Direction = -1.0f;
            } else {
                chassis.drive_motor_chassis[i].Direction = 1.0f;
            }

            //根据当前轮子实际角度校正驱动方向
            fp32 current_angle = chassis.Steering_motor_chassis[i].relative_angle_get * DEG_TO_RAD;
            fp32 angle_diff = angle - current_angle;
            while (angle_diff > PI) angle_diff -= 2.0f * PI;
            while (angle_diff < -PI) angle_diff += 2.0f * PI;
            if (fabsf(angle_diff) > PI / 2.0f) {
                chassis.drive_motor_chassis[i].Direction = -chassis.drive_motor_chassis[i].Direction;
            }

            Steering_Wheel_Target_angle[i] = angle;
            drive_wheel_rpm[i] = speed;
        }

        // 更新最大转速
        if (drive_wheel_rpm[i] > max_rpm) {
            max_rpm = drive_wheel_rpm[i];
        }
    }

    //归一化
    if (max_rpm > M3508_MAX_RPM / LINEAR_SPEED_TO_RPM) {
        fp32 rate = (M3508_MAX_RPM / LINEAR_SPEED_TO_RPM) / max_rpm;
        for (uint8_t i = 0; i < 4; i++) {
            drive_wheel_rpm[i] *= rate;
        }
    }

    //设置驱动电机目标转速
    for (uint8_t i = 0; i < 4; i++) {
        chassis.drive_motor_chassis[i].rpm_set =
            chassis.drive_motor_chassis[i].Direction * drive_wheel_rpm[i] * LINEAR_SPEED_TO_RPM;
    }

    // 设置转向电机目标角度
    for (uint8_t i = 0; i < 4; i++) {
        chassis.Steering_motor_chassis[i].relative_angle_set =
            Steering_Wheel_Target_angle[i] * RAD_TO_DEG;
    }

    //更新当前转向角度反馈
    for (uint8_t i = 0; i < 4; i++) {
        chassis.Steering_motor_chassis[i].relative_angle_get =
            Motor_Ecd_To_Angle_Change(
                chassis.Steering_motor_chassis[i].motor_measure.ecd,
                chassis.Steering_motor_chassis[i].motor_measure.offset_ecd
            );
    }
    chassis.drive_motor_chassis[RF].rpm_set *= RF_DIRECTION;
    chassis.drive_motor_chassis[LF].rpm_set *= LF_DIRECTION;
    chassis.drive_motor_chassis[LB].rpm_set *= LB_DIRECTION;
    chassis.drive_motor_chassis[RB].rpm_set *= RB_DIRECTION;
}

fp32 steering_predicted = 0.f;
fp32 drive_predicted = 0.f;
fp32 predicted_power = 0.f;
/**
  * @brief          功率控制
  * @param[in]      pvParameters
  * @retval         none
*/
void Chassis_Power_Limit_Solve() {
    // chassis.power_limit.power_boost == 1;
    int16_t Steering_give_current[4], drive_give_current[4];
    for (int i = 0; i < 4; i++)
    {
        Steering_give_current[i] = chassis.Steering_motor_chassis[i].give_current;
        drive_give_current[i] = chassis.drive_motor_chassis[i].give_current;
    }
    // 正解预测功率
    Chassis_Power_Ctrl_Handler(Steering_give_current, drive_give_current);
    
    // 从裁判系统获取或设置功率限制值，确保有初始值
    if (detect_list[DETECT_REFEREE].status == ONLINE) {
        chassis.power_limit.chassis_power_limit = Referee.GameRobotStat.chassis_power_limit;
    } else {
        chassis.power_limit.chassis_power_limit = 50;  // 默认50W
    }
    
    power_limit = chassis.power_limit.chassis_power_limit;
    
    // 计算转向电机的总预测功率
    fp32 steering_predicted_power = 0;
    steering_predicted_power += Steering_RF_Limiter.predictPower;
    steering_predicted_power += Steering_LF_Limiter.predictPower;
    steering_predicted_power += Steering_LB_Limiter.predictPower;
    steering_predicted_power += Steering_RB_Limiter.predictPower;

    fp32 drive_predicted_power = 0;
    drive_predicted_power += Drive_RF_Limiter.predictPower;
    drive_predicted_power += Drive_LF_Limiter.predictPower;
    drive_predicted_power += Drive_LB_Limiter.predictPower;
    drive_predicted_power += Drive_RB_Limiter.predictPower;

    steering_predicted = steering_predicted_power;
    drive_predicted = drive_predicted_power;

    predicted_power = steering_predicted_power + drive_predicted_power;

        // 从总功率中扣除转向电机功率，剩余分配给驱动电机
        fp32 drive_power_budget = power_limit - steering_predicted_power;

        // 确保不为负
        if(drive_power_budget < 0) drive_power_budget = 0;

        if(power_boost_on == 0)
        {
            // 转向电机不限制功率，驱动电机获得剩余功率
            powerSchedulerUpdate(&Drive_scheduler, (int16_t)drive_power_budget);
        }
        else if(power_boost_on == 1)
        {
            // 通过能量环输出增加底盘输出的总功率
            if(cap_is_on == 0) {
                // 如果电容离线，闭环缓冲能量
                pid_calc(&buffer_energy_cl, Referee.PowerHeatData.buffer_energy, 40);
                drive_power_budget += buffer_energy_cl.out;
            } else {
                // 如果电容在线，闭环电容能量
                drive_power_budget += cap_Get_Boosting_Power(25.0f, 10.0f, 5.0f);
            }
            // 转向电机不限制功率，驱动电机获得剩余功率（含boost）
            powerSchedulerUpdate(&Drive_scheduler, (int16_t)drive_power_budget);
        }

        // 转向轮不限制功率，直接输出
        // 驱动轮应用功率限制
        drive_give_current[RF] = powerGetLimiterUpdate(&Drive_RF_Limiter, drive_give_current[RF]);
        drive_give_current[LF] = powerGetLimiterUpdate(&Drive_LF_Limiter, drive_give_current[LF]);
        drive_give_current[LB] = powerGetLimiterUpdate(&Drive_LB_Limiter, drive_give_current[LB]);
        drive_give_current[RB] = powerGetLimiterUpdate(&Drive_RB_Limiter, drive_give_current[RB]);

        for (int i = 0; i < 4; i++)
        {
            chassis.Steering_motor_chassis[i].give_current = Steering_give_current[i];
            chassis.drive_motor_chassis[i].give_current = drive_give_current[i];
        }
    for (int i = 0; i < 4; i++) {
        after_current[i] = chassis.Steering_motor_chassis[i].give_current;
    }
}


/**
  * @brief          舵轮电流给定
  * @param[in]      pvParameters
  * pid调整给定的舵轮电流
  * @retval         none
  * 有人问为什么不用循环要把每一项都列出来，我的回答是好看--SkLin
*/
void Chassis_Wheel_Loop_Cal(void) {
  	//驱动
    ramp_calc(&chassis.chassis_ramp[RF],chassis.drive_motor_chassis[RF].rpm_set);
    chassis.drive_motor_chassis[RF].give_current= (int16_t)pid_calc(&chassis.drive_motor_chassis[RF].speed_p,
                                                              chassis.drive_motor_chassis[RF].motor_measure.speed_rpm,
                                                              chassis.drive_motor_chassis[RF].rpm_set);

    ramp_calc(&chassis.chassis_ramp[LF],chassis.drive_motor_chassis[LF].rpm_set);
    chassis.drive_motor_chassis[LF].give_current= (int16_t)pid_calc(&chassis.drive_motor_chassis[LF].speed_p,
                                                              chassis.drive_motor_chassis[LF].motor_measure.speed_rpm,
                                                              chassis.drive_motor_chassis[LF].rpm_set);

    ramp_calc(&chassis.chassis_ramp[LB],chassis.drive_motor_chassis[LB].rpm_set);
    chassis.drive_motor_chassis[LB].give_current= (int16_t)pid_calc(&chassis.drive_motor_chassis[LB].speed_p,
                                                              chassis.drive_motor_chassis[LB].motor_measure.speed_rpm,
                                                              chassis.drive_motor_chassis[LB].rpm_set);

    ramp_calc(&chassis.chassis_ramp[RB],chassis.drive_motor_chassis[RB].rpm_set);
    chassis.drive_motor_chassis[RB].give_current= (int16_t)pid_calc(&chassis.drive_motor_chassis[RB].speed_p,
                                                              chassis.drive_motor_chassis[RB].motor_measure.speed_rpm,
                                                              chassis.drive_motor_chassis[RB].rpm_set);

    //转向
    //RF
    chassis.Steering_motor_chassis[RF].gyro_set= pid_loop_calc(&chassis.Steering_motor_chassis[RF].angle_p,
                                       chassis.Steering_motor_chassis[RF].relative_angle_get,
                                       chassis.Steering_motor_chassis[RF].relative_angle_set,
                                       180,
                                       0);

    chassis.Steering_motor_chassis[RF].give_current = (int16_t)pid_calc(&chassis.Steering_motor_chassis[RF].speed_p,
                                                chassis.Steering_motor_chassis[RF].motor_measure.speed_rpm,
                                                chassis.Steering_motor_chassis[RF].gyro_set);

    //LF
    chassis.Steering_motor_chassis[LF].gyro_set= pid_loop_calc(&chassis.Steering_motor_chassis[LF].angle_p,
                                       chassis.Steering_motor_chassis[LF].relative_angle_get,
                                       chassis.Steering_motor_chassis[LF].relative_angle_set,
                                       180,
                                       0);

    chassis.Steering_motor_chassis[LF].give_current = (int16_t)pid_calc(&chassis.Steering_motor_chassis[LF].speed_p,
                                                chassis.Steering_motor_chassis[LF].motor_measure.speed_rpm,
                                                chassis.Steering_motor_chassis[LF].gyro_set);

    //LB
    chassis.Steering_motor_chassis[LB].gyro_set= pid_loop_calc(&chassis.Steering_motor_chassis[LB].angle_p,
                                       chassis.Steering_motor_chassis[LB].relative_angle_get,
                                       chassis.Steering_motor_chassis[LB].relative_angle_set,
                                       180,
                                       0);

    chassis.Steering_motor_chassis[LB].give_current = (int16_t)pid_calc(&chassis.Steering_motor_chassis[LB].speed_p,
                                                chassis.Steering_motor_chassis[LB].motor_measure.speed_rpm,
                                                chassis.Steering_motor_chassis[LB].gyro_set);

    //RB
    chassis.Steering_motor_chassis[RB].gyro_set= pid_loop_calc(&chassis.Steering_motor_chassis[RB].angle_p,
                                       chassis.Steering_motor_chassis[RB].relative_angle_get,
                                       chassis.Steering_motor_chassis[RB].relative_angle_set,
                                       180,
                                       0);

    chassis.Steering_motor_chassis[RB].give_current = (int16_t)pid_calc(&chassis.Steering_motor_chassis[RB].speed_p,
                                                chassis.Steering_motor_chassis[RB].motor_measure.speed_rpm,
                                                chassis.Steering_motor_chassis[RB].gyro_set);


    Chassis_Power_Limit_Solve();
}

