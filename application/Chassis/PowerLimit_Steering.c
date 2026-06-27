//
// Created by SyLin on 26-1-26.
//

#include "PowerLimit_Steering.h"
#include "PowerLimiter.h"
#include "PID.h"
#include "EnergyConfig.h"
#include "Chassis.h"

//功率控制
Limiter_t Steering_LF_Limiter;
Limiter_t Steering_RF_Limiter;
Limiter_t Steering_RB_Limiter;
Limiter_t Steering_LB_Limiter;
Limiter_t Drive_RF_Limiter;
Limiter_t Drive_LF_Limiter;
Limiter_t Drive_LB_Limiter;
Limiter_t Drive_RB_Limiter;
// 限制器列表
Limiter_Handle Steering_list[4] = {
    &Steering_LF_Limiter, &Steering_RF_Limiter, &Steering_RB_Limiter, &Steering_LB_Limiter
};

Limiter_Handle Drive_list[4] = {
    &Drive_RF_Limiter, &Drive_LF_Limiter, &Drive_LB_Limiter, &Drive_RB_Limiter
};
// 功率限制器调度
LimiterScheduler_t Steering_scheduler;
LimiterScheduler_t Drive_scheduler;
// 缓冲能量控制器，适用于power_boost模式
pid_type_def buffer_energy_cl;


void PowerLimit_Steering_init() {
    powerInitialiseLimiter(&Steering_LF_Limiter, MODEL_6020);
    powerInitialiseLimiter(&Steering_RF_Limiter, MODEL_6020);
    powerInitialiseLimiter(&Steering_RB_Limiter, MODEL_6020);
    powerInitialiseLimiter(&Steering_LB_Limiter, MODEL_6020);
    powerInitialiseLimiter(&Drive_RF_Limiter, MODEL_3508);
    powerInitialiseLimiter(&Drive_LF_Limiter, MODEL_3508);
    powerInitialiseLimiter(&Drive_LB_Limiter, MODEL_3508);
    powerInitialiseLimiter(&Drive_RB_Limiter, MODEL_3508);
    powerInitialiseLimiterScheduler(&Steering_scheduler, 4, Steering_list, 400.0f, 50.0f);
    powerInitialiseLimiterScheduler(&Drive_scheduler, 4, Drive_list, 120.0f, 20.0f);
    // 初始化缓冲能量控制器
    pid_init(&buffer_energy_cl,
             ENERGY_CL_MAX_OUTPUT,
             ENERGY_CL_MAX_INTEGRAL,
             ENERGY_CL_P,
             ENERGY_CL_I,
             ENERGY_CL_D);
}

void Chassis_Power_Ctrl_Handler(int16_t steering[4], int16_t drive[4]) {
    // 正向解出预测功率
    powerLimiterUpdate(&Steering_RF_Limiter, chassis.Steering_motor_chassis[RF].motor_measure.speed_rpm,
                       steering[RF], chassis.Steering_motor_chassis[RF].speed_p.err[0]);
    powerLimiterUpdate(&Steering_LF_Limiter, chassis.Steering_motor_chassis[LF].motor_measure.speed_rpm,
                       steering[LF], chassis.Steering_motor_chassis[LF].speed_p.err[0]);
    powerLimiterUpdate(&Steering_LB_Limiter, chassis.Steering_motor_chassis[LB].motor_measure.speed_rpm,
                       steering[LB], chassis.Steering_motor_chassis[LB].speed_p.err[0]);
    powerLimiterUpdate(&Steering_RB_Limiter, chassis.Steering_motor_chassis[RB].motor_measure.speed_rpm,
                       steering[RB], chassis.Steering_motor_chassis[RB].speed_p.err[0]);
    powerLimiterUpdate(&Drive_RF_Limiter, chassis.drive_motor_chassis[RF].motor_measure.speed_rpm,
                       drive[RF], chassis.drive_motor_chassis[RF].speed_p.err[0]);
    powerLimiterUpdate(&Drive_LF_Limiter, chassis.drive_motor_chassis[LF].motor_measure.speed_rpm,
                       drive[LF], chassis.drive_motor_chassis[LF].speed_p.err[0]);
    powerLimiterUpdate(&Drive_LB_Limiter, chassis.drive_motor_chassis[LB].motor_measure.speed_rpm,
                       drive[LB], chassis.drive_motor_chassis[LB].speed_p.err[0]);
    powerLimiterUpdate(&Drive_RB_Limiter, chassis.drive_motor_chassis[RB].motor_measure.speed_rpm,
                       drive[RB], chassis.drive_motor_chassis[RB].speed_p.err[0]);
}

void chassis_power_limit_steering(void) {
    int16_t give_current_limit;
    chassis.power_limit.total_current = 0;
    chassis.power_limit.total_current_limit = 0;
    fp32 power_buffer = chassis.power_limit.chassis_power_buffer;
    fp32 limit_k;
    if (detect_list[DETECT_REFEREE].status != ONLINE) {
        chassis.chassis_power_limit.total_current_limit = CHASSIS_CURRENT_LIMIT_40W;
    } else {
        chassis.power_limit.chassis_power_buffer = Referee.PowerHeatData.buffer_energy > CHASSIS_POWER_BUFF ?
                                                 CHASSIS_POWER_BUFF : Referee.PowerHeatData.buffer_energy;
        chassis.power_limit.limit_k = chassis.power_limit.chassis_power_buffer / CHASSIS_POWER_BUFF;

        if (chassis.power_limit.chassis_power_buffer < 100) {
            chassis.power_limit.limit_k =
                    chassis.power_limit.limit_k * chassis.power_limit.limit_k;
        } else {
            chassis.power_limit.limit_k =
                    chassis.power_limit.limit_k;
        }

        chassis.power_limit.total_current_limit =
                chassis.power_limit.limit_k * CHASSIS_CURRENT_LIMIT_TOTAL;
    }
    for (int i = 0; i < 4; i++) {
        chassis.power_limit.total_current += abs(chassis.drive_motor_chassis[i].give_current);

        give_current_limit = chassis.drive_motor_chassis[i].give_current;

        if (chassis.power_limit.total_current > chassis.power_limit.total_current_limit) {
            give_current_limit = (int16_t) ((fp32)chassis.drive_motor_chassis[i].give_current *
                                            chassis.power_limit.total_current_limit
                                            / chassis.power_limit.total_current);
        }
        //计算电流
        chassis.drive_motor_chassis[i].give_current = give_current_limit;
    }
}