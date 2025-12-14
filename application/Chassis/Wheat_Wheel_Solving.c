//
// Created by Gularx on 2025/2/26.
//
#include "Wheat_Wheel_Solving.h"

#include "Atti.h"
#include "Chassis.h"

/**
  * @brief          麦轮解算
  * @param[in]      pvParameters
  * 计算麦轮的速度
  * @retval         none
  */

void Chassis_Meknum_Wheel_Cal(void) {
//TODO: 定义的wheel_rpm类型更改为浮点数则会出现小陀螺状态下无法正常移动的问题
    int16_t wheel_rpm[4];
    fp32 max = 0;

    // 根据给定的底盘运动参数，计算四个轮子的转速
    wheel_rpm[0] = ( chassis.vy - chassis.vx + chassis.vw * rotate_ratio_f) * wheel_rpm_ratio;
    wheel_rpm[1] = ( chassis.vy + chassis.vx + chassis.vw * rotate_ratio_f) * wheel_rpm_ratio;
    wheel_rpm[2] = (-chassis.vy - chassis.vx + chassis.vw * rotate_ratio_b) * wheel_rpm_ratio;
    wheel_rpm[3] = (-chassis.vy + chassis.vx + chassis.vw * rotate_ratio_b) * wheel_rpm_ratio;


    for (uint8_t i = 0; i < 4; i++) {
        if (ABS(wheel_rpm[i]) > max) max = ABS(wheel_rpm[i]);
    }

    // 如果超过最大转速，对所有轮子转速进行缩放，使其不超过最大转速
    if (max > M3508_MAX_RPM) {
        float rate = M3508_MAX_RPM / max;
        for (uint8_t i = 0; i < 4; i++) wheel_rpm[i] *= rate;
    }

    //上坡功率重新分配

    chassis.board_pitch=INS_angle[2]*MOTOR_RAD_TO_ANGLE;
    if(abs(chassis.board_pitch) > 8)
    {
        fp32 ramp_p = 0; //上坡功率分配系数p
        ramp_p = 0.6f * sinf(-INS_angle[2]);
        VAL_LIMIT(ramp_p,0,0.25f);
        wheel_rpm[0] *= (1 - 0.7f * sinf(-INS_angle[2]));
        wheel_rpm[1] *= (1 - 0.7f * sinf(-INS_angle[2]));
        wheel_rpm[2] *= (1 + ramp_p);
        wheel_rpm[3] *= (1 + ramp_p);
    }
    // 设置给定的转速
    chassis.motor_chassis[RF].rpm_set=wheel_rpm[0];
    chassis.motor_chassis[LF].rpm_set=wheel_rpm[1];
    chassis.motor_chassis[RB].rpm_set=wheel_rpm[2];
    chassis.motor_chassis[LB].rpm_set=wheel_rpm[3];
}


/**
  * @brief          麦轮电流给定
  * @param[in]      pvParameters
  * pid调整给定的麦轮电流
  * @retval         none
  */
void Chassis_Wheel_Loop_Cal(void) {
    ramp_calc(&chassis.chassis_ramp[RF],chassis.motor_chassis[RF].rpm_set);
    chassis.motor_chassis[RF].give_current= (int16_t)pid_calc(&chassis.motor_chassis[RF].speed_p,
                                                              chassis.motor_chassis[RF].motor_measure.speed_rpm,
                                                              chassis.motor_chassis[RF].rpm_set);

    ramp_calc(&chassis.chassis_ramp[LF],chassis.motor_chassis[LF].rpm_set);
    chassis.motor_chassis[LF].give_current= (int16_t)pid_calc(&chassis.motor_chassis[LF].speed_p,
                                                              chassis.motor_chassis[LF].motor_measure.speed_rpm,
                                                              chassis.motor_chassis[LF].rpm_set);

    ramp_calc(&chassis.chassis_ramp[LB],chassis.motor_chassis[LB].rpm_set);
    chassis.motor_chassis[LB].give_current= (int16_t)pid_calc(&chassis.motor_chassis[LB].speed_p,
                                                              chassis.motor_chassis[LB].motor_measure.speed_rpm,
                                                              chassis.motor_chassis[LB].rpm_set);

    ramp_calc(&chassis.chassis_ramp[RB],chassis.motor_chassis[RB].rpm_set);
    chassis.motor_chassis[RB].give_current= (int16_t)pid_calc(&chassis.motor_chassis[RB].speed_p,
                                                              chassis.motor_chassis[RB].motor_measure.speed_rpm,
                                                              chassis.motor_chassis[RB].rpm_set);

}
