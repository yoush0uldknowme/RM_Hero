//
// Created by SR on 2024/12/13.
//
#include "power_limit.h"
#include "Chassis.h"
#include "Cap.h"

//计算函数f(iset)=K*Ωset+M的系数K，M
void calc_power_limit(pid_type_def *pid,int i)
{
    //计算积分累计
    pid->sum_err += (pid->err[NOW]+pid->err[LAST])/2;
    //根据pid计算函数推出的
    chassis.chassis_power_limit.K[i]=pid->p*CHASSIS_CURRENT_CONVERT;
    chassis.chassis_power_limit.M[i]=-(pid->p*chassis.motor_chassis[i].motor_measure.speed_rpm-pid->iout)*CHASSIS_CURRENT_CONVERT;
    pid->sum_err = fp32_constrain(pid->sum_err,-CHASSIS_3508_PID_MAX_IOUT,-CHASSIS_3508_PID_MAX_IOUT);
}


/* 功率控制 */
void chassis_power_limit(fp32 buffer_limit, fp32 k) {
    chassis.chassis_power_limit.power_set=Referee.GameRobotStat.chassis_power_limit-5;
    chassis.chassis_power_limit.k_c=1;
    //更新电容状态
//    cap_info_update();
    for (int i = 0; i < 4; ++i) {
        //计算K,M
        calc_power_limit(&chassis.motor_chassis[i].speed_p,i);
    }
    float a=0,b=0,c=0;
    for (int i = 0; i < 4; ++i) {
        //a,b,c的计算过程，具体参看华中科技大学功率控制模型
        a += CHASSIS_POWER_R0 * pow(chassis.motor_chassis[i].rpm_set,2) * pow(chassis.chassis_power_limit.K[i],2);
        b += chassis.motor_chassis[i].rpm_set * (2 * CHASSIS_POWER_R0 * chassis.chassis_power_limit.K[i] * chassis.chassis_power_limit.M[i] + CHASSIS_POWER_K0 * chassis.chassis_power_limit.K[i] * chassis.motor_chassis[i].motor_measure.speed_rpm);
        c += CHASSIS_POWER_R0 * pow(chassis.chassis_power_limit.M[i],2) + CHASSIS_POWER_K0 * chassis.chassis_power_limit.M[i] * chassis.motor_chassis[i].motor_measure.speed_rpm;

    }
    c += CHASSIS_POWER_P0;

    //算出下一周期的功率
    chassis.chassis_power_limit.predict_send_power = fp32_constrain(a+b+c,-1000,1000);
    c-=chassis.chassis_power_limit.power_set;
    bool_t dangerous=false;//缓冲能量过低触发危险信号
    //缓冲能量小于10J，危险信号开启

    if(Referee.PowerHeatData.buffer_energy<buffer_limit){
        // dangerous=true;
        //系数c越大，限制的功率值越低，比如最大功率80w，限制的功率降至60w，那底盘功率被限制后最大仅可达到60w上下，几乎不会超过80w
        c+=(buffer_limit-Referee.PowerHeatData.buffer_energy)*1.0f;
    }

    //当预测功率大于最大功率时或者缓冲能量过低时，启动功率限制，求出刚好令预测功率<=最大功率的系数K_c
    if(chassis.chassis_power_limit.predict_send_power > chassis.chassis_power_limit.power_set||dangerous)
    {
//
        if(b*b < 4*c*a)
        {
            chassis.chassis_power_limit.k_c = fp32_constrain(-b/(2*a),0.0f,1.0f);
        }
        else
        {
            float sqrt_result;
            arm_sqrt_f32(b*b - 4*c*a ,&sqrt_result);
            chassis.chassis_power_limit.k_c = fp32_constrain((-b + sqrt_result)/2/a,0.0f,1.0f);
        }
        //当开电容并且缓冲能量充足时，函数直接返回，不执行后续的功率控制
        //电容返回的数据具体参考电容手册
//        if(!dangerous)//cap2.mode==1&&
//        {
//            return;
//        }
//        if(chassis.vw>0.15||chassis.vw<-0.15)
//        {
//            k-=0.02f;
//        }
        chassis.chassis_power_limit.k_c += (1 - chassis.chassis_power_limit.k_c) * k;
        for (int i = 0; i < 4; ++i) {
            //对rpm_set进行比例缩减，达到功率控制效果
            chassis.motor_chassis[i].rpm_set *= chassis.chassis_power_limit.k_c;
        }

    }
}
