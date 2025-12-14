1. 将之前写在内部变量里面的
first_order_filter_type_t vx_slow={.frame_period=1, .num=8};
first_order_filter_type_t vy_slow={.frame_period=1, .num=15};
first_order_filter_type_t vw_slow={.frame_period=4, .num=40};
移动到 Hero.h 中的 chassis_t 结构体里面的
first_order_filter_type_t vx_slow;
first_order_filter_type_t vy_slow;
first_order_filter_type_t vw_slow;
并在 chassis_init 中对其用
first_order_filter_init(&chassis->vx_slow, 1, 8);
first_order_filter_init(&chassis->vy_slow, 1, 15);
first_order_filter_init(&chassis->vw_slow, 1, 40);
进行初始化。另外，下面的代码和上面的应该是同一个，只是用到了不同的函数，因此没有用到现在的代码，已经注释掉
ramp_init(&chassis->chassis_vx_ramp, 0.0001f, MAX_CHASSIS_VX_SPEED, -MAX_CHASSIS_VX_SPEED);
ramp_init(&chassis->chassis_vy_ramp, 0.0001f, MAX_CHASSIS_VY_SPEED, -MAX_CHASSIS_VY_SPEED);
ramp_init(&chassis->chassis_vw_ramp, 0.0001f, MAX_CHASSIS_VW_SPEED, -MAX_CHASSIS_VW_SPEED);

2. 在 chassis_init 中
ramp_init(&chassis->chassis_3508_ramp[LF], 0.0001f, M3508_MAX_RPM, -M3508_MAX_RPM);
ramp_init(&chassis->chassis_3508_ramp[RF], 0.0001f, M3508_MAX_RPM, -M3508_MAX_RPM);
ramp_init(&chassis->chassis_3508_ramp[RB], 0.0001f, M3508_MAX_RPM, -M3508_MAX_RPM);
ramp_init(&chassis->chassis_3508_ramp[LB], 0.0001f, M3508_MAX_RPM, -M3508_MAX_RPM);
是一个斜坡函数，用于减缓上升速率，在最开始没有用到，后面需要测试需不需要添加，已经添加在 chassis_wheel_loop_cal 中的
ramp_calc(&chassis.chassis_3508_ramp[LF],chassis.motor_chassis[RF].rpm_set);
ramp_calc(&chassis.chassis_3508_ramp[RF],chassis.motor_chassis[RF].rpm_set);
ramp_calc(&chassis.chassis_3508_ramp[RB],chassis.motor_chassis[RF].rpm_set);
ramp_calc(&chassis.chassis_3508_ramp[LB],chassis.motor_chassis[RF].rpm_set);
先注释掉，测试完反馈一下哪个效果好

3. 另外，对 chassis_t 里面的具体参数作用进行介绍