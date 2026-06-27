//
// Created by SyLin on 25-9-11.
//

#ifndef STEERING_WHEEL_SOLVING_H
#define STEERING_WHEEL_SOLVING_H

#define RF_DIRECTION -1
#define LF_DIRECTION 1
#define LB_DIRECTION 1
#define RB_DIRECTION -1

#define RF_STEERING_OFFSET_ECD 3210
#define LF_STEERING_OFFSET_ECD 4528
#define LB_STEERING_OFFSET_ECD 2436
#define RB_STEERING_OFFSET_ECD 1185

// #define PI          3.14159265358979323846f
#define RAD_TO_DEG  (180.0f / PI)
#define DEG_TO_RAD  (PI / 180.0f)

#define STEERING_REVERSE_HYSTERESIS_DEG 5.0f



extern void Chassis_Steering_Wheel_Init(void);
extern void Chassis_Steering_Wheel_Cal(void);
extern void Chassis_Wheel_Loop_Cal(void);

#endif //STEERING_WHEEL_SOLVING_H
