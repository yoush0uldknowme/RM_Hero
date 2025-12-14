//
// Created by zuishenglibai on 25-9-11.
//

#ifndef STEERING_WHEEL_SOLVING_H
#define STEERING_WHEEL_SOLVING_H

#define RF_DIRECTION -1
#define LF_DIRECTION 1
#define LB_DIRECTION 1
#define RB_DIRECTION -1

#define RF_STEERING_OFFSET_ECD 1978
#define LF_STEERING_OFFSET_ECD 6122
#define LB_STEERING_OFFSET_ECD 2043
#define RB_STEERING_OFFSET_ECD 2701

// #define PI          3.14159265358979323846f
#define RAD_TO_DEG  (180.0f / PI)
#define DEG_TO_RAD  (PI / 180.0f)

typedef enum {
    RF=0,
    LF=1,
    LB=2,
    RB=3,
}MOTOR_CHASSIS;

void Chassis_Steering_Wheel_Init(void);
void Chassis_Steering_Wheel_Cal(void);
void Chassis_Wheel_Loop_Cal(void);

#endif //STEERING_WHEEL_SOLVING_H
