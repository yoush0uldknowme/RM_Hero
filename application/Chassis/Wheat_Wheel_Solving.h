//
// Created by Gularx on 2025/2/26.
//

#ifndef INC_2025_HERO_THREE_WHEAT_WHEEL_SOLVING_H
#define INC_2025_HERO_THREE_WHEAT_WHEEL_SOLVING_H
#include "Hero.h"

typedef enum {
    RF=0,
    LF=1,
    LB=2,
    RB=3,
}MOTOR_CHASSIS;


extern void Chassis_Meknum_Wheel_Cal(void);
extern void Chassis_Wheel_Loop_Cal(void);

#endif //INC_2025_HERO_THREE_WHEAT_WHEEL_SOLVING_H
