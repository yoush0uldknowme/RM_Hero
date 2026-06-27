//
// Created by Gularx on 2024/2/26.
//
#include "ramp.h"

#ifndef INC_2024_HERO_HERO_H
#define INC_2024_HERO_HERO_H

// #define GIMBAL
#define CHASSIS

/**** task list ****/

// TODO
// TODO

/**** mode selection ****/
typedef enum {
    GIMBAL_RELAX,
    // GIMBAL_ACTIVE,
    GIMBAL_UP,
    GIMBAL_DOWN,
    GIMBAL_UP_DISABLE,
    GIMBAL_CRAZY_CYCLE,
    GIMBAL_AUTO,
    GIMBAL_FIRE,
} Gimbal_Mode_e;

#define GIMBAL_MIDDLE GIMBAL_CRAZY_CYCLE
#define GIMBAL_UP_Disable GIMBAL_UP_DISABLE
#define GIMBAL_UP_DIASABLE GIMBAL_UP_DISABLE

typedef enum {
    Fire_OFF = 0,
    Fire_ON = 1,
} Fire_Mode_e;

typedef enum {
    SHOOT_CLOSE = 0,
    SHOOT_READLY,
    SHOOT_SINGLE,
    SHOOT_OVER,
    SHOOT_BLOCK,
    SHOOT_BLOCK_BACK,
    SHOOT_FAIL,
    SHOOT_OFF
} Shoot_Cmd_e;

typedef enum {
    CHASSIS_RELAX,
    CHASSIS_ONLY,
    CHASSIS_FIRE,
    CHASSIS_FOLLOW_GIMBAL,
    CHASSIS_SPIN_R,
    CHASSIS_SPIN_L,
} Chassis_Mode_e;

#endif // INC_2024_HERO_HERO_H
