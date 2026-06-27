//
// Created by xhuanc on 2022/3/10.
//

#ifndef DEMO1_DETECTION_H
#define DEMO1_DETECTION_H
#include "struct_typedef.h"
#include <stdbool.h>
#include "cmsis_os.h"
#include "bsp_buzzer.h"
#include "bsp_led.h"

#define ONLINE 1
#define OFFLINE 0

typedef struct {
//    uint8_t enable;
    uint32_t last_online_time;
//    uint32_t current_time;
    uint8_t status;
    uint32_t offline_threshold;
    uint8_t warning_level;
//    void (*offline_handle)(void*);
}__packed detect_device_t;

typedef enum {
    DETECT_CHASSIS_3508_RF_DRIVE=0,
    DETECT_CHASSIS_3508_LF_DRIVE,
    DETECT_CHASSIS_3508_LB_DRIVE,
    DETECT_CHASSIS_3508_RB_DRIVE,

    DETECT_CHASSIS_6020_RF_STEERING,
    DETECT_CHASSIS_6020_LF_STEERING,
    DETECT_CHASSIS_6020_LB_STEERING,
    DETECT_CHASSIS_6020_RB_STEERING,

    DETECT_GIMBAL_UP_PITCH,
    DETECT_GIMBAL_DOWN_PITCH,
    DETECT_GIMBAL_YAW,

    //摩擦轮
    DETECT_LAUNCHER_3508_FIRE_L,
    DETECT_LAUNCHER_3508_FIRE_R,

    //拨盘电机
    DETECT_LAUNCHER_3508_TRIGGER,

    DETECT_REMOTE,      //遥控器
    DETECT_AUTO_AIM,    //自动瞄准
    DETECT_REFEREE,     //裁判
    DETECT_CAP,
    DETECT_VIDEO_TRANSIMITTER,  //图传

    DETECT_DEVICE_LIST_LEN, //设备列表LEN
}detect_device_index;

typedef enum
{
    ALL_ONLINE = 1,
    RC_ONLINE,
    VT_ONLINE,
    ALL_OFFLINE,
}control_status;

extern detect_device_t detect_list[DETECT_DEVICE_LIST_LEN];
extern void detect_handle(uint8_t index);

#endif //DEMO1_DETECTION_H
