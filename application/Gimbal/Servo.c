//
// Created by laiyo on 25-5-3.
//

#include "Servo.h"

#include "bsp_servo_pwm.h"
#include "key_board.h"

uint16_t pwm_set;

void Servo_Init() {
    pwm_set = 2000;
    servo_pwm_set(pwm_set,1);
}

void Servo_Control(){
    if(KeyBoard.CTRL.click_flag == KEY_DOWN) {
        if(pwm_set == 2000) {
            pwm_set = 1500;
            servo_pwm_set(pwm_set,1);
        }
        if(KeyBoard.Z.status == KEY_PRESS) {
            pwm_set += 2;
            if(pwm_set > 1900) {
                pwm_set = 1900;
            }
            servo_pwm_set(pwm_set,1);
        }
        if(KeyBoard.X.status == KEY_PRESS) {
            pwm_set -= 2;
            if(pwm_set < 850) {
                pwm_set = 850;
            }
            servo_pwm_set(pwm_set,1);
        }
    }else {
        pwm_set = 2000;
        servo_pwm_set(pwm_set,1);
    }
}
