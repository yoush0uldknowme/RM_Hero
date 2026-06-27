//
// Created by Gularx on 26-2-26.
//

/*********************************************************************************************************
*                                              包含头文件
*********************************************************************************************************/
#include "Cap.h"

#include <string.h>

#include "can.h"
#include "cmsis_os.h"
#include "task.h"
#include "Detection.h"
#include "Referee.h"

/*********************************************************************************************************
*                                              内部变量
*********************************************************************************************************/
Supercap_FeedbackFrame_t feedback_frame;
float Referee_power_limit = 0;
static uint32_t error_count = 0;
uint8_t cap_level = 0;
HAL_StatusTypeDef ret;

/*********************************************************************************************************
*                                              内部函数声明
*********************************************************************************************************/
_Noreturn void Cap_task(void const *pvParameters);
static HAL_StatusTypeDef Cap_Can_Control(float power_limit, uint8_t state);
static void Cap_init();

/*********************************************************************************************************
*                                              内部函数实现
*********************************************************************************************************/
void Cap_task(void const *pvParameters)
{
    vTaskDelay(CAP_TASK_INIT_TIME);
    Cap_init();


    while (1)
    {
        /* 发送控制帧 */
        ret = Cap_Can_Control(Referee.GameRobotStat.chassis_power_limit-5, ENABLE);
        // ret = Cap_Can_Control(Referee.GameRobotStat.chassis_power_limit, DISABLE);

        // 处理发送结果
        if (ret != HAL_OK)
        {
            // 可以记录错误计数
            error_count++;
        }
        osDelay(1000);
    }
}


static void Cap_init() {
    feedback_frame.power_set = 0;
    feedback_frame.current_power = 0;
    feedback_frame.voltage = -0;
}


/**
 * @brief 发送电容控制帧
 * @param power_limit 功率限制值 (W)
 * @param state 状态 ENABLE/DISABLE
 * @return HAL_OK: 成功, HAL_ERROR: 失败, HAL_TIMEOUT: 超时
 */
uint32_t test = 0;
static HAL_StatusTypeDef Cap_Can_Control(float power_limit, uint8_t state)
{
    Supercap_ControlFrame_t control_frame;
    CAN_TxHeaderTypeDef tx_message;
    tx_message.StdId = CAP_CONTROL;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;
    uint32_t tx_mailbox;
    uint32_t start_tick;
    const uint32_t timeout_ms = 10;

    // 填充数据...
    control_frame.bits.enable = (state == ENABLE) ? 0x01 : 0x00;
    control_frame.bits.power_set = power_limit;
    // 等待空闲邮箱
    start_tick = HAL_GetTick();
    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0) {
        if ((HAL_GetTick() - start_tick) > timeout_ms) {
            return HAL_TIMEOUT;
        }
        osDelay(1);  // 让出CPU
    }

    // 发送数据
    return HAL_CAN_AddTxMessage(&hcan2, &tx_message, control_frame.data, &tx_mailbox);
}


/*********************************************************************************************************
*                                              API函数实现
*********************************************************************************************************/
void Cap_Data_Get(uint32_t can_id, const uint8_t *rx_data)
{
    /* 收到的can信息 */
    if (can_id == CAP_FEEDBACK) {
        feedback_frame.power_set = (rx_data[0] << 8) | rx_data[1];
        feedback_frame.current_power = (rx_data[2] << 8) | rx_data[3];
        feedback_frame.voltage = (rx_data[4] << 8) | rx_data[5];
        detect_handle(DETECT_CAP);
    }
}



fp32 cap_Get_Boosting_Power(fp32 _upper, fp32 _mid, fp32 _lower) {
    fp32 boost_Power = 0.0f;
    // 计算剩余电压
    fp32 Volt = (fp32)feedback_frame.voltage * 1.0f / 100.0f;
    // 倘若开启boost之后电容剩余电压下降到给定值，需要时间充能，充能期间功率提升为0
    if (Volt >= _mid){
        boost_Power = Volt * 15.0f;
        cap_level = HIGH;
    }
    else if (Volt < _mid && Volt >= _lower) {
        boost_Power = 5.0f + 10.0f*(Volt-5.0f)/5.0f;
        cap_level = MID;
    }
    else {
        boost_Power = 0;
    }

    return boost_Power;
}