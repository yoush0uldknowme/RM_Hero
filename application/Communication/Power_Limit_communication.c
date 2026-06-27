//
// Created by SkLin on 26-1-28.
//

#include "Power_Limit_communication.h"
#include <stdbool.h>
#include <string.h>
extern UART_HandleTypeDef huart1;
// 全局变量定义
uint8_t usart1_Power[RX_BUFFER_SIZE] = {0};
rx_PowerLimit_t rx_PowerLimit = {0};
volatile bool_t rx_power_valid = false;  // 初始无效
uint8_t test122 = sizeof(rx_PowerLimit);

#ifdef CHASSIS
void USART1_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))
    {
        // 清除 IDLE 标志
        volatile uint32_t tmp = huart1.Instance->SR;
        tmp = huart1.Instance->DR;
        (void)tmp;

        // 获取已接收数据长度
        uint32_t dma_counter = __HAL_DMA_GET_COUNTER(huart1.hdmarx);
        uint16_t received_len = (uint16_t)(RX_BUFFER_SIZE - dma_counter);

        if (received_len > 0 && received_len <= RX_BUFFER_SIZE) {
            PowerLimit_read_data(usart1_Power, received_len);
        }

        HAL_UART_DMAStop(&huart1);
        HAL_UART_Receive_DMA(&huart1, usart1_Power, RX_BUFFER_SIZE);
    }

    // 如果使用了其他中断（如错误中断），也要处理
    // 否则 ORE 等错误会锁死 UART
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
}
#endif

/**
 * @brief 解析缓冲区中的所有有效 PowerLimit 帧，并更新全局变量为最新帧
 * @param buffer: 接收缓冲区
 * @param len: 数据长度
 * @return true: 至少找到一帧；false: 无有效帧
 */
bool_t PowerLimit_read_data(uint8_t *buffer, uint16_t len)
{
    if (!buffer || len < RX_FRAME_TOTAL_SIZE) return false;

    bool_t found = false;
    uint16_t i = 0;

    while (i <= len - RX_FRAME_TOTAL_SIZE) {
        if (buffer[i] == RX_FRAME_START_BYTE &&
            buffer[i + RX_FRAME_TOTAL_SIZE - 1] == RX_FRAME_END_BYTE) {

            // 找到一帧，直接覆盖全局变量（最新帧优先）
            memcpy(&rx_PowerLimit, &buffer[i], sizeof(rx_PowerLimit_t));
            rx_power_valid = true;  // 标记数据有效
            found = true;

            // 跳过这一帧，继续查找下一帧（支持连续多帧）
            i += RX_FRAME_TOTAL_SIZE;
            } else {
                i++; // 未匹配，逐字节滑动
            }
    }

    return found;
}