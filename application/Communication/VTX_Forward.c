#include "VTX_Forward.h"

#include <string.h>

#include "CRC8_CRC16.h"
#include "bsp_usart.h"
#include "usart.h"

static uint8_t vtx_tx_buffer[VTX_0310_FRAME_LEN];
static uint8_t vtx_pending_buffer[VTX_0310_FRAME_LEN];
static uint8_t vtx_pending_valid = 0U;

vtx_forward_stats_t vtx_forward_stats = {0};

static uint8_t vtx_uart6_tx_busy(void)
{
    return ((huart6.hdmatx->Instance->CR & DMA_SxCR_EN) != 0U) ? 1U : 0U;
}

static uint8_t vtx_0310_frame_valid(const uint8_t *frame, uint16_t len)
{
    uint16_t data_len;
    uint16_t cmd_id;

    if ((frame == NULL) || (len != VTX_0310_FRAME_LEN))
    {
        return 0U;
    }

    data_len = (uint16_t)frame[1] | ((uint16_t)frame[2] << 8U);
    cmd_id = (uint16_t)frame[5] | ((uint16_t)frame[6] << 8U);

    if ((frame[0] != 0xA5U) ||
        (data_len != VTX_0310_DATA_LEN) ||
        (cmd_id != 0x0310U))
    {
        return 0U;
    }

    if (verify_CRC8_check_sum((uint8_t *)frame, 5U) == 0U)
    {
        return 0U;
    }

    if (verify_CRC16_check_sum((uint8_t *)frame, VTX_0310_FRAME_LEN) == 0U)
    {
        return 0U;
    }

    return 1U;
}

static void vtx_start_tx(const uint8_t *frame)
{
    memcpy(vtx_tx_buffer, frame, VTX_0310_FRAME_LEN);
    usart6_tx_dma_enable(vtx_tx_buffer, VTX_0310_FRAME_LEN);
    vtx_forward_stats.tx_started++;
}

uint8_t VTX_Forward_Submit(const uint8_t *frame, uint16_t len)
{
    if (vtx_0310_frame_valid(frame, len) == 0U)
    {
        vtx_forward_stats.invalid++;
        return 0U;
    }

    vtx_forward_stats.accepted++;

    if (vtx_uart6_tx_busy() == 0U)
    {
        vtx_start_tx(frame);
        return 1U;
    }

    if (vtx_pending_valid != 0U)
    {
        vtx_forward_stats.replaced++;
    }

    memcpy(vtx_pending_buffer, frame, VTX_0310_FRAME_LEN);
    vtx_pending_valid = 1U;
    vtx_forward_stats.queued++;
    return 1U;
}

void VTX_Forward_Process(void)
{
    if ((vtx_pending_valid != 0U) && (vtx_uart6_tx_busy() == 0U))
    {
        vtx_start_tx(vtx_pending_buffer);
        vtx_pending_valid = 0U;
    }
}
