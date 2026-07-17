#ifndef VTX_FORWARD_H
#define VTX_FORWARD_H

#include <stdint.h>

#define VTX_0310_DATA_LEN   300U
#define VTX_0310_FRAME_LEN  309U

typedef struct
{
    volatile uint32_t accepted;
    volatile uint32_t invalid;
    volatile uint32_t tx_started;
    volatile uint32_t queued;
    volatile uint32_t replaced;
} vtx_forward_stats_t;

uint8_t VTX_Forward_Submit(const uint8_t *frame, uint16_t len);
void VTX_Forward_Process(void);

extern vtx_forward_stats_t vtx_forward_stats;

#endif // VTX_FORWARD_H
