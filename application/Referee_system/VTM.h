//
// Created by SyLin on 26-3-5.
// Virtual Target Mark (VTM) - 虚拟目标标记
// 

#ifndef VTM_H
#define VTM_H

#include "struct_typedef.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 键盘按键位定义（bit0~bit15）
 * bit0:W bit1:S bit2:A bit3:D bit4:Shift bit5:Ctrl bit6:Q bit7:E
 * bit8:R bit9:F bit10:G bit11:Z bit12:X bit13:C bit14:V bit15:B
 */
typedef union
{
    struct
    {
        uint16_t w     : 1;
        uint16_t s     : 1;
        uint16_t a     : 1;
        uint16_t d     : 1;
        uint16_t shift : 1;
        uint16_t ctrl  : 1;
        uint16_t q     : 1;
        uint16_t e     : 1;
        uint16_t r     : 1;
        uint16_t f     : 1;
        uint16_t g     : 1;
        uint16_t z     : 1;
        uint16_t x     : 1;
        uint16_t c     : 1;
        uint16_t v     : 1;
        uint16_t b     : 1;
    } bit;
    uint16_t value;
} __attribute__((packed)) vtm_key_t;

/**
 * @brief VTM数据帧结构体定义
 * 包含虚拟目标数据的完整帧，支持多通道控制数据和CRC16校验
 */
typedef struct
{
    // 帧头 (SOF - Start of Frame)
    uint8_t sof_1;                      // 起始符1 (0-7 bit)
    uint8_t sof_2;                      // 起始符2 (8-15 bit)
    
    // 控制通道数据 (16-59 bit)
    uint64_t ch_0:11;                   // 通道0 (11位) - 0x0A09范围 : 右摇杆水平方向
    uint64_t ch_1:11;                   // 通道1 (11位) - 0x0A53范围 : 右摇杆竖直方向
    uint64_t ch_2:11;                   // 通道2 (11位) - 0x364范围 : 左摇杆竖直方向
    uint64_t ch_3:11;                   // 通道3 (11位) - 0x1024/0x0A53范围 : 左摇杆水平方向

    // 模式和功能控制 (60-66 bit)
    uint64_t mode_sw:2;                 // 模式选择 (2位) - 0x1684范围 : 接收端档位切换开关位置: C:0 N:1 S:2
    uint64_t pause:1;                   // 暂停标志 (1位) : 未按下:0 按下:1
    uint64_t fn_left:1;                    // 功能键1 (1位) : 未按下:0 按下:1
    uint64_t fn_right:1;                    // 功能键2 (1位) : 未按下:0 按下:1

    // 轮值/转动数据 (67-77 bit)
    uint64_t wheel:11;                  // 鼠标滚轮或转轴 (11位) : 364-1024-1684
    
    // 触发相关 (78 bit)
    uint64_t trigger:1;                 // 触发信号 (1位)
    
    // 鼠标坐标数据 (79-110 bit)
    int16_t mouse_x;                    // 鼠标X轴坐标 (16位) - 0±32768范围可选
    int16_t mouse_y;                    // 鼠标Y轴坐标 (16位) - 0±32768范围
    int16_t mouse_z;                    // 鼠标Z轴坐标 (16位) - 0±32768范围
    
    // 鼠标按键 (111-115 bit)
    uint8_t mouse_left:2;               // 鼠标左键 (2位) - 0/1/2范围
    uint8_t mouse_right:2;              // 鼠标右键 (2位) - 0/1/2范围
    uint8_t mouse_middle:2;             // 鼠标中键 (2位)
    
    // 键盘数据 (118-133 bit)
    vtm_key_t key;                      // 键盘按键信息 (16位)
    
    // CRC校验 (152 bit)
    uint16_t crc16;                     // CRC16校验码 (16位)
} __attribute__((packed)) vtm_data_t;

/**
 * @brief VTM数据帧常量定义
 */
#define VTM_SOF_1                   0xA9 // 起始符1
#define VTM_SOF_2                   0x53 // 起始符2
#define VTM_FRAME_SIZE              sizeof(vtm_data_t)
#define VTM_DATA_SIZE               (VTM_FRAME_SIZE - 2) // 不包括CRC16的数据大小
#define VTM_CRC16_SIZE              2

/**
 * @brief 初始化VTM数据结构体
 * @param p_vtm_data VTM数据结构体指针
 */
void vtm_data_init(vtm_data_t *p_vtm_data);

/**
 * @brief 计算CRC16校验和
 * @param p_msg 数据缓冲区指针
 * @param len 数据长度
 * @param crc16 初始CRC16值（通常为0xffff）
 * @return 计算出的CRC16校验和
 */
uint16_t vtm_get_crc16_check_sum(uint8_t *p_msg, uint16_t len, uint16_t crc16);

/**
 * @brief 验证CRC16校验和
 * @param p_msg 数据缓冲区指针
 * @param len 数据长度（包括CRC16）
 * @return 校验结果，true表示校验通过，false表示校验失败
 */
bool vtm_verify_crc16_check_sum(uint8_t *p_msg, uint16_t len);

/**
 * @brief 计算VTM数据帧的CRC16
 * @param p_vtm_data VTM数据结构体指针
 * @return 计算出的CRC16校验和
 */
uint16_t vtm_frame_crc16_calculate(vtm_data_t *p_vtm_data);

/**
 * @brief 验证VTM数据帧
 * @param p_vtm_data VTM数据结构体指针
 * @return 校验结果，true表示帧有效，false表示帧无效
 */
bool vtm_frame_verify(vtm_data_t *p_vtm_data);

/**
 * @brief 打包VTM数据为字节流
 * @param p_vtm_data VTM数据结构体指针
 * @param p_buf 输出缓冲区
 * @param buf_len 缓冲区大小
 * @return 打包后的字节数，失败返回0
 */
uint16_t vtm_pack_data(vtm_data_t *p_vtm_data, uint8_t *p_buf, uint16_t buf_len);

/**
 * @brief 解包字节流为VTM数据
 * @param p_buf 输入缓冲区
 * @param len 缓冲区大小
 * @param p_vtm_data VTM数据结构体指针
 * @return 解包结果，true表示成功，false表示失败
 */
bool vtm_unpack_data(uint8_t *p_buf, uint16_t len, vtm_data_t *p_vtm_data);

/**
 * @brief 从接收缓冲区中解析VTM帧（支持粘包）
 * @param buffer 接收缓冲区
 * @param len 实际接收长度
 * @return true: 至少解析到一帧有效数据; false: 未解析到有效帧
 */
bool_t vtm_read_data(uint8_t *buffer, uint16_t len);

/**
 * @brief 将已解包的VTM数据同步到rc_ctrl和KeyBoard（需在主任务调用）
 */
void vtm_sync_to_control(void);

extern vtm_data_t vtm_rx_data;
extern volatile bool_t vtm_rx_valid;

#endif // VTM_H
