//
// Created by SyLin on 26-3-5.
//

#include "VTM.h"
#include <string.h>
#include "remote.h"
#include "key_board.h"

vtm_data_t vtm_rx_data = {0};
volatile bool_t vtm_rx_valid = false;

static void vtm_set_key_state(Key *key, bool_t pressed)
{
    if (key == NULL) return;
    key->last_status = key->status;
    key->status = pressed ? KEY_DOWN : KEY_RELAX;
}

static uint8_t mode = 0;
static void vtm_apply_to_control(const vtm_data_t *data)
{
    if (data == NULL) return;
    static const uint8_t ws_to_rc_map[3] = {2, 3, 1};

    rc_ctrl.rc.ch[0] = (int16_t)data->ch_0 - RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[1] = (int16_t)data->ch_1 - RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[3] = (int16_t)data->ch_2 - RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[2] = (int16_t)data->ch_3 - RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[AUTO_CHANNEL] = (int16_t)data->wheel - RC_CH_VALUE_OFFSET;

    rc_ctrl.rc.s[RC_s_R] = ws_to_rc_map[data->mode_sw];
    rc_ctrl.rc.s[RC_s_L] = 2;

    // b键失能，g键使能
    if (data->key.bit.b == 1) mode = 1;
    if (data->key.bit.g == 1) mode = 2;
    if (mode == 1)
    {
        rc_ctrl.rc.s[RC_s_L] = RC_SW_DOWN;
        rc_ctrl.rc.s[RC_s_R] = RC_SW_DOWN;
    }
    else if (mode == 2)
    {
        rc_ctrl.rc.s[RC_s_L] = RC_SW_MID;
        rc_ctrl.rc.s[RC_s_R] = RC_SW_MID;
    }

    Referee.keyboard.mouse_x = data->mouse_x;
    Referee.keyboard.mouse_y = data->mouse_y;
    Referee.keyboard.mouse_z = data->mouse_z;

    Referee.keyboard.keyboard_value = data->key.value;

    Referee.keyboard.left_button_down  = (data->mouse_left > 0) ? 1 : 0;
    Referee.keyboard.right_button_down = (data->mouse_right > 0) ? 1 : 0;
}

/**
 * @brief CRC16初始值
 */
static uint16_t crc16_init = 0xffff;

/**
 * @brief CRC16查表数组
 */
static const uint16_t crc16_tab[256] =
{
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

/**
 * @brief 初始化VTM数据结构体
 * @param p_vtm_data VTM数据结构体指针
 */
void vtm_data_init(vtm_data_t *p_vtm_data)
{
    if (p_vtm_data == NULL)
    {
        return;
    }
    
    memset(p_vtm_data, 0, sizeof(vtm_data_t));
    p_vtm_data->sof_1 = VTM_SOF_1;
    p_vtm_data->sof_2 = VTM_SOF_2;
}

/**
 * @brief 计算CRC16校验和
 * @param p_msg 数据缓冲区指针
 * @param len 数据长度
 * @param crc16 初始CRC16值（通常为0xffff）
 * @return 计算出的CRC16校验和
 */
uint16_t vtm_get_crc16_check_sum(uint8_t *p_msg, uint16_t len, uint16_t crc16)
{
    uint8_t data;

    if (p_msg == NULL)
    {
        return 0xffff;
    }

    while (len--)
    {
        data = *p_msg++;
        (crc16) = ((uint16_t)(crc16) >> 8) ^ crc16_tab[((uint16_t)(crc16) ^ (uint16_t)(data)) & 0x00ff];
    }

    return crc16;
}

/**
 * @brief 验证CRC16校验和
 * @param p_msg 数据缓冲区指针
 * @param len 数据长度（包括CRC16）
 * @return 校验结果，true表示校验通过，false表示校验失败
 */
bool vtm_verify_crc16_check_sum(uint8_t *p_msg, uint16_t len)
{
    uint16_t w_expected = 0;

    if ((p_msg == NULL) || (len <= 2))
    {
        return false;
    }
    
    w_expected = vtm_get_crc16_check_sum(p_msg, len - 2, crc16_init);

    return ((w_expected & 0xff) == p_msg[len - 2] && ((w_expected >> 8) & 0xff) == p_msg[len - 1]);
}

/**
 * @brief 计算VTM数据帧的CRC16
 * @param p_vtm_data VTM数据结构体指针
 * @return 计算出的CRC16校验和
 */
uint16_t vtm_frame_crc16_calculate(vtm_data_t *p_vtm_data)
{
    if (p_vtm_data == NULL)
    {
        return 0;
    }

    return vtm_get_crc16_check_sum((uint8_t *)p_vtm_data, VTM_DATA_SIZE, crc16_init);
}

/**
 * @brief 验证VTM数据帧
 * @param p_vtm_data VTM数据结构体指针
 * @return 校验结果，true表示帧有效，false表示帧无效
 */
bool vtm_frame_verify(vtm_data_t *p_vtm_data)
{
    if (p_vtm_data == NULL)
    {
        return false;
    }

    // 检查起始符
    if (p_vtm_data->sof_1 != VTM_SOF_1 || p_vtm_data->sof_2 != VTM_SOF_2)
    {
        return false;
    }

    // 检查CRC16
    return vtm_verify_crc16_check_sum((uint8_t *)p_vtm_data, VTM_FRAME_SIZE);
}

/**
 * @brief 打包VTM数据为字节流
 * @param p_vtm_data VTM数据结构体指针
 * @param p_buf 输出缓冲区
 * @param buf_len 缓冲区大小
 * @return 打包后的字节数，失败返回0
 */
uint16_t vtm_pack_data(vtm_data_t *p_vtm_data, uint8_t *p_buf, uint16_t buf_len)
{
    if (p_vtm_data == NULL || p_buf == NULL || buf_len < VTM_FRAME_SIZE)
    {
        return 0;
    }

    // 计算CRC16并填充
    p_vtm_data->crc16 = vtm_frame_crc16_calculate(p_vtm_data);

    // 复制整个数据帧到缓冲区
    memcpy(p_buf, (uint8_t *)p_vtm_data, VTM_FRAME_SIZE);

    return VTM_FRAME_SIZE;
}

/**
 * @brief 解包字节流为VTM数据
 * @param p_buf 输入缓冲区
 * @param len 缓冲区大小
 * @param p_vtm_data VTM数据结构体指针
 * @return 解包结果，true表示成功，false表示失败
 */
bool vtm_unpack_data(uint8_t *p_buf, uint16_t len, vtm_data_t *p_vtm_data)
{
    if (p_buf == NULL || p_vtm_data == NULL || len < VTM_FRAME_SIZE)
    {
        return false;
    }

    // 复制缓冲区数据到结构体
    memcpy((uint8_t *)p_vtm_data, p_buf, VTM_FRAME_SIZE);

    // 验证数据帧
    return vtm_frame_verify(p_vtm_data);
}

bool_t vtm_read_data(uint8_t *buffer, uint16_t len)
{
    if (buffer == NULL || len < VTM_FRAME_SIZE)
    {
        vtm_rx_valid = false;
        return false;
    }

    if (len > 500) len = 500;

    bool_t found = false;
    uint16_t i = 0;

    // 遍历缓冲区寻找帧头
    while (i <= (len - VTM_FRAME_SIZE))
    {
        // 检查帧头 0xA5 0x5A
        if (buffer[i] == VTM_SOF_1 && buffer[i + 1] == VTM_SOF_2)
        {
            // 尝试解包
            if (vtm_unpack_data(&buffer[i], VTM_FRAME_SIZE, &vtm_rx_data))
            {
                vtm_rx_valid = true;
                found = true;
                // 跳过这一帧，继续寻找下一帧 (处理粘包)
                i += VTM_FRAME_SIZE;
                continue;
            }
            else
            {
                // CRC 校验失败，说明可能是假帧头，向后移动一位继续找
                i++;
            }
        }
        else
        {
            i++;
        }
    }

    if (!found)
    {
        vtm_rx_valid = false;
    }

    return found;
}

void vtm_sync_to_control(void)
{
    if (vtm_rx_valid)
    {
        vtm_apply_to_control(&vtm_rx_data);
    }
}