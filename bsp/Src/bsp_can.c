#include "bsp_can.h"
#include "main.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void can_filter_init(void)
{
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}



void CAN_init(){
    // CAN��ʼ��
    // ʹ�ù����� CAN1: [0~13] CAN2: [14~27]

    // ��������
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0xFFFF;
    can_filter_st.FilterMaskIdLow = 0x0000;

    /****************** CAN1 ������ ***********************/
    can_filter_st.SlaveStartFilterBank = 0;

    // �Զ������ݼ�-[0x110~0x117]
    can_filter_st.FilterBank = 0;
//    can_filter_st.FilterIdHigh = (0x110 << 5);
//    can_filter_st.FilterMaskIdHigh = 0xFF00;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

    // ����CAN1������
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);

    /****************** CAN2 ������ ***********************/
    can_filter_st.SlaveStartFilterBank = 14;
    // ������0
    can_filter_st.FilterBank = 14;
    can_filter_st.FilterIdHigh = (0x200 << 5);
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);

    // ����CAN2������
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

uint8_t CANx_SendStdData(CAN_HandleTypeDef* hcan,uint16_t ID,uint8_t *pData,uint16_t Len)
{
    static CAN_TxHeaderTypeDef   Tx_Header;

    Tx_Header.StdId=ID;
    Tx_Header.ExtId=0;
    Tx_Header.IDE=0;
    Tx_Header.RTR=0;
    Tx_Header.DLC=Len;


    /*�ҵ��յķ������䣬�����ݷ��ͳ�ȥ*/
    if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
    {
        if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
        {
            HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t*)CAN_TX_MAILBOX2);
        }
    }
}
/*void CAN_Gimbal_init(){
    // ��̨��CAN��ʼ��
    // ʹ�ù����� CAN1: [0~6] CAN2: [14~20]

    // ��������
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0xFFFF;
    can_filter_st.FilterMaskIdLow = 0x0000;

    *//****************** CAN1 ������ ***********************//*
    can_filter_st.SlaveStartFilterBank = 0;
    // ������0
//    can_filter_st.FilterBank = 0;
//    can_filter_st.FilterIdHigh = 0x0000;
//    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
//
//    // ����CAN1������
//    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
//    HAL_CAN_Start(&hcan1);
//    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
//    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
    *//****************** CAN2 ������ ***********************//*
    can_filter_st.SlaveStartFilterBank = 14;
    // single_shoot-0x204 ������λ���
    can_filter_st.FilterIdHigh = (0x204 <<5);               // ����ID
    can_filter_st.FilterBank = 14;                          // ���������
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;      // �����ܵĵı��ķ���FIFO0
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);

    // Yaw-0x205 Yaw���
    can_filter_st.FilterBank = 18;
    can_filter_st.FilterIdHigh = (0x205 << 5);
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

    // pitch-0x206 Pitch���
    can_filter_st.FilterIdHigh = (0x206 <<5);               // ����ID
    can_filter_st.FilterBank = 15;                          // ���������
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;      // �����ܵĵı��ķ���FIFO0
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);

    // fire_r-0x207 ��Ħ���ֵ��
    can_filter_st.FilterIdHigh = (0x207 <<5);               // ����ID
    can_filter_st.FilterBank = 16;                          // ���������
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;      // �����ܵĵı��ķ���FIFO0
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);

    // fire_l-0x208 ��Ħ���ֵ��
    can_filter_st.FilterIdHigh = (0x208 <<5);               // ����ID
    can_filter_st.FilterBank = 17;                          // ���������
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;      // �����ܵĵı��ķ���FIFO0
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);

    // ����CAN2������
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}

void CAN_Chass_init(){
    // ���̵�CAN��ʼ��
    // ʹ�ù����� CAN1: [7~13] CAN2: [21~27]

    // ��������
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0xFFFF;
    can_filter_st.FilterMaskIdLow = 0x0000;

    *//****************** CAN1 ������ ***********************//*
    can_filter_st.SlaveStartFilterBank = 0;

    // motor_chassis[RF]-0x201 ���ϵ��̵��
    can_filter_st.FilterBank = 7;
    can_filter_st.FilterIdHigh = (0x201 << 5);
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

    // motor_chassis[LF]-0x202 ���ϵ��̵��
    can_filter_st.FilterBank = 8;
    can_filter_st.FilterIdHigh = (0x202 << 5);
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

    // motor_chassis[LB]-0x203 ���µ��̵��
    can_filter_st.FilterBank = 9;
    can_filter_st.FilterIdHigh = (0x203 << 5);
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

    // motor_chassis[RB]-0x204 ���µ��̵��
    can_filter_st.FilterBank = 10;
    can_filter_st.FilterIdHigh = (0x204 << 5);
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

    // CAN_IMAGE_2006_TRANSMISSION-0x205 ͼ�����
    can_filter_st.FilterBank = 11;
    can_filter_st.FilterIdHigh = (0x205 << 5);
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

    // TRIGGER-0x207 ���̵��
    can_filter_st.FilterBank = 12;
    can_filter_st.FilterIdHigh = (0x207 << 5);
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

    // �Զ������ݼ�-[0x110~0x117]
    can_filter_st.FilterBank = 13;
    can_filter_st.FilterIdHigh = (0x110 << 5);
    can_filter_st.FilterMaskIdHigh = 0xFF00;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

    // ����CAN1������
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
    *//****************** CAN2 ������ ***********************//*
*//*    can_filter_st.SlaveStartFilterBank = 14;
    // ������0
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

    // ����CAN2������
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);*//*
}*/
