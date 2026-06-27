/////
//#include <launcher.h>
//#include <Detection.h>
//#include "bsp_led.h"
//#include "main.h"
//#include "cmsis_os.h"
//#include "Gimbal.h"
//#include "Chassis.h"
//#include "Auto.h"
//#include "Cap.h"
//extern chassis_t chassis;
//extern TIM_HandleTypeDef htim5;
//extern cap_info_t cap_info;
//extern cap2_info_t cap2;
//led_t led;
//
//uint8_t led_flowing;
///**
//  * @brief          aRGB show
//  * @param[in]      aRGB: 0xaaRRGGBB, 'aa' is alpha, 'RR' is red, 'GG' is green, 'BB' is blue
//  * @retval         none
//  */
///**
//  * @brief          ��ʾRGB
//  * @param[in]      aRGB:0xaaRRGGBB,'aa' ��͸����,'RR'�Ǻ�ɫ,'GG'����ɫ,'BB'����ɫ
//  * @retval         none
//  */
//void aRGB_led_show(uint32_t aRGB)
//{
//    static uint8_t alpha;
//    static uint16_t red,green,blue;
//
//    alpha = (aRGB & 0xFF000000) >> 24;
//    red = ((aRGB & 0x00FF0000) >> 16) * alpha;
//    green = ((aRGB & 0x0000FF00) >> 8) * alpha;
//    blue = ((aRGB & 0x000000FF) >> 0) * alpha;
//
//            __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, blue);
//            __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, green);
//            __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, red);
//}
//
//#define LED_SET(n) HAL_GPIO_WritePin(LED##n##_PORT,LED##n##_PIN,GPIO_PIN_SET)
//#define LED_RESET(n) HAL_GPIO_WritePin(LED##n##_PORT,LED##n##_PIN,GPIO_PIN_RESET)
//
//void led_dashboard(){
//    //switch(led.mode){
//    //case SPIN:
//    if(chassis.mode==CHASSIS_SPIN)//��
//    {
//        LED_RESET(6);
//    }
//    else{
//        LED_SET(6);
//    }
//    //break;
//
//    //case SHOOT:
//    if(ABS(launcher.fire_l.motor_measure->speed_rpm)>500&&ABS(launcher.fire_r.motor_measure->speed_rpm)>500)
//        HAL_GPIO_WritePin(LED5_PORT,LED5_PIN,GPIO_PIN_RESET);
//    else
//        HAL_GPIO_WritePin(LED5_PORT,LED5_PIN,GPIO_PIN_SET);
//    // break;
//
//    //case AUTO_AIM:
//    if(detect_list[DETECT_AUTO_AIM].status==OFFLINE)
//    {
//        HAL_GPIO_WritePin(LED1_PORT,LED1_PIN,GPIO_PIN_SET);
//    }
//    else{
//        HAL_GPIO_WritePin(LED1_PORT,LED1_PIN,GPIO_PIN_RESET);
//    };
//    // break;
//
//    // case CAP:
//    if(cap2.send_data[0]==0xFF)//���ݿ�����
//        HAL_GPIO_WritePin(LED2_PORT,LED2_PIN,GPIO_PIN_RESET);
//    else
//        HAL_GPIO_WritePin( LED2_PORT,LED2_PIN,GPIO_PIN_SET);
//    // }
//}
//
//
//void led_init(){
//    HAL_GPIO_WritePin(LED1_PORT,LED1_PIN,GPIO_PIN_SET);
//    HAL_GPIO_WritePin(LED3_PORT,LED3_PIN,GPIO_PIN_SET);
//    HAL_GPIO_WritePin(LED5_PORT,LED5_PIN,GPIO_PIN_SET);
//    HAL_GPIO_WritePin(LED7_PORT,LED7_PIN,GPIO_PIN_SET);
//    HAL_GPIO_WritePin(LED2_PORT,LED2_PIN,GPIO_PIN_SET);
//    HAL_GPIO_WritePin(LED4_PORT,LED4_PIN,GPIO_PIN_SET);
//    HAL_GPIO_WritePin(LED6_PORT,LED6_PIN,GPIO_PIN_SET);
//}
//
//void led_light(uint8_t led_1,uint8_t led_2,uint8_t led_3,uint8_t led_4,uint8_t led_5,uint8_t led_6,uint8_t led_7){
//    if(led_1==1){
//        HAL_GPIO_WritePin(LED1_PORT,LED1_PIN,GPIO_PIN_RESET);
//    }
//    else{
//        HAL_GPIO_WritePin(LED1_PORT,LED1_PIN,GPIO_PIN_SET);
//    }
//
//    if(led_2==1){
//        HAL_GPIO_WritePin(LED2_PORT,LED2_PIN,GPIO_PIN_RESET);
//    }
//    else{
//        HAL_GPIO_WritePin(LED2_PORT,LED2_PIN,GPIO_PIN_SET);
//    }
//
//    if(led_3==1){
//        HAL_GPIO_WritePin(LED3_PORT,LED3_PIN,GPIO_PIN_RESET);
//    }
//    else{
//        HAL_GPIO_WritePin(LED3_PORT,LED3_PIN,GPIO_PIN_SET);
//    }
//
//    if(led_4==1){
//        HAL_GPIO_WritePin(LED4_PORT,LED4_PIN,GPIO_PIN_RESET);
//    }
//    else{
//        HAL_GPIO_WritePin(LED4_PORT,LED4_PIN,GPIO_PIN_SET);
//    }
//
//    if(led_5==1){
//        HAL_GPIO_WritePin(LED5_PORT,LED5_PIN,GPIO_PIN_RESET);
//    }
//    else{
//        HAL_GPIO_WritePin(LED5_PORT,LED5_PIN,GPIO_PIN_SET);
//    }
//
//    if(led_6==1){
//        HAL_GPIO_WritePin(LED6_PORT,LED6_PIN,GPIO_PIN_RESET);
//    }
//    else{
//        HAL_GPIO_WritePin(LED6_PORT,LED6_PIN,GPIO_PIN_SET);
//    }
//
//    if(led_7==1){
//        HAL_GPIO_WritePin(LED7_PORT,LED7_PIN,GPIO_PIN_RESET);
//    }
//    else{
//        HAL_GPIO_WritePin(LED7_PORT,LED7_PIN,GPIO_PIN_SET);
//    }
//}
//
//
//void led_flow(){
//    led_flowing=1;
//    led_light(1,0,0,0,0,0,1);
//    osDelay(DELAY_TIME);
//
//    led_light(0,0,0,0,0,0,0);
//    osDelay(DELAY_TIME);
//
//    led_light(0,0,1,0,1,0,0);
//    osDelay(DELAY_TIME);
//
//    led_light(0,0,0,1,0,0,0);
//    osDelay(DELAY_TIME);
//
//    led_light(1,0,0,1,0,0,1);
//    osDelay(DELAY_TIME);
//
//    led_light(0,1,0,1,0,1,0);
//    osDelay(DELAY_TIME);
//
//    led_light(0,0,1,1,1,0,0);
//    osDelay(DELAY_TIME);
//
//    led_light(1,0,1,1,1,0,1);
//    osDelay(DELAY_TIME);
//
//    led_light(0,1,1,1,1,1,0);
//    osDelay(DELAY_TIME);
//
//    led_light(1,1,1,1,1,1,1);
//    osDelay(DELAY_TIME);
//
//    led_light(0,0,0,1,0,0,0);
//    osDelay(DELAY_TIME);
//
//    led_light(0,0,1,0,1,0,0);
//    osDelay(DELAY_TIME);
//
//    led_light(0,1,0,0,0,1,0);
//    osDelay(DELAY_TIME);
//
//    led_light(1,0,0,0,0,0,1);
//    osDelay(DELAY_TIME);
//
//    led_light(1,1,0,0,0,1,1);
//    osDelay(DELAY_TIME);
//
//    led_light(1,1,1,0,1,1,1);
//    osDelay(DELAY_TIME);
//
//    led_light(1,1,1,1,1,1,1);
//    osDelay(DELAY_TIME);
//
//    led_light(0,0,0,0,0,0,0);
//    osDelay(DELAY_TIME);
//
//    led_light(1,1,1,1,1,1,1);
//    osDelay(DELAY_TIME);
//
//    led_light(0,0,0,0,0,0,0);
//    osDelay(DELAY_TIME);
//
//    led_light(1,1,1,1,1,1,1);
//    osDelay(2000);
//
//    led_light(0,0,0,0,0,0,0);
//    led_flowing=0;
//}
//
//void led_off_flow(){
//    led_flowing=1;
//    led_light(1,1,1,1,1,1,1);
//    osDelay(DELAY_TIME);
//
//    led_light(0,0,0,0,0,0,0);
//    osDelay(DELAY_TIME);
//
//    led_light(1,1,1,1,1,1,1);
//    osDelay(DELAY_TIME);
//
//    led_light(0,1,1,1,1,1,0);
//    osDelay(DELAY_TIME);
//
//    led_light(1,0,1,1,1,0,1);
//    osDelay(DELAY_TIME);
//
//    led_light(1,1,0,1,0,1,1);
//    osDelay(DELAY_TIME);
//
//    led_light(1,1,1,0,1,1,1);
//    osDelay(DELAY_TIME);
//
//    led_light(0,1,1,0,1,1,0);
//    osDelay(DELAY_TIME);
//
//    led_light(1,0,1,0,1,0,1);
//    osDelay(DELAY_TIME);
//
//    led_light(1,1,0,0,0,1,1);
//    osDelay(DELAY_TIME);
//
//    led_light(0,1,0,0,0,1,0);
//    osDelay(DELAY_TIME);
//
//    led_light(1,0,0,0,0,0,1);
//    osDelay(DELAY_TIME);
//
//    led_light(0,1,0,0,0,1,0);
//    osDelay(DELAY_TIME);
//
//    led_light(0,0,1,0,1,0,0);
//    osDelay(DELAY_TIME);
//
//    led_light(0,0,0,1,0,0,0);
//    osDelay(DELAY_TIME);
//
//    led_light(0,0,0,0,0,0,0);
//    osDelay(DELAY_TIME);
//
//    led_light(0,0,1,0,1,0,0);
//    osDelay(DELAY_TIME);
//
//    led_light(0,1,0,0,0,1,0);
//    osDelay(DELAY_TIME);
//
//    led_light(1,0,0,0,0,0,1);
//    osDelay(DELAY_TIME);
//
//    led_light(0,0,0,0,0,0,0);
//    osDelay(DELAY_TIME);
//
//    led_light(1,0,0,0,0,0,1);
//    osDelay(DELAY_LONG_TIME);
//
//    led_light(0,0,0,0,0,0,0);
//    led_flowing=0;
//}
//
//
//
//void led_Task(void const *pvParameters){
//    vTaskDelay(CHASSIS_TASK_INIT_TIME);
//    led_init();
//
//    while(1){
//        if(chassis.last_mode==CHASSIS_RELAX && chassis.mode!=CHASSIS_RELAX){
//            led_flow();
//        }
//        else if(chassis.last_mode!=CHASSIS_RELAX && chassis.mode==CHASSIS_RELAX){
//            led_off_flow();
//        }
//        if(led_flowing==0){
//            led_dashboard();
//            vTaskDelay(10);
//        }
//    }
//
//
//}
//
// 24������UI�ư���´���


// todo SITREP 24/03/29:
// ���о����ִ��������������
// RGB_DAT�еĺ�ɫ����ɫ�����ǰһλ�ĵ��飬[0]���ã������һλͬʱ���Ƶ���ǰ��������
// ����ɫ����֣���[0]���Ӱ��ǰһλ
// e.g.�� {255,255,255},{128,128,128}
// ��ʾΪ��{224,128,128},{128,000,000}
// �����Ų飬����HAL_SPI_Transmit_DMA()��һ���Ų����ģ��������ڴ�����й�
// ��ֵ��ǲ���FreeRTOS��Ҳ������
// �ź�ĩ�˻���һ��Ī�������룬��ȫ�����²���ôӰ����ʾ
// ����Ҳ���ڴ浼�µģ���Ϊÿ��reset�궼��һ��


// ��Ϊ״̬��ʾʹ��ʱ
// 1. С���� = ��ɫ  ��־λ: chassis.mode = CHASSIS_SPIN
// 2. ���鿪 = ��ɫ  ��־λ��detect_list[DETECT_AUTO_AIM].status = ONLINE
// 3. ������ = ��ɫ+��ɫ  ��־λ��ABS(launcher.fire_l.motor_measure->speed_rpm) > 500 && ABS(launcher.fire_r.motor_measure->speed_rpm)>500
// 4. ���ݿ� = ��ɫ  ��־λ��cap2.send_data[0] == 0xFF
// 5. ���տ� = ��ɫ  ��־λ��magazine_cover_is_closed = false


// ԭ��ws2812����ʹ�õ��߹�����ͨѶ�����������ߵ͵�ƽ�����ǿ��ߵ͵�ƽ�ĳ���ʱ�����ж�0��1
// *��Ȼ��֪�������õĵ������Ҫ����ٵĳ���ʱ������ws2812������
// ��SPI2��ΪTransmit only Master��ģ��ߵ͵�ƽ
//����Ȼ��PWMҲ�У�����C���PWM��������7����

#include "bsp_led.h"
#include "main.h"
#include "Chassis.h"
#include "spi.h"
#include "cmsis_os.h"
#include "launcher.h"

//�����ⲿ����
extern chassis_t chassis;
extern TIM_HandleTypeDef htim5;
extern bool magazine_cover_is_closed;
// ���õ���ɫ�����ȵ��ıȽϵͣ����������ʵ���е�������
//const RGBColor_TypeDef RED      = {5, 0, 0};
//const RGBColor_TypeDef GREEN    = {0, 5, 0};
//const RGBColor_TypeDef BLUE     = {0, 0, 5};
//const RGBColor_TypeDef YELLOW   = {5, 5, 0};
//const RGBColor_TypeDef MAGENTA  = {5, 0, 5};
//const RGBColor_TypeDef BLACK    = {0, 0, 0};
//const RGBColor_TypeDef WHITE    = {0, 5, 5};
//const RGBColor_TypeDef BRIGHT_WHITE    = { 255, 255, 255};

//SPI������Ϊ5.25M
//ģ��bit��:0xC0 Ϊ 0 (10000000),0xF8 Ϊ 1(11111000)
//                    -_______            -----___  һ����λ = 1/5.25MHz = 0.19us
const uint8_t code[]={0x80,0xF8};

//����ɫ������
RGBColor_TypeDef RGB_DAT[RGB_NUM];
//��Ч����״̬��
uint8_t RGB_Ongoing_flag = 1;
//�ɴ������ledģʽ��ò��û�ã���������̨��Ӧ�ò�����ش��룬�������ź���
led_t led;

//������������������������������������������������������������������������������������������������ʵ����������������������������������������������������������������������������������������������������

/**
  * @brief �ײ�SPI���ͺ���
  */
extern DMA_HandleTypeDef hdma_spi2_tx;
static void SPI_Send(uint8_t* SPI_RGB_BUFFER)
{
    /* �ж��ϴ�DMA��û�д������ */
    while(HAL_DMA_GetState(&hdma_spi2_tx) != HAL_DMA_STATE_READY);
    /* ����һ��(24bit��)RGB ���ݵ� 2812 */
    HAL_SPI_Transmit_DMA(&hspi2,SPI_RGB_BUFFER,24);
}




/**
  * @brief ��ɫ���ú���
  * @description ����LedId���������ɫ��������ΪColorɫ������ֱ��������Ƶ�
  * @param LedId �����ţ�Color ��ɫ�ṹ��
  */
void RGB_Set_Color(uint8_t LedId, RGBColor_TypeDef Color)
{
    if(LedId < RGB_NUM)
    {
        RGB_DAT[LedId].R = Color.R;
        RGB_DAT[LedId].G = Color.G;
        RGB_DAT[LedId].B = Color.B;
    }
}




/**
  * @brief ��ɫˢ�º���
  * @param refresh_NUM Ҫˢ�µĵ�������
  * @description ����ɫ���ݷ��͵����飬�Դ�֡�����ʱ
  */
void RGB_Refresh(uint8_t refresh_NUM)
{
    static uint8_t RGB_BUFFER[24];
    uint8_t data_B ,data_R ,data_G ;
    //��������ɫת��Ϊ 24 ��Ҫ���͵��ֽ�����
    if(refresh_NUM > 0 && refresh_NUM <= RGB_NUM)
    {
        for(uint8_t i = 0; i < refresh_NUM; ++i) //todo
        {

            //ȫ���Ȱ�
//            data_G = RGB_DAT[i].G;
//            data_R = RGB_DAT[i].R;
//            data_B = RGB_DAT[i].B;
            //PS��2812�ķ���˳���ǴӸ�λ��GRB
            //���������ڱ����۾��İ汾��ȫ�����е�̫���ˣ�
            int decline=8;//Խ���Խ��
            data_G = RGB_DAT[i].G /decline;
            data_R = RGB_DAT[i].R /decline;
            data_B = RGB_DAT[i].B /decline;
            for (uint8_t j = 0; j < 8; j++)
            {
                RGB_BUFFER[07 - j] = code[data_G & 0x01];
                RGB_BUFFER[15 - j] = code[data_R & 0x01];
                RGB_BUFFER[23 - j] = code[data_B & 0x01];
                data_G >>= 1;
                data_R >>= 1;
                data_B >>= 1;
            }
            SPI_Send(RGB_BUFFER);
        }

//        7�ŵ�������ݷ�����Ϻ󣬷��ͳ�ʱ��͵�ƽ ��־֡������Ա������ʾ��
        //�⺯����ʱ1ms�������ã�
//        vTaskDelay(1);
        //��������ʱ1�������ã�
//        for (uint8_t i = 0; i < 25; ++i)
//        {
//            RGB_BUFFER[i] = 0;
//        }
//        SPI_Send(RGB_BUFFER);
        //��������ʱ2��������ã�
        uint8_t RGB_DAT_NULL[24] = {};
        SPI_Send(RGB_DAT_NULL);
        //��������ʱ3 ???
//        uint8_t RGB_DAT_NULL = 0x00;
//        for (uint8_t i = 0; i < 25; ++i)
//        {
//            SPI_Send(&RGB_DAT_NULL);
//        }

    }
}




/**
  * @brief ������ws2812�������ݵ�SPI2�ķ�����ɻص�����
  * @description ÿ�η���������1���������Ʋʺ�����˳��
  */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if(hspi->Instance == hspi2.Instance)
    {
        if (RGB_Counter == 255)
        {
            RGB_Counter = 0;
        }
        else
        {
            RGB_Counter++;
        }
    }
}




/**
  * @brief          aRGB show
  * @param[in]      aRGB: 0xaaRRGGBB, 'aa' is alpha, 'RR' is red, 'GG' is green, 'BB' is blue
  * @retval         none
  */
/**
  * @brief          ��ʾRGB
  * @param[in]      aRGB:0xaaRRGGBB,'aa' ��͸����,'RR'�Ǻ�ɫ,'GG'����ɫ,'BB'����ɫ
  * @retval         none
  */
void aRGB_led_show(uint32_t aRGB)//CV�����ϴ��룬����C���Դ��ĵ�
{
    static uint8_t alpha;
    static uint16_t red,green,blue;

    alpha = (aRGB & 0xFF000000) >> 24;
    red = ((aRGB & 0x00FF0000) >> 16) * alpha;
    green = ((aRGB & 0x0000FF00) >> 8) * alpha;
    blue = ((aRGB & 0x000000FF) >> 0) * alpha;
            __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, blue);
            __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, green);
            __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, red);
}






//�������������������������������������������������������������������������������������߼�����ʵ��������������������������������������������������������������������������������������������������������
/**
  * @brief �ư�����
  * @description FreeRTOS, RGB LED dashboard main task
  */
extern RC_ctrl_t rc_ctrl;
void led_Task(void const *pvParameters)
{
    vTaskDelay(357);
    while(1)
    {
        //������������������������������������debug����������������������������������������������
//        led_dashboard(); д�����õĴ�����

//        chassis.mode = CHASSIS_ONLY;
//        chassis.last_mode = CHASSIS_FOLLOW_GIMBAL;
//        launcher.fire_l.motor_measure->speed_rpm = 600;
//        launcher.fire_r.motor_measure->speed_rpm = 600;
//        detect_list[DETECT_AUTO_AIM].status = OFFLINE;
//        cap2.send_data[0] = 0xFF;
//        magazine_cover_is_closed = true;
        //������������������������������������end��������������������������������������������������

        //���ϵ�ʱֱ�Ӳʺ��Ч
        if(chassis.last_mode == CHASSIS_RELAX && chassis.mode == CHASSIS_RELAX)
        {
            //RGB_Debug();
            RGB_Rainbow(RGB_NUM, 1);
            RGB_Ongoing_flag = 1;
        }
        //��ʧ���л�����ʱ��һ��֮�����Ǳ���
        if(chassis.last_mode == CHASSIS_RELAX && chassis.mode != CHASSIS_RELAX)
        {
            if (RGB_Ongoing_flag == 1)
            {
                for (uint8_t i = 0; i < 28; ++i)
                {
                    RGB_Rainbow(RGB_NUM, 2);
                }
            }


            RGB_Ongoing_flag = 0;
        }
            //������ģʽ����ʧ�ܺ����ʺ�
        else if(chassis.last_mode != CHASSIS_RELAX && chassis.mode == CHASSIS_RELAX)
        {
            RGB_Rainbow(RGB_NUM, 0);
            RGB_Ongoing_flag = 1;
        }
            //��������¿����Ǳ���
        else if(chassis.last_mode != CHASSIS_RELAX && chassis.mode != CHASSIS_RELAX)
        {
            RGB_Ongoing_flag = 0;
        }

        //�����Ǳ���
        if(RGB_Ongoing_flag == 0)
        {
            // led_dashboard();

        }
//        osDelay(10);
        //      led_dashboard();
        osDelay(10);
    }
}


/**
  * @brief �ư�debug
  */
void RGB_Debug()
{
    for (int i = 0; i < RGB_NUM; ++i)
    {
        RGB_Set_Color(i, (RGBColor_TypeDef){0, 0, 0});
    }
    RGB_DAT[0].R = 255;
    RGB_DAT[0].B = 255;
    RGB_DAT[0].G = 255;
    RGB_DAT[1].R = 128;
    RGB_DAT[1].B = 128;
    RGB_DAT[1].G = 128;
    //    RGB_Set_Color(13, (RGBColor_TypeDef){0, 255, 0}); //��
//    RGB_Set_Color(1, (RGBColor_TypeDef){255, 255, 0}); //��
//    RGB_Set_Color(2, (RGBColor_TypeDef){0, 255, 0}); //��
//    RGB_Set_Color(3, (RGBColor_TypeDef){0, 255, 255}); //��
//    RGB_Set_Color(4, (RGBColor_TypeDef){0, 0, 255}); //��
//    RGB_Set_Color(5, (RGBColor_TypeDef){255, 0, 255}); //��
//    RGB_Set_Color(6, (RGBColor_TypeDef){255, 255, 255}); //��
    RGB_Refresh(RGB_NUM);
    osDelay(500);
}


/**
  * @brief �ư���ʾ״̬����
  * @description ���ݳ���״̬������Ӧ����
//   */
// void led_dashboard()
// {
//     //��յư�
//     for (uint8_t i = 0; i < RGB_NUM; ++i)
//     {
//         RGB_Set_Color(i, (RGBColor_TypeDef){0, 0, 0});
//     }
//
// //    led_Task();д�����õĴ�����
// // �ݸ�ֽ��0 6 2 4 5
//     //С�����ж�
//     if(true)
//     {
//         RGB_Set_Color(2, (RGBColor_TypeDef){0, 255, 0});//��ɫ
//     }
//     //�����ж�
//     if(ABS(launcher.fire_l.motor_measure->speed_rpm) > 500 && ABS(launcher.fire_r.motor_measure->speed_rpm) > 500)
//     {
//         RGB_Set_Color(6, (RGBColor_TypeDef){255, 255, 0});
//     }
//     //�����ж�
//     if(detect_list[DETECT_AUTO_AIM].status == OFFLINE)
//     {
//         RGB_Set_Color(0, (RGBColor_TypeDef){0, 0, 255});
//     }
//     //�����ж�
//     if(rc_ctrl.rc.s[1]==1)
//     {
//         RGB_Set_Color(4, (RGBColor_TypeDef){255, 0, 0});//��ɫ
//     }
//     //�����ж�
// //    if(magazine_cover_is_closed == false)
// //    {
// //        RGB_Set_Color(5, (RGBColor_TypeDef){255, 0, 255});
// //    }
//     if(rc_ctrl.rc.ch[4]<-300)
//     {
//         RGB_Set_Color(5, (RGBColor_TypeDef){255, 0, 255});
//     }
//     //���༭�õĵư����ݷ��͵��ư�
//     RGB_Refresh(RGB_NUM);
// }
//
//



/**
 * @brief ��ɫ��Ч
 * @description �õ�ɫ�������е���
 * @param RGB_LEN ��������
 * @param R ��ɫֵ 0~255
 * @param G ��ɫֵ 0~255
 * @param B ��ɫֵ 0~255
 */
void RGB_Single_Color(uint16_t RGB_LEN, uint8_t R, uint8_t G, uint8_t B)
{
    for(uint8_t i=0; i<RGB_LEN;i++)
    {
        RGB_Set_Color(i, (RGBColor_TypeDef) {R, G, B});
    }
    RGB_Refresh(RGB_LEN);
}




/**
  * @brief RGB�ʺ��Ч
  * @description ��������ܿ���?��Ϊһ�������Ҿ�����̫����,�ܷ����ҶԿ�ѰַRGB��Ч������,�Ƽ�������Ȥζ
  * @param RGB_LEN ������������RGB_NUM�ĺ궨�����
  * @param Mode ��Чģʽ��0Ϊ������䣬1Ϊһ�𽥱䣬2Ϊ�����������
  */
RGBColor_TypeDef RGB_Rainbow_Value = {255,0, 0};//��ʼ��λ
uint8_t Rainbow_Status_Flag = 1; //�����Ǵ�0��ʼ�ģ����ǡ���
void RGB_Rainbow(uint8_t RGB_LEN, uint8_t MODE)
{
    uint8_t rainbow_Step = 0;

    RGB_Ongoing_flag = 1;
    if(RGB_Rainbow_Value.R == 255 && RGB_Rainbow_Value.G == 0 && RGB_Rainbow_Value.B == 0)
    {
        Rainbow_Status_Flag = 1;
    }
    if(RGB_Rainbow_Value.G == 255 && RGB_Rainbow_Value.R == 0)
    {
        Rainbow_Status_Flag = 2;
    }
    if(RGB_Rainbow_Value.B == 255 && RGB_Rainbow_Value.R == 0)
    {
        Rainbow_Status_Flag = 3;
    }

    switch (MODE)
    {
        case 0:
            rainbow_Step = 5;
            break;
        case 1:
            rainbow_Step = 3;
            break;
        case 2:
            rainbow_Step = 15;
            break;
        default:
            rainbow_Step = 1;
            break;
    }

    switch(Rainbow_Status_Flag)
    {
        case 0:
            RGB_Rainbow_Value.R = (RGB_Rainbow_Value.R + rainbow_Step) ;
            break;
        case 1:
            RGB_Rainbow_Value.R = (RGB_Rainbow_Value.R - rainbow_Step) ;
            RGB_Rainbow_Value.G = (RGB_Rainbow_Value.G + rainbow_Step) ;
            break;
        case 2:
            RGB_Rainbow_Value.G = (RGB_Rainbow_Value.G - rainbow_Step) ;
            RGB_Rainbow_Value.B = (RGB_Rainbow_Value.B + rainbow_Step) ;
            break;
        case 3:
            RGB_Rainbow_Value.B = (RGB_Rainbow_Value.B - rainbow_Step) ;
            RGB_Rainbow_Value.R = (RGB_Rainbow_Value.R + rainbow_Step) ;
            break;
        default:
            break;
    }
    switch (MODE)
    {
        case 0://�������
        case 2://�����������
            RGB_Set_Color(RGB_Counter % RGB_NUM, RGB_Rainbow_Value);
            break;
        case 1://һ�𽥱�
            for(uint8_t i=0;i<RGB_LEN;i++)
            {
                RGB_Set_Color(i, RGB_Rainbow_Value);
            }
            break;
        default:
            break;
    }
    RGB_Refresh(RGB_LEN);
    osDelay(50);
    RGB_Ongoing_flag = 0;
}


/**
  * @brief RGB������Ч
  * @description ��ûд��
  * @param RGB_LEN ��������
  */
void RGB_Flow(uint16_t RGB_LEN, uint8_t R, uint8_t G, uint8_t B) //todo
{
    RGB_Set_Color(RGB_NUM, (RGBColor_TypeDef){0, 0, 0});
    RGB_DAT[RGB_Counter % RGB_LEN] = (RGBColor_TypeDef){R, G, B};
    RGB_Refresh(RGB_LEN);
}


/**
  * @brief ��ĳ����ɫ�������е���
  * @description ���Ե����ã���Ȼ��������ֿ��������ɫ��������ɫ
  */
void RGB_Test(void)
{
    uint8_t i;
    for(i=0;i<RGB_NUM;i++)
    {
        RGB_Set_Color(i, (RGBColor_TypeDef) {0x66, 0xCC, 0xFF});
    }
    RGB_Refresh(RGB_NUM);
}





//���������������������������������������������������������������������������������ϴ��룬�Ա���ʱ֮�衪��������������������������������������������������������������������������������������
//**
//  * @brief ֡������ͺ���
//  * @description ���ͳ��͵�ƽ���õ���֪��һ֡���ݷ�����ϣ�������ʾ�ˣ�ʹ��HAL_Delay��
//  */
//void RGB_RST(void)
//{
//    uint8_t dat[100] = {0};
//    /* �ж��ϴ�DMA��û�д������ */
//    while(HAL_DMA_GetState(&hdma_spi2_tx) != HAL_DMA_STATE_READY);
//    /* RGB RESET */
//    HAL_SPI_Transmit_DMA(&hspi2,dat,100);
//    HAL_Delay(10);
//    //osDelay(10);
//}



//**
//  * @brief ֡������ͺ���
//  * @description ���ͳ��͵�ƽ���õ���֪��һ֡���ݷ�����ϣ�������ʾ�ˣ����Ϳ����ݣ�
//  */
//void RGB_Reset(void)
//{
//    uint8_t dat[525] = {0};
//    /* �ж��ϴ�DMA��û�д������ */
//    while(HAL_DMA_GetState(&hdma_spi2_tx) != HAL_DMA_STATE_READY);
//    /* ���ͳ��͵�ƽ��152us) */
//    HAL_SPI_Transmit_DMA(&hspi2,dat,525);
//}
