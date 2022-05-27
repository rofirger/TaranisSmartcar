/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ����Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		main
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		ADS v1.2.2
 * @Target core		TC264D
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-3-23
 ********************************************************************************************************************/


#include "headfile.h"
#pragma section all "cpu0_dsram"
#include "img_process.h"
#include "Config.h"
#include "fuzzy_pid.h"

// ȫ�ֱ�����
uint8_t src_pixel_mat[MT9V03X_H][MT9V03X_W];
int16_t encoder1;
int16_t encoder2;
uint8_t left_line[MT9V03X_H];
uint8_t mid_line[MT9V03X_H];
uint8_t right_line[MT9V03X_H];
// int16_t pwm_left = 8000, pwm_right = 7000;
#define LEFT_SPEED_BASE 200
#define RIGHT_SPEED_BASE 200

int16_t left_speed = 200, right_speed = 200;
extern RoadType road_type;
volatile float slope = 0;
unsigned char thredshold = 0;
// ���PD
PID pid_steer = {1.0, 0, 0.23};
PosErr error_steer = {{0, 0, 0}, 0};
int32_t steer_pwm = 625;
bool is_right_out = true;
// ���PID
PID pid_motor_left = {2.8, 2, 10};
Error error_motor_left = {0, 0, 0};
PID pid_motor_right = {2.8, 2, 10};
Error error_motor_right = {0, 0, 0};
// ֱ��ͼ
short hist_gram[256];

void Init()
{
    //disableInterrupts();
    get_clk();//��ȡʱ��Ƶ��  ��ر���
    mt9v03x_init(); //��ʼ������ͷ
    //�����Ļһֱ��ʾ��ʼ����Ϣ����������ͷ����
    //���ʹ�����壬һֱ����while(!uart_receive_flag)�������Ƿ�������OK?��������ͷ�����ô����뵥Ƭ�������Ƿ���ȷ
    //���ͼ��ֻ�ɼ�һ�Σ����鳡�ź�(VSY)�Ƿ�����OK?

    // ֱ�����PWM��ʼ��
    gtm_pwm_init(ATOM0_CH4_P02_4, 6000, 0);//ATOM 0ģ���ͨ��4 ʹ��P02_4�������PWM  PWMƵ��50HZ  ռ�ձȰٷ�֮0/GTM_ATOM0_PWM_DUTY_MAX*100  GTM_ATOM0_PWM_DUTY_MAX�궨����zf_gtm_pwm.h
    gtm_pwm_init(ATOM0_CH5_P02_5, 6000,  0);
    gtm_pwm_init(ATOM0_CH6_P02_6, 6000, 0);
    gtm_pwm_init(ATOM0_CH7_P02_7, 6000, 0);

    // ���ʹ��
    gpio_init(P21_2, GPO, 1, PUSHPULL);
    gtm_pwm_init(ATOM0_CH0_P21_2, 800, 10000);
    // ���PWM��ʼ��
    gtm_pwm_init(ATOM0_CH1_P33_9, 50, 625); // 550����, 625��ֵ, 700����

    // ��������ʼ��
    gpt12_init(GPT12_T2, GPT12_T2INB_P33_7, GPT12_T2EUDB_P33_6); // ���ֱ������� ����Ϊǰ��
    gpt12_init(GPT12_T4, GPT12_T4INA_P02_8, GPT12_T4EUDA_P00_9); // ���ֱ������� ����Ϊǰ��
    // ��������ʼ��
    gpio_init(P33_10, GPO, 0, PUSHPULL);
    // ��������ָʾ��GPIO��ʼ��
    gpio_init(P21_4, GPO, 0, PUSHPULL);
    // TFT��ʼ��
    lcd_init();
    // ������ʼ��
    uart_init (WIRELESS_UART, 57600, WIRELESS_UART_TX, WIRELESS_UART_RX);  //��ʼ������

    // pit�ж�
    pit_interrupt_ms(CCU6_0, PIT_CH0, 20);


    //IfxCpu_emitEvent(&g_cpuSyncEvent);
    //IfxCpu_waitEvent(&g_cpuSyncEvent, 0xFFFF);
    //systick_delay_ms(STM0, 20);
    enableInterrupts();

    // ���뿪�س�ʼ��
        gpio_init(SW1, GPI, 1, PULLDOWN);
        gpio_init(SW2, GPI, 1, PULLDOWN);
        gpio_init(SW3, GPI, 1, PULLDOWN);
        gpio_init(SW4, GPI, 1, PULLDOWN);
        gpio_init(SW5, GPI, 1, PULLDOWN);
        gpio_init(SW6, GPI, 1, PULLDOWN);


    pwm_duty(ATOM0_CH4_P02_4, 0);    // ����ǰ��
    pwm_duty(ATOM0_CH5_P02_5, 0);    // ����ǰ��
    pwm_duty(ATOM0_CH6_P02_6, 0);    // ���ֺ���
    pwm_duty(ATOM0_CH7_P02_7, 0);    // ���ֺ���
    // ��ձ���������
    gpt12_clear(GPT12_T2);
    gpt12_clear(GPT12_T4);

}

void Blink()
{
    static int8_t i = 0;
    if (i == 50)
    {
        gpio_toggle(P21_4);
        i = 0;
    }
    i++;
}

void SendImg(uint8_t* _img, uint16_t _width, uint16_t _height)
{
    uart_putchar(WIRELESS_UART, 0x01);uart_putchar(WIRELESS_UART, 0xfe);
    uart_putbuff(WIRELESS_UART, _img, _width * _height);  //����ͼ��
    uart_putchar(WIRELESS_UART, 0xfe);uart_putchar(WIRELESS_UART, 0x01);
}


bool is_go = false;
void Switch()
{
    if (gpio_get(SW1) == 0)
    {
        lcd_displayimage032(src_pixel_mat[0], MT9V03X_W, MT9V03X_H);
    }
    if (gpio_get(SW2) == 0)
    {
        lcd_showfloat(0, 0, slope, 2, 6);
    }
    if (gpio_get(SW3) == 0)
    {
        SendImg(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
    }
    if (gpio_get(SW4) == 0)
    {
        is_go = true;
    }
    else
    {
        is_go = false;
    }
    if(gpio_get(SW5) == 0)
    {
        lcd_showint32(0, 3, road_type, 3);
    }
}

int core0_main(void)
{
    Init();
    while (TRUE)
    {
        if (mt9v03x_finish_flag)
        {
            Blink();
            Switch();
            //lcd_displayimage032(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);

            for (int i = 0; i < MT9V03X_H; ++i)
            {
                  for (int j = 0; j < MT9V03X_W; ++j)
                  {
                        src_pixel_mat[i][j] = mt9v03x_image[i][j];
                  }
            }

            GetHistGram(MT9V03X_W, MT9V03X_H);
            thredshold = OTSUThreshold();
            BinaryzationProcess(MT9V03X_H, MT9V03X_W, thredshold);
            AuxiliaryProcess(MT9V03X_H, MT9V03X_W, thredshold, left_line, mid_line, right_line);
            UserProcess(left_line, mid_line, right_line, MT9V03X_H, MT9V03X_W, thredshold, &slope);
            for (uint8_t i = 0; i < MT9V03X_H; ++i)
            {
                src_pixel_mat[i][left_line[i]] = 0;
                src_pixel_mat[i][mid_line[i]] = 0;
                src_pixel_mat[i][right_line[i]] = 0;
            }


            slope = PID_Pos(&error_steer, &pid_steer, 0, slope);
            steer_pwm = 625 + atan(slope) * 95;
            if (steer_pwm > 700)
                steer_pwm = 700;
            else if (steer_pwm < 550)
                steer_pwm = 550;
            left_speed = LEFT_SPEED_BASE + slope * 20;
            right_speed = RIGHT_SPEED_BASE - slope * 20;
            pwm_duty(ATOM0_CH1_P33_9, steer_pwm);      // 550����, 625��ֵ, 700����
            if (road_type != NO_FIX_ROAD)
            {
                gpio_set(P33_10, 1);
            }
            else
            {
                gpio_set(P33_10, 0);
            }
            mt9v03x_finish_flag = 0;
        }
    }
}



#pragma section all restore
