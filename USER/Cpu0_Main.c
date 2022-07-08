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
#include "TFT_GUI.h"
#include "control.h"

// ȫ�ֱ�����
uint8_t src_pixel_mat[MT9V03X_H][MT9V03X_W];
int16_t encoder1;
int16_t encoder2;
uint8_t left_line[MT9V03X_H];
uint8_t mid_line[MT9V03X_H];
uint8_t right_line[MT9V03X_H];
// int16_t pwm_left = 8000, pwm_right = 7000;
int16_t LEFT_SPEED_BASE = 290;
int16_t RIGHT_SPEED_BASE = 290;

volatile int16_t left_speed = 290, right_speed = 290;
extern RoadType road_type;
volatile float slope = 0;
unsigned char thredshold = 0;
bool is_right_out = false;
int32_t steer_pwm = 625;
extern int16_t left_encoder;
extern int16_t right_encoder;
// ���
uint32_t pwm_right = 2200;
uint32_t pwm_left = 2200;
// ֱ��ͼ
short hist_gram[256];
// �Ƿ���ͣ
extern bool is_stop;

void Init ()
{
    //disableInterrupts();
    get_clk(); //��ȡʱ��Ƶ��  ��ر���
    mt9v03x_init(); //��ʼ������ͷ
    //�����Ļһֱ��ʾ��ʼ����Ϣ����������ͷ����
    //���ʹ�����壬һֱ����while(!uart_receive_flag)�������Ƿ�������OK?��������ͷ�����ô����뵥Ƭ�������Ƿ���ȷ
    //���ͼ��ֻ�ɼ�һ�Σ����鳡�ź�(VSY)�Ƿ�����OK?

    // ֱ�����PWM��ʼ��
    gtm_pwm_init(ATOM0_CH4_P02_4, 6000, 0); //ATOM 0ģ���ͨ��4 ʹ��P02_4�������PWM  PWMƵ��50HZ  ռ�ձȰٷ�֮0/GTM_ATOM0_PWM_DUTY_MAX*100  GTM_ATOM0_PWM_DUTY_MAX�궨����zf_gtm_pwm.h
    gtm_pwm_init(ATOM0_CH5_P02_5, 6000, 0);
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
    //lcd_init();
    GUI_init(ERU_CH5_REQ1_P15_8);
    // ������ʼ��
    uart_init(UART_3, 576000, UART3_TX_P21_7, UART3_RX_P21_6);  //��ʼ������

    // pit�ж�
    pit_interrupt_ms(CCU6_0, PIT_CH0, 20);

    // esp spi��ʼ��
    spi_init(SPI_1, SPI1_SCLK_P10_2, SPI1_MOSI_P10_3, SPI1_MISO_P10_1, SPI1_CS9_P10_5, 3, 10 * 1000 * 1000);

    gpio_init(P20_13, GPO, 0, PUSHPULL);

    pwm_duty(ATOM0_CH4_P02_4, 0);    // ����ǰ��
    pwm_duty(ATOM0_CH5_P02_5, 0);    // ����ǰ��
    pwm_duty(ATOM0_CH6_P02_6, 0);    // ���ֺ���
    pwm_duty(ATOM0_CH7_P02_7, 0);    // ���ֺ���
    // ��ձ���������
    gpt12_clear(GPT12_T2);
    gpt12_clear(GPT12_T4);
    enableInterrupts();
}

void Blink ()
{
    static int8_t i = 0;
    if (i == 50)
    {
        gpio_toggle(P21_4);
        i = 0;
    }
    i++;
}

void SendImg (uint8_t *_img, uint16_t _width, uint16_t _height)
{
    uart_putchar(UART_3, 0x01);
    uart_putchar(UART_3, 0xfe);
    uart_putbuff(UART_3, _img, _width * _height);  //����ͼ��
    uart_putchar(UART_3, 0xfe);
    uart_putchar(UART_3, 0x01);
}

volatile bool is_go = false;

void Stop ()
{
    uint8_t num_black = 0;
    for (int16_t i = MT9V03X_H - 1; i > MT9V03X_H - 10; --i)
    {
        if (left_line[i] == right_line[i])
        {
            num_black++;
        }
    }
    if (num_black > 5)
    {
        left_speed = 0;
        right_speed = 0;
    }
}

void ControlSpeed ()
{
    static uint8_t add = 0;
    if (add == 2)
    {
        LEFT_SPEED_BASE++;
        RIGHT_SPEED_BASE++;
        if (LEFT_SPEED_BASE > 200)
        {
            LEFT_SPEED_BASE = 200;
        }
        if (RIGHT_SPEED_BASE > 200)
        {
            RIGHT_SPEED_BASE = 200;
        }
        add = 0;
    }
    add++;
}

int core0_main (void)
{
    Init();
    while (road_type != IN_CARBARN)
    {
        gpio_set(P20_13, 0);
        if (mt9v03x_finish_flag)
        {
            Blink();
            //lcd_displayimage032(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
            //ControlSpeed();
            for (int i = 0; i < MT9V03X_H; ++i)
            {
                for (int j = 0; j < MT9V03X_W; ++j)
                {
                    src_pixel_mat[i][j] = mt9v03x_image[i][j];
                }
            }
            //if (location[0] == 3)
            //SendImg((uint8_t*) src_pixel_mat[0], MT9V03X_W, MT9V03X_H);
            GetHistGram(MT9V03X_W, MT9V03X_H);
            thredshold = OTSUThreshold();
            BinaryzationProcess(MT9V03X_H, MT9V03X_W, thredshold);
            AuxiliaryProcess(MT9V03X_H, MT9V03X_W, thredshold, left_line, mid_line, right_line);
            UserProcess(left_line, mid_line, right_line, MT9V03X_H, MT9V03X_W, thredshold, &slope);

            if (location[0] == 2)
            {
                is_go = true;
            }
            else
                is_go = false;
            if (road_type == LEFT_ROTARY_IN_SECOND_SUNKEN || road_type == RIGHT_ROTARY_IN_SECOND_SUNKEN)
            {
                slope = slope * 1.52;
            }
            if (road_type == LEFT_ROTARY_IN_FIRST_SUNKEN)
            {
                slope += 0.1;
            }
            if (road_type == RIGHT_ROTARY_IN_FIRST_SUNKEN)
            {
                slope -= 0.1;
            }
            if (road_type == IN_LEFT_ROTARY)
            {
                slope -= 0.12;
            }
            if (road_type == IN_RIGHT_ROTARY)
            {
                slope += 0.12;
            }
            if (road_type == LEFT_ROTARY_OUT_FIRST_SUNKEN)
            {
                slope += 0.05;
            }
            if (road_type == RIGHT_ROTARY_OUT_FIRST_SUNKEN)
            {
                slope -= 0.05;
            }
            Stop();
            slope = PID_Pos(&error_steer, &pid_steer, 0, slope);
            steer_pwm = 625 + atan(slope) * 95;
            if (steer_pwm > 700)
                steer_pwm = 700;
            else if (steer_pwm < 550)
                steer_pwm = 550;
            pwm_duty(ATOM0_CH1_P33_9, steer_pwm);      // 550����, 625��ֵ, 700����
            if (road_type != IN_CARBARN && is_go)
            {

                left_speed = LEFT_SPEED_BASE - slope * 13.5;
                right_speed = RIGHT_SPEED_BASE + slope * 13.5;
                //pwm_duty(ATOM0_CH4_P02_4, pwm_right + slope * 110 );    // ����ǰ��
                //pwm_duty(ATOM0_CH5_P02_5, pwm_left - slope * 110);    // ����ǰ��
            }
            else if (road_type == IN_CARBARN)
            {
                //pwm_duty(ATOM0_CH1_P33_9, 700);      // 550����, 625��ֵ, 700����
                systick_delay_ms(STM0, 30);
                pwm_duty(ATOM0_CH1_P33_9, 625);      // 550����, 625��ֵ, 700����
                LEFT_SPEED_BASE = 0;
                RIGHT_SPEED_BASE = 0;
                left_speed = 0;
                right_speed = 0;
                //pwm_duty(ATOM0_CH4_P02_4,0);
                //pwm_duty(ATOM0_CH5_P02_5,0);
                //pwm_duty(ATOM0_CH6_P02_6,0);
                //pwm_duty(ATOM0_CH7_P02_7,0);
            }

            if (road_type == LEFT_ROTARY_IN_SECOND_SUNKEN || road_type == RIGHT_ROTARY_IN_SECOND_SUNKEN)
            {
                gpio_set(P33_10, 1);
            }
            else
            {
                gpio_set(P33_10, 0);
            }

            if (location[0] == 3)
            {
                for (int i = 0; i < MT9V03X_H; ++i)
                {
                    src_pixel_mat[i][left_line[i]] = 0;
                    src_pixel_mat[i][mid_line[i]] = 0;
                    src_pixel_mat[i][right_line[i]] = 0;
                }
                lcd_displayimage032((uint8_t*) src_pixel_mat, MT9V03X_W, MT9V03X_H);
                lcd_showfloat(0, 6, slope, 3, 3);
                lcd_showuint16(0, 7, road_type);
                //uart_putbuff(UART_3, src_pixel_mat[0], MT9V03X_W * MT9V03X_H);

                //spi_mosi(SPI_1, SPI1_CS9_P10_5, src_pixel_mat[0], NULL, 60 * 90, 1);
            }

            mt9v03x_finish_flag = 0;
        }
    }
}

#pragma section all restore
