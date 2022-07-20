#include "headfile.h"
#pragma section all "cpu0_dsram"
#include "img_process.h"
#include "Config.h"
#include "fuzzy_pid.h"
#include "TFT_GUI.h"
#include "control.h"
#include "debug.h"

// ȫ�ֱ�����
uint8_t src_pixel_mat[MT9V03X_H][MT9V03X_W];
int16_t encoder1;
int16_t encoder2;
uint8_t left_line[MT9V03X_H];
uint8_t mid_line[MT9V03X_H];
uint8_t right_line[MT9V03X_H];
// int16_t pwm_left = 8000, pwm_right = 7000;
int16_t LEFT_SPEED_BASE = 360;
int16_t RIGHT_SPEED_BASE = 360;
float huandao;

 int16_t left_speed = 140, right_speed = 140;
extern RoadType road_type;
volatile float slope = 0;
unsigned char thredshold = 0;
bool is_right_out = false;
int32_t steer_pwm = 625;
extern int16_t left_encoder;
extern int16_t right_encoder;
extern int pwm_TFT;
// ���
uint32_t pwm_right;
uint32_t pwm_left ;
// ֱ��ͼ
short hist_gram[256];
// �Ƿ���ͣ
extern bool is_stop;
extern int _kind;
int steer_cha;
int _kind_sum=0;
int _kind_sum_aim;
int _kind_sum_end;
int steer_cha2;
extern int juli;
int juli_temp;
int xianfu;
int juli_jizhuan;
int sancha_sum1;
int sancha_sum2;
int sancha_sum_aim;
PID pid_steer_temp;
PID pid_steer_new;
extern int _kind_end_sumf;
int yuzhi;//slope��СS�����⴦�����ֵ
float pwm_steer;
int ruhuan_pwm;
extern int huandao_inL;
extern int huandao_outL;
extern int huandao_inR;
extern int huandao_outR;
extern float piancha;
extern float piancha_temp;

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
    uart_init(UART_3, 115200, UART3_TX_P21_7, UART3_RX_P21_6);  //��ʼ������

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



            //Stop();
            if(road_type == IN_LEFT_JUNCTION_ING||road_type == OUT_LEFT_JUNCTION_ING)
            {
                juli = 15;
            }

            if(road_type == IN_RIGHT_JUNCTION_ING||road_type == OUT_RIGHT_JUNCTION_ING)
            {
                juli = 15;
            }


            if(_kind==1||_kind_end_sumf==1)
            {
                juli = juli_jizhuan;
                pid_steer.P=pid_steer_new.P;
                pid_steer.D=pid_steer_new.D;


            }
            if(_kind==2||_kind_end_sumf==2)
            {
                juli = juli_jizhuan;
                pid_steer.P=pid_steer_new.P;
                pid_steer.D=pid_steer_new.D;
            }
            if(_kind==0)
            {
                juli = juli_temp;
                pid_steer.P=pid_steer_temp.P;
                pid_steer.D=pid_steer_temp.D;
            }
            /*
            if(_kind==1&&slope<-yuzhi)
            {
                juli = juli_temp;
                pid_steer.P=pid_steer_temp.P;
                pid_steer.D=pid_steer_temp.D;
            }
            if(_kind==2&&slope>yuzhi)
            {
                juli = juli_temp;
                pid_steer.P=pid_steer_temp.P;
                pid_steer.D=pid_steer_temp.D;
            }
            */

            if(road_type!=LEFT_ROTARY_IN_SECOND_SUNKEN&&road_type!=RIGHT_ROTARY_IN_SECOND_SUNKEN&&_kind==0)
            {
                juli = juli_temp;
            }
            if(road_type!=IN_LEFT_ROTARY&&road_type !=IN_RIGHT_ROTARY&&_kind==0)
            {
                juli = juli_temp;
            }
            if(road_type == LEFT_ROTARY_IN_SECOND_SUNKEN||road_type == RIGHT_ROTARY_IN_SECOND_SUNKEN)
            {
                juli = 15;
            }

            if(road_type == LEFT_ROTARY_IN_FIRST_SUNKEN||road_type == RIGHT_ROTARY_IN_FIRST_SUNKEN)
            {
                juli = 15;
            }
            if(road_type ==IN_LEFT_ROTARY||road_type ==IN_RIGHT_ROTARY)
            {
                juli = 15;
            }
            if(road_type == LEFT_ROTARY_OUT_FIRST_SUNKEN||road_type == RIGHT_ROTARY_OUT_FIRST_SUNKEN)
            {
                juli = 15;
            }
            if((huandao_inL == 1&&huandao_outL == 0)||(huandao_inR == 1&&huandao_outR == 0))
            {
                juli =15;
            }


            /*
            if(road_type == IN_LEFT_JUNCTION_ING||road_type == IN_RIGHT_JUNCTION_ING)
            {
                sancha_sum1++;
                juli = 15;
            }
            if((sancha_sum1>sancha_sum_aim)&&(sancha_sum2<sancha_sum_aim))
            {
                juli = juli_temp;
            }
            if(road_type == OUT_LEFT_JUNCTION_ING||road_type == OUT_RIGHT_JUNCTION_ING)
            {
                sancha_sum2++;
                juli = 15;
            }
            if((sancha_sum1>sancha_sum_aim)&&(sancha_sum2>sancha_sum_aim))
            {
                juli = juli_temp;
            }
            */

            if(_kind==1)
            {
                slope = fabsf(slope)/20;
            }
            if(_kind==2)
            {
                slope = -fabsf(slope)/20;
            }

            pwm_steer = PID_Pos(&error_steer, &pid_steer, 0, slope);
            error_steer.loc_sum = 0;
            steer_pwm = 625 + atan(pwm_steer) * 95.0;


            if(road_type==LEFT_ROTARY_IN_FIRST_SUNKEN)
            {
                piancha = 0.5;
            }
            if(road_type==RIGHT_ROTARY_IN_FIRST_SUNKEN)
            {
                piancha = 0.5;
            }
            if(road_type!=RIGHT_ROTARY_IN_FIRST_SUNKEN||road_type!=LEFT_ROTARY_IN_FIRST_SUNKEN)
            {
                piancha = piancha_temp;
            }




            if (steer_pwm > (700-xianfu)&&juli==juli_temp)
                steer_pwm =(700-xianfu);
            else if (steer_pwm < (550+xianfu)&&juli==juli_temp)
                steer_pwm = (550+xianfu);
            if (steer_pwm > 700&&(juli==15||juli==juli_jizhuan))
                steer_pwm = 700;
            else if (steer_pwm < 550&&(juli==15||juli==juli_jizhuan))
                steer_pwm = 550;



            if(_kind==1)
            {
                _kind_sum++;
            }
            if(_kind==2)
            {
                _kind_sum++;
            }
            if(_kind==0)
            {
                _kind_sum=0;
            }
            /*
            if(_kind==1&&_kind_sum>_kind_sum_aim)
            {
                steer_pwm = 700-steer_cha;
            }
            if(_kind==2&&_kind_sum>_kind_sum_aim)
            {
                steer_pwm = 550+steer_cha;
            }
            if(_kind==1&&_kind_sum>_kind_sum_end)
            {
                steer_pwm = 700-steer_cha2;
            }
            if(_kind==2&&_kind_sum>_kind_sum_end)
            {
                steer_pwm = 550+steer_cha2;
            }*/

            pwm_duty(ATOM0_CH1_P33_9, steer_pwm);      // 550����, 625��ֵ, 700����
            //pwm_duty(ATOM0_CH4_P02_4, 1000);    // ����ǰ��
            //pwm_duty(ATOM0_CH5_P02_5, 1000);    // ����ǰ��
            //pwm_duty(ATOM0_CH6_P02_6, 0);    // ����ǰ��
            //pwm_duty(ATOM0_CH7_P02_7, 0);    // ����ǰ��
            if (road_type != IN_CARBARN && is_go && _kind == 0)
            {
                pwm_right = pwm_TFT;
                pwm_left = pwm_TFT;

                //left_speed = LEFT_SPEED_BASE - slope * 18.5;
                //right_speed = RIGHT_SPEED_BASE + slope * 18.5;
                //pwm_duty(ATOM0_CH4_P02_4, pwm_right );    // ����ǰ��
                //pwm_duty(ATOM0_CH5_P02_5, pwm_left);    // ����ǰ��
                //pwm_duty(ATOM0_CH6_P02_6, 0 );    // ���ֺ���
                //pwm_duty(ATOM0_CH7_P02_7, 0);    // ���ֺ���
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

            if (road_type == LEFT_ROTARY_IN_FIRST_SUNKEN || road_type == RIGHT_ROTARY_IN_FIRST_SUNKEN || _kind != 0)
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
                //lcd_showfloat(0, 6, slope, 3, 3);
                //lcd_showuint16(0, 7, road_type);
                //uart_putbuff(UART_3, src_pixel_mat[0], MT9V03X_W * MT9V03X_H);

                //spi_mosi(SPI_1, SPI1_CS9_P10_5, src_pixel_mat[0], NULL, 60 * 90, 1);
            }

            mt9v03x_finish_flag = 0;
        }
        supermonitor();
    }
}
