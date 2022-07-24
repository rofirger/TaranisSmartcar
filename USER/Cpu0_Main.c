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

int16_t left_speed = 360, right_speed = 360;
extern RoadType road_type;
volatile float slope = 0;
unsigned char thredshold = 0;
bool is_right_out = false;
int32_t steer_pwm = 625;
float pwm_steer;
float last_pwm_steer;
extern int pwm_TFT;
// ���
uint32_t pwm_right;
uint32_t pwm_left;
// ֱ��ͼ
short hist_gram[256];
// �Ƿ���ͣ
extern bool is_stop;

extern float target_pwm_left;
extern float target_pwm_right;

extern int16_t left_encoder;
extern int16_t right_encoder;

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
    gpio_init(P21_5, GPO, 0, PUSHPULL);
    // TFT��ʼ��
    //lcd_init();
    GUI_init(ERU_CH5_REQ1_P15_8);
    // ������ʼ��
    uart_init(UART_3, 115200, UART3_TX_P21_7, UART3_RX_P21_6);  //��ʼ������

    // pit�ж�
    pit_interrupt_ms(CCU6_0, PIT_CH0, 20);
    pit_interrupt_ms(CCU6_0, PIT_CH1, 5);
    // ��ת���Լ�������
    pit_interrupt_ms(CCU6_1, PIT_CH0, 10);
    // esp spi��ʼ��
    //spi_init(SPI_1, SPI1_SCLK_P10_2, SPI1_MOSI_P10_3, SPI1_MISO_P10_1, SPI1_CS9_P10_5, 3, 10 * 1000 * 1000);
    icm20602_init_spi();
    systick_delay_ms(STM1,2000);
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
            {
                is_go = false;
                error_motor_left.currentError = 0;
                error_motor_left.lastError = 0;
                error_motor_left.previoursError = 0;
                error_motor_right.currentError = 0;
                error_motor_right.lastError = 0;
                error_motor_right.previoursError = 0;
                target_pwm_left = 0;
                target_pwm_right=0;
                pwm_duty(ATOM0_CH4_P02_4, 0);    // ����ǰ��
                pwm_duty(ATOM0_CH5_P02_5, 0);    // ����ǰ��
                pwm_duty(ATOM0_CH6_P02_6, 0);    // ���ֺ���
                pwm_duty(ATOM0_CH7_P02_7, 0);    // ������
            }

            if ((bend_type == NO_BEND || bend_type == LEFT_NORMAL_BEND || bend_type == RIGHT_NORMAL_BEND) && bend_deal._out_bend_count == 0)
            {
                //SteerPidChange(pid_steer.P, pid_steer.I, pid_steer.D);
                pwm_steer = PID_Pos(&error_steer, &pid_steer, 0, (float)mid_offset._total_offset / (float)(slope_cal._end_cal_y - mid_offset._end_src_rows));
                // ���pwm_steer����һ������ֵ,��Ҫע���Ƿ��Ǵ���ֱ���ϣ����ߴ���Сs����
                pwm_steer = last_pwm_steer * 0.2 + pwm_steer * 0.8;
                gpio_set(P21_5, 0);
            }
            else
            {
                //SteerPidChange(pid_steer_sharp.P, pid_steer_sharp.I, pid_steer_sharp.D);
                pwm_steer = PID_Pos(&sharp_error_steer, &pid_steer_sharp, 0, slope);
                gpio_set(P21_5, 1);
            }

            // ��·��
            if (road_type == LEFT_ROTARY_IN_SECOND_SUNKEN ||
                road_type == RIGHT_ROTARY_IN_SECOND_SUNKEN ||
                road_type == LEFT_ROTARY_OUT_FIRST_SUNKEN ||
                road_type == RIGHT_ROTARY_OUT_FIRST_SUNKEN)
            {
                pwm_steer = PID_Pos(&sharp_error_steer, &pid_steer_sharp, 0, slope);
                pwm_steer *= 1.5;
            }

            error_steer.loc_sum = 0;
            sharp_error_steer.loc_sum = 0;
            steer_pwm = 625 + atan(pwm_steer) * 95.0;

            if (steer_pwm > (700))
                steer_pwm = (700);
            else if (steer_pwm < (550))
                steer_pwm = (550);

            pwm_duty(ATOM0_CH1_P33_9, steer_pwm);      // 550����, 625��ֵ, 700����
            //pwm_duty(ATOM0_CH4_P02_4, 1000);    // ����ǰ��
            //pwm_duty(ATOM0_CH5_P02_5, 1000);    // ����ǰ��
            //pwm_duty(ATOM0_CH6_P02_6, 0);    // ����ǰ��
            //pwm_duty(ATOM0_CH7_P02_7, 0);    // ����ǰ��
            left_speed = LEFT_SPEED_BASE  - sharp_bend_sub_speed - pwm_steer * diff_speed._left_speed_factor;
            right_speed = RIGHT_SPEED_BASE - sharp_bend_sub_speed + pwm_steer * diff_speed._right_speed_factor;
            if (road_type != IN_CARBARN && is_go)
            {


                //pwm_duty(ATOM0_CH4_P02_4, pwm_right );    // ����ǰ��
                //pwm_duty(ATOM0_CH5_P02_5, pwm_left);    // ����ǰ��
                //pwm_duty(ATOM0_CH6_P02_6, 0 );    // ���ֺ���
                //pwm_duty(ATOM0_CH7_P02_7, 0);    // ���ֺ���
            }
            else if (road_type == IN_CARBARN)
            {
                //pwm_duty(ATOM0_CH1_P33_9, 700);      // 550����, 625��ֵ, 700����
                systick_delay_ms(STM0, 30);
                //pwm_duty(ATOM0_CH1_P33_9, 625);      // 550����, 625��ֵ, 700����
                LEFT_SPEED_BASE = 0;
                RIGHT_SPEED_BASE = 0;
                left_speed = 0;
                right_speed = 0;
                //pwm_duty(ATOM0_CH4_P02_4,0);
                //pwm_duty(ATOM0_CH5_P02_5,0);
                //pwm_duty(ATOM0_CH6_P02_6,0);
                //pwm_duty(ATOM0_CH7_P02_7,0);
            }

            if (road_type == LEFT_ROTARY_IN_FIRST_SUNKEN || road_type == RIGHT_ROTARY_IN_FIRST_SUNKEN)
            {
                //gpio_set(P33_10, 1);
            }
            else
            {
                //gpio_set(P33_10, 0);
            }

            if (location[0] == 3)
            {
                for (int i = 0; i < MT9V03X_H; ++i)
                {
                    src_pixel_mat[i][left_line[i]] = 0;
                    src_pixel_mat[i][mid_line[i]] = 0;
                    src_pixel_mat[i][right_line[i]] = 0;
                    if (i == slope_cal._start_cal_y || i == sharp_slope_cal._start_cal_y)
                    {
                        memset(src_pixel_mat[i], 0, sizeof(char) * MT9V03X_W);
                    }
                }
                lcd_displayimage032((uint8_t*) src_pixel_mat, MT9V03X_W, MT9V03X_H);
                lcd_showint16(0, 5, mid_offset._end_src_rows);
                lcd_showint16(50, 5, bend_type);
                lcd_showfloat(0, 6, slope, 3, 3);
                lcd_showfloat(50, 6, pwm_steer, 4, 3);
                lcd_showuint16(0, 7, road_type);
                lcd_showint16(0, 9, bend_deal._bend_count);
                lcd_showint16(50, 9, bend_deal._out_bend_count);
                //lcd_showfloat(50, 7, (float)mid_offset._total_offset / (float)(MT9V03X_H - mid_offset._end_src_rows), 3, 3);
                //uart_putbuff(UART_3, src_pixel_mat[0], MT9V03X_W * MT9V03X_H);

                //spi_mosi(SPI_1, SPI1_CS9_P10_5, src_pixel_mat[0], NULL, 60 * 90, 1);
            }
//            if (location[0] == 2)
//            {
//                lcd_showint16(0, 7, target_pwm_left);
//                lcd_showint16(50, 7, target_pwm_right);
//                lcd_showint16(0, 8, left_encoder);
//                lcd_showint16(50, 8, right_encoder);
//            }
            mt9v03x_finish_flag = 0;
        }
        //supermonitor();
    }
}
