

/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file            isr
 * @company         成都逐飞科技有限公司
 * @author          逐飞科技(QQ3184284598)
 * @version         查看doc内version文件 版本说明
 * @Software        ADS v1.2.2
 * @Target core     TC264D
 * @Taobao          https://seekfree.taobao.com/
 * @date            2020-3-23
 ********************************************************************************************************************/


#include "isr_config.h"
#include "isr.h"
#include "stdint.h"
#include "img_process.h"
#include "fuzzy_pid.h"
#include "control.h"
#include "TFT_GUI.h"
#include "math.h"

extern bool is_go;

extern int16_t left_speed;
extern int16_t right_speed;
volatile int16_t left_encoder;
volatile int16_t right_encoder;
volatile float target_pwm_left;
volatile float target_pwm_right;
float target_pwm_left_test;
float target_pwm_right_test;
int pwm_TFT;
extern int _kind;
extern int _kind_sum;
extern int _kind_sum_aim;
extern int _kind_sum_aim2;
int _kind_sum_sp;
int sp_kh;
int sp_kl;
int sp_n;
extern int juli;
extern int steer_pwm;
float sp_cha_kL;
float sp_cha_kL2;
float sp_cha_kR;
float sp_cha_kR2;
float start_sum;
float start_sum_aim;
int sancha_sum_in;
int sancha_in;
int sancha_sum_out;
int sancha_out;
int bang;
extern float slope;
PID pid_motor_leftn;
PID pid_motor_rightn;
 PID pid_motor_left_temp;
 PID pid_motor_right_temp;
 int jiansu;
 extern RoadType road_type;
 int _kind_last;
 int _kind_end_sumf;
 int _kind_end_sum;
 int _kind_end_sum_aim;
 int jiansu2;
 extern float pwm_steer;
 int inrot_jiansu;
float mid_jiansu_k;
int huandao_inL;
int huandao_outL;
int huandao_inR;
int huandao_outR;
int sancha_sum_in_1;

PID_STRUCT left_motor_pid_structrue = {0, 0, 0, 0, 0, 0};
PID_STRUCT right_motor_pid_structrue = {0, 0, 0, 0, 0, 0};

//PIT中断函数  示例
IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    enableInterrupts();//开启中断嵌套
    PIT_CLEAR_FLAG(CCU6_0, PIT_CH0);
    static left_encoder_last,right_encoder_last;
    left_encoder = abs(gpt12_get(GPT12_T2)); // 左轮编码器， 正数为前进
    right_encoder = abs(gpt12_get(GPT12_T4)); // 右轮编码器， 负数为前进
/*
    left_encoder=0.2*left_encoder_last+0.8*left_encoder;
    right_encoder=0.2*right_encoder_last+0.8*right_encoder;
    left_encoder_last=left_encoder;
    right_encoder_last=right_encoder;
    */


    if(_kind_last-_kind==1)
    {
        _kind_end_sumf=1;
    }
    if(_kind_last-_kind==2)
    {
        _kind_end_sumf=2;
    }
    if( _kind_end_sumf==1||_kind_end_sumf==2)
    {
        _kind_end_sum++;
    }
    if(_kind_end_sum==_kind_end_sum_aim)
    {
        _kind_end_sumf=0;
        _kind_end_sum=0;
    }

    /*
    if(road_type == LEFT_ROTARY_IN_SECOND_SUNKEN)
    {
        _kind = 1;
    }
    if(road_type == RIGHT_ROTARY_IN_SECOND_SUNKEN)
    {
        _kind = 2;
    }
    if(road_type == LEFT_ROTARY_OUT_FIRST_SUNKEN)
    {
        _kind = 1;
    }
    if(road_type == RIGHT_ROTARY_OUT_FIRST_SUNKEN)
    {
        _kind = 2;
    }
    */

    if(road_type == LEFT_ROTARY_IN_FIRST_SUNKEN)
    {
        huandao_outL = 0;
        huandao_inL = 1;
    }
    if(road_type == LEFT_ROTARY_OUT_FIRST_SUNKEN)
    {
        huandao_outL = 1;
        huandao_inL = 0;
    }
    if(road_type == RIGHT_ROTARY_IN_FIRST_SUNKEN)
    {
        huandao_outR = 0;
        huandao_inR = 1;
    }
    if(road_type == RIGHT_ROTARY_OUT_FIRST_SUNKEN)
    {
        huandao_outR = 1;
        huandao_inR = 0;
    }



    if(road_type == IN_LEFT_JUNCTION_ING||road_type == IN_RIGHT_JUNCTION_ING)
    {
        sancha_in=1;
        sancha_sum_in_1++;
    }
    if(sancha_in==1)
    {
        sancha_sum_in++;
    }
    if(sancha_in==0)
    {
        sancha_sum_in=0;
        sancha_sum_in_1=0;
    }
    if(road_type == OUT_LEFT_JUNCTION_ING||road_type == OUT_RIGHT_JUNCTION_ING)
    {
        sancha_out=1;
        sancha_in=0;
    }






/*
    if(_kind==1&&_kind_end_sumf==2)
    {
        _kind_end_sumf=0;
        _kind_end_sum=0;
    }
    if(_kind==2&&_kind_end_sumf==1)
    {
        _kind_end_sumf=0;
        _kind_end_sum=0;
    }
    */

    if(_kind==1&&_kind_sum>_kind_sum_aim2)
    {
        left_speed = sp_n*mid_jiansu_k-fabsf(pwm_steer)*sp_cha_kL;
        if(left_speed<0)
        {
            left_speed = 0;
        }
        right_speed = sp_kh*mid_jiansu_k+fabsf(pwm_steer)*sp_cha_kL2;
    }
    if(_kind==2&&_kind_sum>_kind_sum_aim2)
    {
        right_speed = sp_n*mid_jiansu_k-fabsf(pwm_steer)*sp_cha_kR;
        if(right_speed<0)
        {
            right_speed = 0;
        }
        left_speed = sp_kh*mid_jiansu_k+fabsf(pwm_steer)*sp_cha_kR2;
    }
    if(_kind_end_sumf==1)
    {
        left_speed = sp_n-fabsf(pwm_steer)*sp_cha_kL;
        if(left_speed<0)
        {
            left_speed = 0;
        }
        right_speed = sp_kh+fabsf(pwm_steer)*sp_cha_kL2;
    }
    if(_kind_end_sumf==2)
    {
        right_speed = sp_n-fabsf(pwm_steer)*sp_cha_kR;
        if(right_speed<0)
        {
            right_speed = 0;
        }
        left_speed = sp_kh+fabsf(pwm_steer)*sp_cha_kR2;
    }


    if(_kind==0)
    {
        left_speed=sp_n;
        right_speed=sp_n;
    }
    if(_kind==1&&_kind_sum<_kind_sum_aim)
    {
        left_speed =  jiansu;
        right_speed = jiansu2;
        pid_motor_left.P = pid_motor_leftn.P;
        pid_motor_left.I = pid_motor_leftn.I;
        pid_motor_right.P = pid_motor_rightn.P;
        pid_motor_right.I = pid_motor_rightn.I;

    }
    if(_kind==2&&_kind_sum<_kind_sum_aim)
    {
        left_speed =  jiansu2;
        right_speed = jiansu;
        pid_motor_left.P = pid_motor_leftn.P;
        pid_motor_left.I = pid_motor_leftn.I;
        pid_motor_right.P = pid_motor_rightn.P;
        pid_motor_right.I = pid_motor_rightn.I;

    }



    if(_kind==0)
    {
        pid_motor_left.P = pid_motor_left_temp.P;
        pid_motor_left.I = pid_motor_left_temp.I;
        pid_motor_right.P = pid_motor_right_temp.P;
        pid_motor_right.I = pid_motor_right_temp.I;
    }

    if(road_type==LEFT_ROTARY_IN_FIRST_SUNKEN||road_type==RIGHT_ROTARY_IN_FIRST_SUNKEN)
    {
        left_speed=inrot_jiansu;
        right_speed=inrot_jiansu;
        pid_motor_left.P = pid_motor_leftn.P;
        pid_motor_left.I = pid_motor_leftn.I;
        pid_motor_right.P = pid_motor_rightn.P;
        pid_motor_right.I = pid_motor_rightn.I;
    }
    if((huandao_inL == 1&&huandao_outL == 0)||road_type == LEFT_ROTARY_OUT_FIRST_SUNKEN)
    {
        left_speed = 120;
        right_speed = 160;
    }
    if((huandao_inR == 1&&huandao_outR == 0)||road_type == RIGHT_ROTARY_OUT_FIRST_SUNKEN)
    {
        left_speed = 160;
        right_speed = 120;
    }


    //if(sancha_in==1&&)
    /*
    if(road_type == IN_LEFT_JUNCTION_ING||road_type == OUT_LEFT_JUNCTION_ING)
    {
        left_speed = fabsf(pwm_steer)*5;
        right_speed = 0;
        pid_motor_left.P = pid_motor_leftn.P;
        pid_motor_left.I = pid_motor_leftn.I*2;
        pid_motor_right.P = pid_motor_rightn.P;
        pid_motor_right.I = pid_motor_rightn.I*2;
    }

    if (road_type == IN_LEFT_JUNCTION_ED || road_type == IN_RIGHT_JUNCTION_ED)
    {
        is_go = false;
    }
    if(road_type == IN_RIGHT_JUNCTION_ING||road_type == OUT_RIGHT_JUNCTION_ING)
    {
        left_speed = 0;
        right_speed = fabsf(pwm_steer)*5;
        pid_motor_left.P = pid_motor_leftn.P;
        pid_motor_left.I = pid_motor_leftn.I*2;
        pid_motor_right.P = pid_motor_rightn.P;
        pid_motor_right.I = pid_motor_rightn.I*2;
    }*/

    if (is_go)
    {



        start_sum++;
        if(start_sum>start_sum_aim)
        {
            start_sum = 1000;
        }
        if(start_sum<start_sum_aim)
        {
            left_speed = (float)sp_n*(start_sum/start_sum_aim);
            right_speed = (float)sp_n*(start_sum/start_sum_aim);
        }
        if(start_sum==1000&&_kind==0&&(road_type!=LEFT_ROTARY_IN_FIRST_SUNKEN&&road_type!=RIGHT_ROTARY_IN_FIRST_SUNKEN&&
            road_type!=LEFT_ROTARY_IN_SECOND_SUNKEN&&road_type!=RIGHT_ROTARY_IN_SECOND_SUNKEN)&&(huandao_inL != 1&&huandao_outL != 0)
                &&(huandao_inR != 1&&huandao_outR != 0)&&road_type != IN_LEFT_JUNCTION_ING&&road_type != IN_RIGHT_JUNCTION_ING
                &&road_type != OUT_LEFT_JUNCTION_ING&&road_type != OUT_RIGHT_JUNCTION_ING)
        {
            left_speed = sp_n;
            right_speed = sp_n;
        }
        target_pwm_left += PID_Increase(&error_motor_left, &pid_motor_left, left_encoder, left_speed);
        target_pwm_right += PID_Increase(&error_motor_right, &pid_motor_right, right_encoder, right_speed);

        //target_pwm_left += PID_Calc(&left_motor_pid_structrue, left_speed, left_encoder);
        //target_pwm_right += PID_Calc(&right_motor_pid_structrue, right_speed, right_encoder);
        target_pwm_left_test += PID_Increase(&error_motor_left, &pid_motor_left, left_encoder, left_speed);
        target_pwm_right_test += PID_Increase(&error_motor_right, &pid_motor_right, right_encoder, right_speed);

        if (target_pwm_left > 7000 || target_pwm_left < -5000)
            target_pwm_left = 0;
        if (target_pwm_right >7000 || target_pwm_right < -5000)
            target_pwm_right = 0;








        if(target_pwm_left >= 0&&target_pwm_right >=0)
        {
            pwm_duty(ATOM0_CH4_P02_4, target_pwm_right);    // 右轮前进
            pwm_duty(ATOM0_CH5_P02_5, target_pwm_left);    // 左轮前进
            pwm_duty(ATOM0_CH6_P02_6, 0);    // 左轮后退
            pwm_duty(ATOM0_CH7_P02_7, 0);    // 右轮后退
        }
        if (target_pwm_left < 0&&target_pwm_right > 0)
        {
            pwm_duty(ATOM0_CH4_P02_4, target_pwm_right);    // 右轮前进
            pwm_duty(ATOM0_CH5_P02_5, 0);    // 左轮前进
            pwm_duty(ATOM0_CH6_P02_6, -target_pwm_left);    // 左轮后退
            pwm_duty(ATOM0_CH7_P02_7, 0);    // 右轮后退
        }
        if (target_pwm_right < 0&&target_pwm_left > 0)
        {
            pwm_duty(ATOM0_CH4_P02_4, 0);    // 右轮前进
            pwm_duty(ATOM0_CH5_P02_5, target_pwm_left);    // 左轮前进
            pwm_duty(ATOM0_CH6_P02_6, 0);    // 左轮后退
            pwm_duty(ATOM0_CH7_P02_7, -target_pwm_right);    // 右轮后退
        }
        if(target_pwm_right < 0&&target_pwm_left < 0)
        {
            pwm_duty(ATOM0_CH4_P02_4, 0);    // 右轮前进
            pwm_duty(ATOM0_CH5_P02_5, 0);    // 左轮前进
            pwm_duty(ATOM0_CH6_P02_6, -target_pwm_left);    // 左轮后退
            pwm_duty(ATOM0_CH7_P02_7, -target_pwm_right);    // 右轮后退
        }



    }
    _kind_last = _kind;


if(location[0] != 1)
{

    //lcd_showint16(0, 4, _kind_end_sumf);
    //lcd_showint16(0, 5, _kind_end_sum);
    lcd_showint16(0, 5, _kind);
    lcd_showint16(0, 6, _kind_sum);
    lcd_showfloat(0, 7, slope, 3, 3);
    lcd_showuint16(0, 8, road_type);
    //lcd_showint16(0, 8, left_speed);
    gpt12_clear(GPT12_T2);
    gpt12_clear(GPT12_T4);
}
}


IFX_INTERRUPT(cc60_pit_ch1_isr, 0, CCU6_0_CH1_ISR_PRIORITY)
{
    enableInterrupts();//开启中断嵌套
    PIT_CLEAR_FLAG(CCU6_0, PIT_CH1);

}

IFX_INTERRUPT(cc61_pit_ch0_isr, 0, CCU6_1_CH0_ISR_PRIORITY)
{
    enableInterrupts();//开启中断嵌套
    PIT_CLEAR_FLAG(CCU6_1, PIT_CH0);

}

IFX_INTERRUPT(cc61_pit_ch1_isr, 0, CCU6_1_CH1_ISR_PRIORITY)
{
    enableInterrupts();//开启中断嵌套
    PIT_CLEAR_FLAG(CCU6_1, PIT_CH1);

}




IFX_INTERRUPT(eru_ch0_ch4_isr, 0, ERU_CH0_CH4_INT_PRIO)
{
    enableInterrupts();//开启中断嵌套
    if(GET_GPIO_FLAG(ERU_CH0_REQ4_P10_7))//通道0中断
    {
        CLEAR_GPIO_FLAG(ERU_CH0_REQ4_P10_7);
    }

    if(GET_GPIO_FLAG(ERU_CH4_REQ13_P15_5))//通道4中断
    {
        CLEAR_GPIO_FLAG(ERU_CH4_REQ13_P15_5);
    }
}

IFX_INTERRUPT(eru_ch1_ch5_isr, 0, ERU_CH1_CH5_INT_PRIO)
{
    enableInterrupts();//开启中断嵌套
    if(GET_GPIO_FLAG(ERU_CH1_REQ5_P10_8))//通道1中断
    {
        CLEAR_GPIO_FLAG(ERU_CH1_REQ5_P10_8);
    }

    if(GET_GPIO_FLAG(ERU_CH5_REQ1_P15_8))//通道5中断
    {
        eru_triggered();
        CLEAR_GPIO_FLAG(ERU_CH5_REQ1_P15_8);
    }
}

//由于摄像头pclk引脚默认占用了 2通道，用于触发DMA，因此这里不再定义中断函数
//IFX_INTERRUPT(eru_ch2_ch6_isr, 0, ERU_CH2_CH6_INT_PRIO)
//{
//  enableInterrupts();//开启中断嵌套
//  if(GET_GPIO_FLAG(ERU_CH2_REQ7_P00_4))//通道2中断
//  {
//      CLEAR_GPIO_FLAG(ERU_CH2_REQ7_P00_4);
//
//  }
//  if(GET_GPIO_FLAG(ERU_CH6_REQ9_P20_0))//通道6中断
//  {
//      CLEAR_GPIO_FLAG(ERU_CH6_REQ9_P20_0);
//
//  }
//}



IFX_INTERRUPT(eru_ch3_ch7_isr, 0, ERU_CH3_CH7_INT_PRIO)
{
    enableInterrupts();//开启中断嵌套
    if(GET_GPIO_FLAG(ERU_CH3_REQ6_P02_0))//通道3中断
    {
        CLEAR_GPIO_FLAG(ERU_CH3_REQ6_P02_0);
        if      (CAMERA_GRAYSCALE == camera_type)   mt9v03x_vsync();
        else if (CAMERA_BIN_UART  == camera_type)   ov7725_uart_vsync();
        else if (CAMERA_BIN       == camera_type)   ov7725_vsync();

    }
    if(GET_GPIO_FLAG(ERU_CH7_REQ16_P15_1))//通道7中断
    {
        CLEAR_GPIO_FLAG(ERU_CH7_REQ16_P15_1);

    }
}



IFX_INTERRUPT(dma_ch5_isr, 0, ERU_DMA_INT_PRIO)
{
    enableInterrupts();//开启中断嵌套

    if      (CAMERA_GRAYSCALE == camera_type)   mt9v03x_dma();
    else if (CAMERA_BIN_UART  == camera_type)   ov7725_uart_dma();
    else if (CAMERA_BIN       == camera_type)   ov7725_dma();
}


//串口中断函数  示例
IFX_INTERRUPT(uart0_tx_isr, 0, UART0_TX_INT_PRIO)
{
    enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrTransmit(&uart0_handle);
}
IFX_INTERRUPT(uart0_rx_isr, 0, UART0_RX_INT_PRIO)
{
    enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrReceive(&uart0_handle);
}
IFX_INTERRUPT(uart0_er_isr, 0, UART0_ER_INT_PRIO)
{
    enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrError(&uart0_handle);
}

//串口1默认连接到摄像头配置串口
IFX_INTERRUPT(uart1_tx_isr, 0, UART1_TX_INT_PRIO)
{
    enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrTransmit(&uart1_handle);
}
IFX_INTERRUPT(uart1_rx_isr, 0, UART1_RX_INT_PRIO)
{
    enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrReceive(&uart1_handle);
    if      (CAMERA_GRAYSCALE == camera_type)   mt9v03x_uart_callback();
    else if (CAMERA_BIN_UART  == camera_type)   ov7725_uart_callback();
}
IFX_INTERRUPT(uart1_er_isr, 0, UART1_ER_INT_PRIO)
{
    enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrError(&uart1_handle);
}


//串口2默认连接到无线转串口模块
IFX_INTERRUPT(uart2_tx_isr, 0, UART2_TX_INT_PRIO)
{
    enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrTransmit(&uart2_handle);
}
IFX_INTERRUPT(uart2_rx_isr, 0, UART2_RX_INT_PRIO)
{
    enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrReceive(&uart2_handle);
    switch(wireless_type)
    {
        case WIRELESS_SI24R1:
        {
            wireless_uart_callback();
        }break;

        case WIRELESS_CH9141:
        {
            bluetooth_ch9141_uart_callback();
        }break;
        default:break;
    }

}
IFX_INTERRUPT(uart2_er_isr, 0, UART2_ER_INT_PRIO)
{
    enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrError(&uart2_handle);
}



IFX_INTERRUPT(uart3_tx_isr, 0, UART3_TX_INT_PRIO)
{
    enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrTransmit(&uart3_handle);
}
IFX_INTERRUPT(uart3_rx_isr, 0, UART3_RX_INT_PRIO)
{
    enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrReceive(&uart3_handle);
    if(GPS_TAU1201 == gps_type)
    {
        gps_uart_callback();
    }
}
IFX_INTERRUPT(uart3_er_isr, 0, UART3_ER_INT_PRIO)
{
    enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrError(&uart3_handle);
}
