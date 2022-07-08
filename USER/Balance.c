/*
 * Balance.c
 *
 *  Created on: 2022年4月1日
 *      Author: robocar
 */
#include "Balance.h"


#define pi 3.1415926

/*
//求根公式
unsigned int m_sqrt(unsigned int x)
{
 int8  ans=0;
 int8 p=0x80;
 while(p!=0)
 {
     ans+=p;
 if(ans*ans>x)
 {
     ans-=p;
 }
     p=(int8)(p/2);
 }
 return(ans);
}*/

//一阶互补滤波
float acc_angle;     //初始加速度（经处理）
float gyro_angle;    //初始角速度（经处理）
float angle=0;


struct{
    float acc_ratio;      //加速度计比例
    float gyro_ratio;     //陀螺仪比例
    float dt;             //采样周期
}angle_calc_0={0.85,0.95,0.005};


float acc_ratio=20;      //加速度计比例
float gyro_ratio=20;     //陀螺仪比例
float dt=0.005;             //采样周期


float angle_calc(float angle_m, float gyro_m)
{

    float temp_angle;
    float error_angle;
    float gyro_now;
    static float last_angle0;
    static uint8 first_angle;

    if(!first_angle)//判断是否为第一次运行本函数
    {
        //如果是第一次运行，则将上次角度值设置为与加速度值一致
        first_angle = 1;
        last_angle0 = angle_m;
    }

    gyro_now = gyro_m * gyro_ratio;
    //根据测量到的加速度值转换为角度之后与上次的角度值求偏差
    error_angle = (angle_m - last_angle0)*acc_ratio;
    //根据偏差与陀螺仪测量得到的角度值计算当前角度值
    temp_angle = last_angle0 + (error_angle + gyro_now)*dt;
    //保存当前角度值
    last_angle0 = temp_angle;


    return temp_angle;

}
//角度   左负右正
void get_IMU_data()
{
    pit_disable_interrupt(CCU6_0, PIT_CH1);
    get_icm20602_accdata_spi();
    get_icm20602_gyro_spi();
    //加速度角度解算
    acc_angle=(float)atan(sqrt(pow(icm_acc_x, 2) + pow(icm_acc_y, 2))/ icm_acc_z)/pi*180;
    //简化计算  atan2(icm_acc_y,icm_acc_z)
    gyro_angle=(float)icm_gyro_x*0.061035156;

    angle = angle_calc(acc_angle, gyro_angle);
    pit_enable_interrupt(CCU6_0, PIT_CH1);
}





