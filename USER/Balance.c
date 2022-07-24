/*
 * Balance.c
 *
 *  Created on: 2022��4��1��
 *      Author: robocar
 */
#include "Balance.h"


#define pi 3.1415926

/*
//�����ʽ
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

////һ�׻����˲�
//float acc_angle;     //��ʼ���ٶȣ�������
//float gyro_angle;    //��ʼ���ٶȣ�������
//float angle=0;
//
//
//struct{
//    float acc_ratio;      //���ٶȼƱ���
//    float gyro_ratio;     //�����Ǳ���
//    float dt;             //��������
//}angle_calc_0={0.85,0.95,0.005};
//
//
//float acc_ratio=20;      //���ٶȼƱ���
//float gyro_ratio=20;     //�����Ǳ���
//float dt=0.005;             //��������
//
//
//float angle_calc(float angle_m, float gyro_m)
//{
//
//    float temp_angle;
//    float error_angle;
//    float gyro_now;
//    static float last_angle0;
//    static uint8 first_angle;
//
//    if(!first_angle)//�ж��Ƿ�Ϊ��һ�����б�����
//    {
//        //����ǵ�һ�����У����ϴνǶ�ֵ����Ϊ����ٶ�ֵһ��
//        first_angle = 1;
//        last_angle0 = angle_m;
//    }
//
//    gyro_now = gyro_m * gyro_ratio;
//    //���ݲ������ļ��ٶ�ֵת��Ϊ�Ƕ�֮�����ϴεĽǶ�ֵ��ƫ��
//    error_angle = (angle_m - last_angle0)*acc_ratio;
//    //����ƫ���������ǲ����õ��ĽǶ�ֵ���㵱ǰ�Ƕ�ֵ
//    temp_angle = last_angle0 + (error_angle + gyro_now)*dt;
//    //���浱ǰ�Ƕ�ֵ
//    last_angle0 = temp_angle;
//
//
//    return temp_angle;
//
//}
////�Ƕ�   ������
//void get_IMU_data()
//{
//    pit_disable_interrupt(CCU6_0, PIT_CH1);
//    get_icm20602_accdata_spi();
//    get_icm20602_gyro_spi();
//    //���ٶȽǶȽ���
//    acc_angle=(float)atan(sqrt(pow(icm_acc_x, 2) + pow(icm_acc_y, 2))/ icm_acc_z)/pi*180;
//    //�򻯼���  atan2(icm_acc_y,icm_acc_z)
//    gyro_angle=(float)icm_gyro_x*0.061035156;
//
//    angle = angle_calc(acc_angle, gyro_angle);
//    pit_enable_interrupt(CCU6_0, PIT_CH1);
//}
//

//б�¼��
float slide_angle = 0;
float slide_acc_angle = 18;
float slide_gyro_angle = 10;
float slide_angle_calc (float angle_m, float gyro_m);
void slide_detect ()
{
    //���ٶȽǶȽ���
    slide_acc_angle = (float) atan(icm_acc_x / sqrt(pow(icm_acc_y, 2) + pow(icm_acc_z, 2))) / pi * 180;
    //�򻯼���  atan2(icm_acc_y,icm_acc_z)
    slide_gyro_angle = -(float) icm_gyro_y * 0.061035156;
    slide_angle = slide_angle_calc(slide_acc_angle, slide_gyro_angle);
}

float slide_acc_ratio = 10;      //���ٶȼƱ���   �޿�����2.55
float slide_gyro_ratio = 48;     //�����Ǳ���  �޿�����0.985

float slide_angle_calc (float angle_m, float gyro_m)
{
    float temp_angle;
    float error_angle;
    float gyro_now;
    static float slide_last_angle0;
    static uint8 slide_first_angle;

    if (!slide_first_angle)             //�ж��Ƿ�Ϊ��һ�����б�����
    {
        //����ǵ�һ�����У����ϴνǶ�ֵ����Ϊ����ٶ�ֵһ��
        slide_first_angle = 1;
        slide_last_angle0 = angle_m;
    }

    gyro_now = gyro_m * slide_gyro_ratio;
    //���ݲ������ļ��ٶ�ֵת��Ϊ�Ƕ�֮�����ϴεĽǶ�ֵ��ƫ��
    error_angle = (angle_m - slide_last_angle0) * slide_acc_ratio;
    //����ƫ���������ǲ����õ��ĽǶ�ֵ���㵱ǰ�Ƕ�ֵ
    temp_angle = slide_last_angle0 + (error_angle + gyro_now) * 0.005;
    //���浱ǰ�Ƕ�ֵ
    slide_last_angle0 = temp_angle;

    return temp_angle;
}




