/*
 * Balance.h
 *
 *  Created on: 2022��4��1��
 *      Author: robocar
 */

#ifndef USER_BALANCE_H_
#define USER_BALANCE_H_
#include "headfile.h"
#include "math.h"
//��ʼ��
extern void init(void);
//һ�׻����˲�
extern float acc_angle;     //��ʼ���ٶȣ�������
extern float gyro_angle;    //��ʼ���ٶȣ�������
extern float angle;         //����Ƕ�

extern float acc_ratio;      //���ٶȼƱ���
extern float gyro_ratio;     //�����Ǳ���
//��ȡ���������ݲ�����
void get_IMU_data(void);
float angle_calc(float angle_m, float gyro_m);


#endif /* USER_BALANCE_H_ */
