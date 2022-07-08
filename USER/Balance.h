/*
 * Balance.h
 *
 *  Created on: 2022年4月1日
 *      Author: robocar
 */

#ifndef USER_BALANCE_H_
#define USER_BALANCE_H_
#include "headfile.h"
#include "math.h"
//初始化
extern void init(void);
//一阶互补滤波
extern float acc_angle;     //初始加速度（经处理）
extern float gyro_angle;    //初始角速度（经处理）
extern float angle;         //解算角度

extern float acc_ratio;      //加速度计比例
extern float gyro_ratio;     //陀螺仪比例
//获取陀螺仪数据并处理
void get_IMU_data(void);
float angle_calc(float angle_m, float gyro_m);


#endif /* USER_BALANCE_H_ */
