/*
 * TFT_GUI.h
 *
 *  Created on: 2022年2月13日
 *      Author: fengq
 */
//简介
/*
 将屏幕分成10行，每行最多显示16位字符
 一页显示9个变量
 前六个为变量名，后九位为变量的值
 */

#ifndef CODE_TFT_GUI_H_
#define CODE_TFT_GUI_H_
#include "headfile.h"
#include "control.h"
#include "Balance.h"
//当参数个数大于36时需要修改“TFT_GUI.c"的宏定义
#define TotalParaNumber 36
//存当前参数的数组，可以在外部调用
extern double paraData[TotalParaNumber];
extern int16 control_duty;
//当前菜单的坐标
extern uint8 location[2];
extern uint8 wifiInitOK;


//PID矩阵
extern uint8 PID_Matrix1[7][7],PID_Matrix2[7][7],PID_Matrix3[7][7];
extern float PID_Matrix4[3][4];


//初始化TFT屏幕
void GUI_init(ERU_PIN_enum eru_pin);
//当遥控接收器的引脚中断被触发时调用此函数
void eru_triggered();
//画一个空心正方形
void drawRectangle(uint16 x,uint16 y,uint8 length,uint8 width,uint16 color);
//刷新整个页面
void refresh();
//相当于C标准库strcpy();
void str_cpy(char* Dest,char* Source,uint8 length);
//当接收到遥控接收器发出的a，b后进行相应的动作
void received(uint8 a,uint8 b);
//初始化page函数
void page_init();
//page1Para_init从flash中加载参数的值
void page1ParaInit();
//page1ParaSave向flash中写入参数值
void page1ParaSave();
//对菜单进行限制，防止选中空白区域
void GUI_limited();
//写入前对参数进行检查，防止出现错误,全部正确则返回1
uint8 GUI_Para_Check();
//刷新参数，并将其转化为float
void GUI_Load_Data();
//使用esp和红外时初始化此模块
void wifiInit (ERU_PIN_enum eru_pin);
//发送图片至上位机
void sentImageToWifi(uint8 [][188]);
//发送一个字符串到上位机
void sentCharToWifi(uint8);
void receivedWithoutGUI(uint8,uint8);
//page2触发control模式;

uint8 GET_NUMBER(double, int);
void PID_Matrix1_val(uint8 *PID_Matrix11);
void PID_Matrix2_val(uint8 *PID_Matrix22);
void PID_Matrix3_val(uint8 *PID_Matrix33);
void PID_Matrix4_val(float *PID_Matrix44);


static void controlLeft();
static void controlRight();
static void controlForward();
static void controlBackward();
static void controlClear();

#endif /* CODE_TFT_GUI_H_ */
