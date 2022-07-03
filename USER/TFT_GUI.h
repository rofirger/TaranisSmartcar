/*
 * TFT_GUI.h
 *
 *  Created on: 2022��2��13��
 *      Author: fengq
 */
//���
/*
 ����Ļ�ֳ�10�У�ÿ�������ʾ16λ�ַ�
 һҳ��ʾ9������
 ǰ����Ϊ�����������λΪ������ֵ
 */

#ifndef CODE_TFT_GUI_H_
#define CODE_TFT_GUI_H_
#include "headfile.h"
//��������������36ʱ��Ҫ�޸ġ�TFT_GUI.c"�ĺ궨��
#define TotalParaNumber 36
//�浱ǰ���������飬�������ⲿ����
extern double paraData[TotalParaNumber];
extern int16 control_duty;
//��ǰ�˵�������
extern uint8 location[2];
extern uint8 wifiInitOK;
//��ʼ��TFT��Ļ
void GUI_init(ERU_PIN_enum eru_pin);
//��ң�ؽ������������жϱ�����ʱ���ô˺���
void eru_triggered();
//��һ������������
void drawRectangle(uint16 x,uint16 y,uint8 length,uint8 width,uint16 color);
//ˢ������ҳ��
void refresh();
//�൱��C��׼��strcpy();
void str_cpy(char* Dest,char* Source,uint8 length);
//�����յ�ң�ؽ�����������a��b�������Ӧ�Ķ���
void received(uint8 a,uint8 b);
//��ʼ��page����
void page_init();
//page1Para_init��flash�м��ز�����ֵ
void page1ParaInit();
//page1ParaSave��flash��д�����ֵ
void page1ParaSave();
//�Բ˵��������ƣ���ֹѡ�пհ�����
void GUI_limited();
//д��ǰ�Բ������м�飬��ֹ���ִ���,ȫ����ȷ�򷵻�1
uint8 GUI_Para_Check();
//ˢ�²�����������ת��Ϊfloat
void GUI_Load_Data();
//ʹ��esp�ͺ���ʱ��ʼ����ģ��
void wifiInit (ERU_PIN_enum eru_pin);
//����ͼƬ����λ��
void sentImageToWifi(uint8 [][188]);
//����һ���ַ�������λ��
void sentCharToWifi(uint8);
void receivedWithoutGUI(uint8,uint8);
//page2����controlģʽ;
static void controlLeft();
static void controlRight();
static void controlForward();
static void controlBackward();
static void controlClear();

#endif /* CODE_TFT_GUI_H_ */
