/*
 * fuzzy_pid.h
 *
 *  Created on: 2022��5��26��
 *      Author: ���
 */

#ifndef _FUZZYPID_H
#define _FUZZYPID_H

typedef struct
{
    float SetPoint;
    float Kp;
    float Ki;
    float Kd;
    float LastErr;
    float PreErr;
}PID_STRUCT;

float PID_Calc(PID_STRUCT *p,float SetPoint,float Current);

#endif

