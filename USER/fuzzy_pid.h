/*
 * fuzzy_pid.h
 *
 *  Created on: 2022年5月26日
 *      Author: 随风
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

