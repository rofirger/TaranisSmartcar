/*
 * fuzzy_pid.c
 *
 *  Created on: 2022年5月26日
 *      Author: 随风
 */



#include "fuzzy_pid.h"

PID_STRUCT PID_Data;

static float E_TABLE[]={-3,-2,-1,0,1,2,3};
static float EC_TABLE[]={-3,-2,-1,0,1,2,3};
static float KpTABLE[]={0.4,0.3,0.6,1.2};
static float KiTABLE[]={0.1,0.12,0.16,0.19};
static float KdTABLE[]={0.4,0.6,0.8,1.0};
//模糊规则到时候再慢慢调
static char KpRULE[7][7]=
{
3,3,3,3,3,3,3,
2,2,2,2,1,2,2,
1,1,1,1,1,1,1,
1,1,0,1,0,1,1,
0,0,1,0,0,1,0,
0,1,0,1,0,0,2,
3,3,3,3,3,3,3
};

static char KiRULE[7][7]=
{
    0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,
    2,0,0,0,0,0,1,
    3,3,3,3,3,3,3
};

static char KdRULE[7][7]=
{
    3,3,3,2,2,2,2,
    2,2,2,1,1,1,1,
    1,1,2,1,1,2,1,
    1,1,0,1,0,1,1,
    1,1,0,0,0,1,1,
    2,2,1,0,1,1,1,
    3,3,3,3,2,3,2
};


static float fuzzy_kp(float e,float ec)
{
    char num,pe,pec;
    float eFuzzy[2]={0,0};
    float ecFuzzy[2]={0,0};
    float KpFuzzy[4]={0};
    float Kp;

//误差隶属函数
    if(e<E_TABLE[0])
    {
        eFuzzy[0]=1.0;
        pe=0;
    }
    else if(e>=E_TABLE[0]&&e<E_TABLE[1])
    {
        eFuzzy[0]=(E_TABLE[1]-e)/(E_TABLE[1]-E_TABLE[0]);
        pe=0;
    }
        else if(e>=E_TABLE[1]&&e<E_TABLE[2])
    {
        eFuzzy[0]=(E_TABLE[2]-e)/(E_TABLE[2]-E_TABLE[1]);
        pe=1;
    }
        else if(e>=E_TABLE[2]&&e<E_TABLE[3])
    {
        eFuzzy[0]=(E_TABLE[3]-e)/(E_TABLE[3]-E_TABLE[2]);
        pe=2;
    }
        else if(e>=E_TABLE[3]&&e<E_TABLE[4])
    {
        eFuzzy[0]=(E_TABLE[4]-e)/(E_TABLE[4]-E_TABLE[3]);
        pe=3;
    }
        else if(e>=E_TABLE[4]&&e<E_TABLE[5])
    {
        eFuzzy[0]=(E_TABLE[5]-e)/(E_TABLE[5]-E_TABLE[4]);
        pe=4;
    }
        else if(e>=E_TABLE[5]&&e<E_TABLE[6])
    {
        eFuzzy[0]=(E_TABLE[6]-e)/(E_TABLE[6]-E_TABLE[5]);
        pe=5;
    }
    else
    {
      eFuzzy[0]=0;
        pe=6;
    }
    eFuzzy[1]=1-eFuzzy[0];

    //误差变化率隶属函数
        if(e<EC_TABLE[0])
    {
        ecFuzzy[0]=1.0;
        pec=0;
    }
    else if(ec>=EC_TABLE[0]&&ec<EC_TABLE[1])
    {
        ecFuzzy[0]=(EC_TABLE[1]-ec)/(EC_TABLE[1]-EC_TABLE[0]);
        pec=0;
    }
        else if(ec>=EC_TABLE[1]&&ec<EC_TABLE[2])
    {
        ecFuzzy[0]=(EC_TABLE[2]-e)/(EC_TABLE[2]-EC_TABLE[1]);
        pec=1;
    }
        else if(ec>=EC_TABLE[2]&&ec<EC_TABLE[3])
    {
        ecFuzzy[0]=(EC_TABLE[3]-e)/(EC_TABLE[3]-EC_TABLE[2]);
        pec=2;
    }
        else if(ec>=EC_TABLE[3]&&ec<EC_TABLE[4])
    {
        ecFuzzy[0]=(EC_TABLE[4]-e)/(EC_TABLE[4]-EC_TABLE[3]);
        pec=3;
    }
        else if(ec>=EC_TABLE[4]&&ec<EC_TABLE[5])
    {
        ecFuzzy[0]=(EC_TABLE[5]-e)/(EC_TABLE[5]-EC_TABLE[4]);
        pec=4;
    }
        else if(ec>=EC_TABLE[5]&&ec<EC_TABLE[6])
    {
        ecFuzzy[0]=(EC_TABLE[6]-e)/(EC_TABLE[6]-EC_TABLE[5]);
        pec=5;
    }
        else
    {
      ecFuzzy[0]=0;
        pec=6;
    }
    ecFuzzy[1]=1-ecFuzzy[0];

    //查询模糊表
    num=KpRULE[pe][pec];
    KpFuzzy[num]+=eFuzzy[0]*ecFuzzy[0];
    num=KpRULE[pe][pec+1];
    KpFuzzy[num]+=eFuzzy[0]*ecFuzzy[1];
    num=KpRULE[pe+1][pec];
    KpFuzzy[num]+=eFuzzy[1]*ecFuzzy[0];
    num=KpRULE[pe+1][pec+1];
    KpFuzzy[num]+=eFuzzy[1]*ecFuzzy[1];
    Kp=KpFuzzy[0]*KpTABLE[0]+KpFuzzy[1]*KpTABLE[1]+KpFuzzy[2]*KpTABLE[2]+KpFuzzy[3]*KpTABLE[3];
    return Kp;
}


float fuzzy_ki(float e,float ec)
{
    char num,pe,pec;
    float eFuzzy[2]={0,0};
    float ecFuzzy[2]={0,0};
    float KiFuzzy[4]={0};
    float Ki;

//误差隶属函数
    if(e<E_TABLE[0])
    {
        eFuzzy[0]=1.0;
        pe=0;
    }
    else if(e>=E_TABLE[0]&&e<E_TABLE[1])
    {
        eFuzzy[0]=(E_TABLE[1]-e)/(E_TABLE[1]-E_TABLE[0]);
        pe=0;
    }
        else if(e>=E_TABLE[1]&&e<E_TABLE[2])
    {
        eFuzzy[0]=(E_TABLE[2]-e)/(E_TABLE[2]-E_TABLE[1]);
        pe=1;
    }
        else if(e>=E_TABLE[2]&&e<E_TABLE[3])
    {
        eFuzzy[0]=(E_TABLE[3]-e)/(E_TABLE[3]-E_TABLE[2]);
        pe=2;
    }
        else if(e>=E_TABLE[3]&&e<E_TABLE[4])
    {
        eFuzzy[0]=(E_TABLE[4]-e)/(E_TABLE[4]-E_TABLE[3]);
        pe=3;
    }
        else if(e>=E_TABLE[4]&&e<E_TABLE[5])
    {
        eFuzzy[0]=(E_TABLE[5]-e)/(E_TABLE[5]-E_TABLE[4]);
        pe=4;
    }
        else if(e>=E_TABLE[5]&&e<E_TABLE[6])
    {
        eFuzzy[0]=(E_TABLE[6]-e)/(E_TABLE[6]-E_TABLE[5]);
        pe=5;
    }
    else
    {
      eFuzzy[0]=0;
        pe=6;
    }
    eFuzzy[1]=1-eFuzzy[0];

    //误差变化率隶属函数
        if(e<EC_TABLE[0])
    {
        ecFuzzy[0]=1.0;
        pec=0;
    }
    else if(ec>=EC_TABLE[0]&&ec<EC_TABLE[1])
    {
        ecFuzzy[0]=(EC_TABLE[1]-ec)/(EC_TABLE[1]-EC_TABLE[0]);
        pec=0;
    }
        else if(ec>=EC_TABLE[1]&&ec<EC_TABLE[2])
    {
        ecFuzzy[0]=(EC_TABLE[2]-e)/(EC_TABLE[2]-EC_TABLE[1]);
        pec=1;
    }
        else if(ec>=EC_TABLE[2]&&ec<EC_TABLE[3])
    {
        ecFuzzy[0]=(EC_TABLE[3]-e)/(EC_TABLE[3]-EC_TABLE[2]);
        pec=2;
    }
        else if(ec>=EC_TABLE[3]&&ec<EC_TABLE[4])
    {
        ecFuzzy[0]=(EC_TABLE[4]-e)/(EC_TABLE[4]-EC_TABLE[3]);
        pec=3;
    }
        else if(ec>=EC_TABLE[4]&&ec<EC_TABLE[5])
    {
        ecFuzzy[0]=(EC_TABLE[5]-e)/(EC_TABLE[5]-EC_TABLE[4]);
        pec=4;
    }
        else if(ec>=EC_TABLE[5]&&ec<EC_TABLE[6])
    {
        ecFuzzy[0]=(EC_TABLE[6]-e)/(EC_TABLE[6]-EC_TABLE[5]);
        pec=5;
    }
        else
    {
      ecFuzzy[0]=0;
        pec=6;
    }
    ecFuzzy[1]=1-ecFuzzy[0];

    //查询模糊表
    num=KiRULE[pe][pec];
    KiFuzzy[num]+=eFuzzy[0]*ecFuzzy[0];
    num=KiRULE[pe][pec+1];
    KiFuzzy[num]+=eFuzzy[0]*ecFuzzy[1];
    num=KiRULE[pe+1][pec];
    KiFuzzy[num]+=eFuzzy[1]*ecFuzzy[0];
    num=KiRULE[pe+1][pec+1];
    KiFuzzy[num]+=eFuzzy[1]*ecFuzzy[1];
    Ki=KiFuzzy[0]*KiTABLE[0]+KiFuzzy[1]*KiTABLE[1]+KiFuzzy[2]*KiTABLE[2]+KiFuzzy[3]*KiTABLE[3];
    return Ki;
}


float fuzzy_kd(float e,float ec)
{
    char num,pe,pec;
    float eFuzzy[2]={0,0};
    float ecFuzzy[2]={0,0};
    float KdFuzzy[4]={0};
    float Kd;

//误差隶属函数
    if(e<E_TABLE[0])
    {
        eFuzzy[0]=1.0;
        pe=0;
    }
    else if(e>=E_TABLE[0]&&e<E_TABLE[1])
    {
        eFuzzy[0]=(E_TABLE[1]-e)/(E_TABLE[1]-E_TABLE[0]);
        pe=0;
    }
        else if(e>=E_TABLE[1]&&e<E_TABLE[2])
    {
        eFuzzy[0]=(E_TABLE[2]-e)/(E_TABLE[2]-E_TABLE[1]);
        pe=1;
    }
        else if(e>=E_TABLE[2]&&e<E_TABLE[3])
    {
        eFuzzy[0]=(E_TABLE[3]-e)/(E_TABLE[3]-E_TABLE[2]);
        pe=2;
    }
        else if(e>=E_TABLE[3]&&e<E_TABLE[4])
    {
        eFuzzy[0]=(E_TABLE[4]-e)/(E_TABLE[4]-E_TABLE[3]);
        pe=3;
    }
        else if(e>=E_TABLE[4]&&e<E_TABLE[5])
    {
        eFuzzy[0]=(E_TABLE[5]-e)/(E_TABLE[5]-E_TABLE[4]);
        pe=4;
    }
        else if(e>=E_TABLE[5]&&e<E_TABLE[6])
    {
        eFuzzy[0]=(E_TABLE[6]-e)/(E_TABLE[6]-E_TABLE[5]);
        pe=5;
    }
    else
    {
      eFuzzy[0]=0;
        pe=6;
    }
    eFuzzy[1]=1-eFuzzy[0];

    //误差变化率隶属函数
        if(e<EC_TABLE[0])
    {
        ecFuzzy[0]=1.0;
        pec=0;
    }
    else if(ec>=EC_TABLE[0]&&ec<EC_TABLE[1])
    {
        ecFuzzy[0]=(EC_TABLE[1]-ec)/(EC_TABLE[1]-EC_TABLE[0]);
        pec=0;
    }
        else if(ec>=EC_TABLE[1]&&ec<EC_TABLE[2])
    {
        ecFuzzy[0]=(EC_TABLE[2]-e)/(EC_TABLE[2]-EC_TABLE[1]);
        pec=1;
    }
        else if(ec>=EC_TABLE[2]&&ec<EC_TABLE[3])
    {
        ecFuzzy[0]=(EC_TABLE[3]-e)/(EC_TABLE[3]-EC_TABLE[2]);
        pec=2;
    }
        else if(ec>=EC_TABLE[3]&&ec<EC_TABLE[4])
    {
        ecFuzzy[0]=(EC_TABLE[4]-e)/(EC_TABLE[4]-EC_TABLE[3]);
        pec=3;
    }
        else if(ec>=EC_TABLE[4]&&ec<EC_TABLE[5])
    {
        ecFuzzy[0]=(EC_TABLE[5]-e)/(EC_TABLE[5]-EC_TABLE[4]);
        pec=4;
    }
        else if(ec>=EC_TABLE[5]&&ec<EC_TABLE[6])
    {
        ecFuzzy[0]=(EC_TABLE[6]-e)/(EC_TABLE[6]-EC_TABLE[5]);
        pec=5;
    }
        else
    {
      ecFuzzy[0]=0;
        pec=6;
    }
    ecFuzzy[1]=1-ecFuzzy[0];

    //查询模糊表
    num=KdRULE[pe][pec];
    KdFuzzy[num]+=eFuzzy[0]*ecFuzzy[0];
    num=KdRULE[pe][pec+1];
    KdFuzzy[num]+=eFuzzy[0]*ecFuzzy[1];
    num=KdRULE[pe+1][pec];
    KdFuzzy[num]+=eFuzzy[1]*ecFuzzy[0];
    num=KdRULE[pe+1][pec+1];
    KdFuzzy[num]+=eFuzzy[1]*ecFuzzy[1];
    Kd=KdFuzzy[0]*KdTABLE[0]+KdFuzzy[1]*KdTABLE[1]+KdFuzzy[2]*KdTABLE[2]+KdFuzzy[3]*KdTABLE[3];
    return Kd;
}

float PID_Calc(PID_STRUCT *p,float SetPoint,float Current)
{
    float Kp,Ki,Kd;
    float pErr,iErr,dErr;
    float err,ecerr,delt;

    err = SetPoint - Current;
    ecerr = err/SetPoint;
    pErr = err - p->LastErr;
    iErr = err;
    dErr = err - 2*(p->LastErr)+p->PreErr;

    Kp = fuzzy_kp(err,ecerr);
    Ki = fuzzy_ki(err,ecerr);
    Kd = fuzzy_kd(err,ecerr);

    delt = Kp*pErr + Ki*iErr + Kd*dErr;
    p->PreErr = p->LastErr;
    p->LastErr = err ;

    return (Current+delt);
}


