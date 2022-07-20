/*
 * control.c
 *
 *  Created on: 2022��7��8��
 *      Author: ���
 */

#include "control.h"
#include "img_process.h"

// ���PD
PID pid_steer_standard =  {0.830, 0, 4.125};

extern PID pid_steer = {0.50, 0, 0};





PosErr error_steer = {{0, 0, 0}, 0};



// ���PID
PID pid_motor_standrd = {5.2,2,0};
PID pid_motor_left = {3, 2, 0};
Error error_motor_left = {0, 0, 0};
PID pid_motor_right = {3, 2, 0};
Error error_motor_right = {0, 0, 0};

// ��ת��PID
PID pid_sharp_bend = {0.0052, 0.00002, 0.00002};
Error error_sharp_bend = {0, 0, 0};

/*<!
 *  @brief      ����ʽPID
 *  *sptr ��������
 *  *pid:  PID����
 *  nowPoint��ʵ��ֵ
 *  targetPoint��   ����ֵ
 */
// ����ʽPID�������
float PID_Increase (Error *sptr, PID *pid, float nowPoint, float targetPoint)
{
    float increase;                                                                          //���ó���ʵ������
    sptr->currentError = targetPoint - nowPoint;                                             // ���㵱ǰ���
    increase = pid->P * (sptr->currentError - sptr->lastError)                               //����P
    + pid->I * sptr->currentError                                                 //����I
    + pid->D * (sptr->currentError - 2 * sptr->lastError + sptr->previoursError); //΢��D
    sptr->previoursError = sptr->lastError;                                                  // ����ǰ�����
    sptr->lastError = sptr->currentError;                                                    // �����ϴ����
    return increase;                                                                         // ��������
}

/*<!
 *  @brief      λ��ʽPID
 *  *sptr ��������
 *  *pid:  PID����
 *  now_point��ʵ��ֵ
 *  target_point��   ����ֵ
 */
float PID_Pos (PosErr *sptr, PID *pid, float now_point, float target_point)
{
    float pos_;                                                       // λ��
    sptr->err.currentError = target_point - now_point;                // ���㵱ǰ���
    sptr->loc_sum += sptr->err.currentError;                          // �ۼ����
    pos_ = pid->P * sptr->err.currentError                            // ����P
    + pid->I * sptr->loc_sum                                   // ����I
    + pid->D * (sptr->err.currentError - sptr->err.lastError); // ΢��D
    sptr->err.lastError = sptr->err.currentError;                     // �����ϴ����
    return pos_;
}

inline void SteerPidChange(float _p, float _i, float _d)
{
    pid_steer.P = _p;
    pid_steer.I = _i;
    pid_steer.D = _d;
}

inline void SteerPidReset()
{
    pid_steer.P = pid_steer_standard.P;
    pid_steer.I = pid_steer_standard.I;
    pid_steer.D = pid_steer_standard.D;
}
