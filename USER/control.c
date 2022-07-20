/*
 * control.c
 *
 *  Created on: 2022年7月8日
 *      Author: 随风
 */

#include "control.h"
#include "img_process.h"

// 舵机PD
PID pid_steer_standard =  {0.830, 0, 4.125};

extern PID pid_steer = {0.50, 0, 0};





PosErr error_steer = {{0, 0, 0}, 0};



// 电机PID
PID pid_motor_standrd = {5.2,2,0};
PID pid_motor_left = {3, 2, 0};
Error error_motor_left = {0, 0, 0};
PID pid_motor_right = {3, 2, 0};
Error error_motor_right = {0, 0, 0};

// 急转弯PID
PID pid_sharp_bend = {0.0052, 0.00002, 0.00002};
Error error_sharp_bend = {0, 0, 0};

/*<!
 *  @brief      增量式PID
 *  *sptr ：误差参数
 *  *pid:  PID参数
 *  nowPoint：实际值
 *  targetPoint：   期望值
 */
// 增量式PID电机控制
float PID_Increase (Error *sptr, PID *pid, float nowPoint, float targetPoint)
{
    float increase;                                                                          //最后得出的实际增量
    sptr->currentError = targetPoint - nowPoint;                                             // 计算当前误差
    increase = pid->P * (sptr->currentError - sptr->lastError)                               //比例P
    + pid->I * sptr->currentError                                                 //积分I
    + pid->D * (sptr->currentError - 2 * sptr->lastError + sptr->previoursError); //微分D
    sptr->previoursError = sptr->lastError;                                                  // 更新前次误差
    sptr->lastError = sptr->currentError;                                                    // 更新上次误差
    return increase;                                                                         // 返回增量
}

/*<!
 *  @brief      位置式PID
 *  *sptr ：误差参数
 *  *pid:  PID参数
 *  now_point：实际值
 *  target_point：   期望值
 */
float PID_Pos (PosErr *sptr, PID *pid, float now_point, float target_point)
{
    float pos_;                                                       // 位置
    sptr->err.currentError = target_point - now_point;                // 计算当前误差
    sptr->loc_sum += sptr->err.currentError;                          // 累计误差
    pos_ = pid->P * sptr->err.currentError                            // 比列P
    + pid->I * sptr->loc_sum                                   // 积分I
    + pid->D * (sptr->err.currentError - sptr->err.lastError); // 微分D
    sptr->err.lastError = sptr->err.currentError;                     // 更新上次误差
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
