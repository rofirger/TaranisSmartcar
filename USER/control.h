/*
 * control.h
 *
 *  Created on: 2022年7月8日
 *      Author: 随风
 */

#ifndef USER_CONTROL_H_
#define USER_CONTROL_H_
#include "img_process.h"

// 舵机PD
extern PID pid_steer;
extern PosErr error_steer;
// 电机PID
extern PID pid_motor_left;
extern Error error_motor_left;
extern PID pid_motor_right;
extern Error error_motor_right;

extern PID pid_sharp_bend;
extern Error error_sharp_bend;

float PID_Increase (Error *sptr, PID *pid, float nowPoint, float targetPoint);
float PID_Pos (PosErr *sptr, PID *pid, float now_point, float target_point);

#endif /* USER_CONTROL_H_ */
