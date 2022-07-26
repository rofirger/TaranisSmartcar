/*
 * control.h
 *
 *  Created on: 2022��7��8��
 *      Author: ���
 */

#ifndef USER_CONTROL_H_
#define USER_CONTROL_H_
#include "img_process.h"
typedef struct DiffSpeed
{
        float _left_speed_factor;
        float _right_speed_factor;
}DiffSpeed;


// ���PD
extern PID pid_steer;
extern PID pid_steer_sharp;
extern PID pid_steer_in_sharp;
extern PosErr error_steer;
extern PosErr sharp_error_steer;
extern PosErr in_sharp_error_steer;

// ���PID
extern PID pid_motor_left;
extern Error error_motor_left;
extern PID pid_motor_right;
extern Error error_motor_right;

extern PID pid_sharp_bend;
extern Error error_sharp_bend;
extern int16_t LEFT_SPEED_BASE;
extern int16_t RIGHT_SPEED_BASE;

extern SlopeCal slope_cal;
extern SlopeCal sharp_slope_cal;

extern DiffSpeed diff_speed;;

typedef struct BendDeal
{
        BendType _last_bend_type;
        int32_t _bend_count;
        int32_t _out_bend_count;
}BendDeal;

// ���������ֵ,����Ӧ�ò鿴�жϺ���
extern float out_bend_threshold;
extern int32_t bend_threshold;
extern float no_bend_offset_threshold;
extern BendDeal bend_deal;
extern int16_t sharp_bend_sub_speed;

float PID_Increase (Error *sptr, PID *pid, float nowPoint, float targetPoint);
float PID_Pos (PosErr *sptr, PID *pid, float now_point, float target_point);
void SteerPidChange(float _p, float _i, float _d);
void SteerPidReset();
int16_t NonLinearProcessMidOffset(int16_t _offset);

#endif /* USER_CONTROL_H_ */
