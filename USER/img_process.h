/*
 * img_process.h
 *
 *  Created on: 2022年4月15日
 *      Author: 随风
 */

#ifndef USER_IMG_PROCESS_H_
#define USER_IMG_PROCESS_H_

#include "headfile.h"
#include <stdint.h>
//#include "TFT_GUI.h"
typedef char bool;
#define true 1
#define false 0

#define ABS(x) (((x) >= 0) ? (x) : (-(x)))
// 路况
typedef enum RoadType
{
        PREPARE_TO_OUT_CARBARN, // 准备出库 0
        OUT_CARBARN,            // 出库过程中 1
        NO_FIX_ROAD,            // 普通路况 2
        ONLY_FIX_LEFT_ROAD,     // 3
        ONLY_FIX_RIGHT_ROAD,    // 4
        CROSSROAD,              // 十字路口5

        IN_LEFT_ROTARY,                // 在左环道 6
        IN_RIGHT_ROTARY,               // 在右环道 7
        LEFT_ROTARY_IN_FIRST_SUNKEN,   // 进环道前的第一个路口 8
        LEFT_ROTARY_IN_SECOND_SUNKEN,  // 进环道前的第二个路口 9
        LEFT_ROTARY_OUT_FIRST_SUNKEN,  // 出环道后的第一个路口 10
        LEFT_ROTARY_OUT_SECOND_SUNKEN, // 出环道后的第二个路口 11

        RIGHT_ROTARY_IN_FIRST_SUNKEN,   // 进环道前的第一个路口 12
        RIGHT_ROTARY_IN_SECOND_SUNKEN,  // 进环道前的第二个路口 13
        RIGHT_ROTARY_OUT_FIRST_SUNKEN,  // 出环道后的第一个路口 14
        RIGHT_ROTARY_OUT_SECOND_SUNKEN, // 出环道后的第二个路口 15

        IN_LEFT_JUNCTION_ING,   // 进入左三叉    16
        IN_RIGHT_JUNCTION_ING,  // 进入右三叉    17
        IN_LEFT_JUNCTION_ED,    // 18
        IN_RIGHT_JUNCTION_ED,   // 19
        OUT_LEFT_JUNCTION_ING,  // 20
        OUT_RIGHT_JUNCTION_ING, // 21
        IN_CARBARN
} RoadType;

// 用于控制代码
typedef RoadType RoadTypeForControl;

typedef enum FixPointType
{
        ARC_LEFT,
        ARC_RIGHT,
        CLIFF,
        NO_DEFINE_VALUE_FIX_POINT,
        NO_TYPE
} FixPointType;
typedef struct FixPoint
{
        FixPointType fix_point_type;
        int16_t pos_col;
} FixPoint;

typedef struct Offset
{
        int16_t _offset[MT9V03X_H];
        int16_t _end_src_rows;
        int16_t _total_offset;
}Offset;

// 二次拟合曲线系数
typedef struct QuadraticCoeffic
{
        float a0;
        float a1;
        float a2;
} QuadraticCoeffic;

// 一次拟合直线系数
typedef struct StraightLineCoeffic
{
        float k;
        float a;
} StraightLineCoeffic;

// pid参数
typedef struct PID
{
        float P, I, D;
} PID;

// 增量式
typedef struct Error
{
        float currentError;   //当前误差
        float lastError;      //上一次误差
        float previoursError; //上上次误差
} Error;

// 位置式
typedef struct PosErr
{
        Error err;
        float loc_sum;
} PosErr;

// 弯道类型
typedef enum BendType
{
        NO_BEND,                 // 无弯
        LEFT_STRAIGHT_BEND,      // 应该往左拐的直道弯道
        RIGHT_STRAIGHT_BEND,     // 应该往右拐的直道弯道
        LEFT_SHARP_BEND,         // 往左拐的急转弯
        RIGHT_SHARP_BEND,        // 往右拐的急转弯
        LEFT_NORMAL_BEND,        // 往左拐的正常弯道
        RIGHT_NORMAL_BEND        // 往右拐的正常弯道
}BendType;

// 斜率计算相关
typedef struct SlopeCal
{
        float _slope;
        int16_t _start_cal_y;
        int16_t _end_cal_y;
}SlopeCal;

extern Offset mid_offset;
extern BendType bend_type;
extern SlopeCal slope_cal;

// 斜坡检测俯仰角
extern float slide_angle;

float PID_Pos(PosErr *sptr, PID *pid, float now_point, float target_point);
void GetHistGram(uint8_t width, uint8_t height);
unsigned char OTSUThreshold();
void BinaryzationProcess(int rows, int cols, unsigned int threshold_value);
float PID_Increase(Error *sptr, PID *pid, float nowPoint, float targetPoint);
void AuxiliaryProcess(uint8_t src_rows, uint8_t src_cols, unsigned char threshold_val, uint8_t *left_line,
                      uint8_t *mid_line, uint8_t *right_line);
float Sqrt(float number);
QuadraticCoeffic QuadraticCurveFit(uint8_t *mid_line_arr, uint16_t start_index, uint16_t end_index);
float CurvatureCal(uint8_t *mid_line_arr, int16_t start_index, int16_t end_index);
StraightLineCoeffic LinearRegress(uint8_t *mid_line_arr, uint16_t start_index, uint16_t end_index);
void CorrectLRLine(uint8_t *left_line, uint8_t *right_line, uint8_t src_rows, uint8_t src_cols);
void FixRoad(uint8_t *left_line, uint8_t *right_line, uint8_t src_rows, uint8_t src_cols, uint8_t end_src_rows,
             unsigned char threshold_val);
uint8_t FindStraightLine(uint8_t *mid_line, uint8_t src_rows, uint8_t src_cols);
void UserProcess(uint8_t *left_line, uint8_t *mid_line, uint8_t *right_line, uint8 src_rows, uint8_t src_cols,
                 uint8_t threshold_value, float *slope);
uint8_t FindCircle(uint8_t *line_, uint8_t start_index_, uint8_t height);

#endif /* USER_IMG_PROCESS_H_ */
