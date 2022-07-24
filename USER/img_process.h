/*
 * img_process.h
 *
 *  Created on: 2022��4��15��
 *      Author: ���
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
// ·��
typedef enum RoadType
{
        PREPARE_TO_OUT_CARBARN, // ׼������ 0
        OUT_CARBARN,            // ��������� 1
        NO_FIX_ROAD,            // ��ͨ·�� 2
        ONLY_FIX_LEFT_ROAD,     // 3
        ONLY_FIX_RIGHT_ROAD,    // 4
        CROSSROAD,              // ʮ��·��5

        IN_LEFT_ROTARY,                // ���󻷵� 6
        IN_RIGHT_ROTARY,               // ���һ��� 7
        LEFT_ROTARY_IN_FIRST_SUNKEN,   // ������ǰ�ĵ�һ��·�� 8
        LEFT_ROTARY_IN_SECOND_SUNKEN,  // ������ǰ�ĵڶ���·�� 9
        LEFT_ROTARY_OUT_FIRST_SUNKEN,  // ��������ĵ�һ��·�� 10
        LEFT_ROTARY_OUT_SECOND_SUNKEN, // ��������ĵڶ���·�� 11

        RIGHT_ROTARY_IN_FIRST_SUNKEN,   // ������ǰ�ĵ�һ��·�� 12
        RIGHT_ROTARY_IN_SECOND_SUNKEN,  // ������ǰ�ĵڶ���·�� 13
        RIGHT_ROTARY_OUT_FIRST_SUNKEN,  // ��������ĵ�һ��·�� 14
        RIGHT_ROTARY_OUT_SECOND_SUNKEN, // ��������ĵڶ���·�� 15

        IN_LEFT_JUNCTION_ING,   // ����������    16
        IN_RIGHT_JUNCTION_ING,  // ����������    17
        IN_LEFT_JUNCTION_ED,    // 18
        IN_RIGHT_JUNCTION_ED,   // 19
        OUT_LEFT_JUNCTION_ING,  // 20
        OUT_RIGHT_JUNCTION_ING, // 21
        IN_CARBARN
} RoadType;

// ���ڿ��ƴ���
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

// �����������ϵ��
typedef struct QuadraticCoeffic
{
        float a0;
        float a1;
        float a2;
} QuadraticCoeffic;

// һ�����ֱ��ϵ��
typedef struct StraightLineCoeffic
{
        float k;
        float a;
} StraightLineCoeffic;

// pid����
typedef struct PID
{
        float P, I, D;
} PID;

// ����ʽ
typedef struct Error
{
        float currentError;   //��ǰ���
        float lastError;      //��һ�����
        float previoursError; //���ϴ����
} Error;

// λ��ʽ
typedef struct PosErr
{
        Error err;
        float loc_sum;
} PosErr;

// �������
typedef enum BendType
{
        NO_BEND,                 // ����
        LEFT_STRAIGHT_BEND,      // Ӧ������յ�ֱ�����
        RIGHT_STRAIGHT_BEND,     // Ӧ�����ҹյ�ֱ�����
        LEFT_SHARP_BEND,         // ����յļ�ת��
        RIGHT_SHARP_BEND,        // ���ҹյļ�ת��
        LEFT_NORMAL_BEND,        // ����յ��������
        RIGHT_NORMAL_BEND        // ���ҹյ��������
}BendType;

// б�ʼ������
typedef struct SlopeCal
{
        float _slope;
        int16_t _start_cal_y;
        int16_t _end_cal_y;
}SlopeCal;

extern Offset mid_offset;
extern BendType bend_type;
extern SlopeCal slope_cal;

// б�¼�⸩����
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
