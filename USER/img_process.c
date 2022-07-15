#include "img_process.h"
#include "control.h"
extern uint8_t src_pixel_mat[MT9V03X_H][MT9V03X_W];
extern int16_t pwm_left;
extern int16_t pwm_right;
extern short hist_gram[256];
extern bool is_right_out;
extern int16_t left_speed;
extern int16_t right_speed;

// 透视变换矩阵及与透视变换相关
float perspective_transform_mat[3][3] = {{1.2243, 7.0365, -21.1096}, {0, 9.238, -21.5936}, {0, 0.0748, 1}};
typedef struct Pos
{
    short x;
    short y;
} Pos;
Pos mid_line_perspective_transform[MT9V03X_H];

// 上位机
//#define UPPER_COMPUTER
// 下位机
#define LOWER_COMPUTER

/*
 * 二值化函数
 */
void BinaryzationProcess (int rows, int cols, unsigned int threshold_value)
{
    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            src_pixel_mat[i][j] = src_pixel_mat[i][j] > threshold_value ? 255 : 0;
        }
    }
}

// 获取图片的直方图
void GetHistGram (uint8_t width, uint8_t height)
{
    for (int i_ = 0; i_ < 256; ++i_)
    {
        hist_gram[i_] = 0;
    }
    for (int i_ = 0; i_ < height; ++i_)
    {
        for (uint8_t j_ = 0; j_ < width; ++j_)
        {
            hist_gram[src_pixel_mat[i_][j_]]++;
        }
    }
}

// 大津法获取阈值
unsigned char OTSUThreshold ()
{
    int X, Y, amount = 0;
    int pixel_back = 0, pixel_fore = 0, pixel_integral_back = 0, pixel_integral_fore = 0, pixel_integral = 0;
    double OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差;
    int pixel_min_value, pixel_max_value;
    int threshold = 0;

    for (pixel_min_value = 0; pixel_min_value < 256 && hist_gram[pixel_min_value] == 0; pixel_min_value++)
        ;
    for (pixel_max_value = 255; pixel_max_value > pixel_min_value && hist_gram[pixel_max_value] == 0; pixel_max_value--)
        ;
    if (pixel_max_value == pixel_min_value)
        return pixel_max_value; // 图像中只有一个颜色
    if (pixel_min_value + 1 == pixel_max_value)
        return pixel_min_value; // 图像中只有二个颜色

    for (Y = pixel_min_value; Y <= pixel_max_value; Y++)
        amount += hist_gram[Y]; //  像素总数

    pixel_integral = 0;
    for (Y = pixel_min_value; Y <= pixel_max_value; Y++)
        pixel_integral += hist_gram[Y] * Y;
    SigmaB = -1;
    for (Y = pixel_min_value; Y < pixel_max_value; Y++)
    {
        pixel_back = pixel_back + hist_gram[Y];
        pixel_fore = amount - pixel_back;
        OmegaBack = (double) pixel_back / amount;
        OmegaFore = (double) pixel_fore / amount;
        pixel_integral_back += hist_gram[Y] * Y;
        pixel_integral_fore = pixel_integral - pixel_integral_back;
        MicroBack = (double) pixel_integral_back / pixel_back;
        MicroFore = (double) pixel_integral_fore / pixel_fore;
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);
        if (Sigma > SigmaB)
        {
            SigmaB = Sigma;
            threshold = Y;
        }
    }
    return threshold;
}

#ifdef UPPER_COMPUTER
unsigned char **AuxiliaryProcess(unsigned char **src_pixel_mat, size_t src_rows, size_t src_cols, unsigned char threshold_val, size_t *left_line, size_t *mid_line, size_t *right_line)
#endif // UPPER_COMPUTER
#ifdef LOWER_COMPUTER
void AuxiliaryProcess (uint8_t src_rows, uint8_t src_cols, unsigned char threshold_val, uint8_t *left_line,
        uint8_t *mid_line, uint8_t *right_line)
#endif // LOWER_COMPUTER
{
#ifdef UPPER_COMPUTER
    size_t mid_point = src_cols >> 1;
    size_t left_right_miss_point = 0;
#endif // UPPER_COMPUTER
#ifdef LOWER_COMPUTER
    uint8_t mid_point = src_cols >> 1;
    uint8_t left_right_miss_point = 0;
    int16_t min_dist = src_cols;
#endif // LOWER_COMPUTER
    for (int i = src_rows - 1; i >= 0; --i)
    {
        if (src_pixel_mat[src_rows - 1][mid_point] < threshold_val)
        {
            // 重新寻找扫线中点
            for (int16_t j = mid_point - 40; j < mid_point + 40; j++)
            {
                if (src_pixel_mat[src_rows - 1][j] > threshold_val)
                {
                    mid_point = j;
                }
            }
        }

#ifdef UPPER_COMPUTER
        size_t cur_point = mid_point;
#endif // UPPER_COMPUTER
#ifdef LOWER_COMPUTER
        uint8_t cur_point = mid_point;
#endif // LOWER_COMPUTER
        // 扫描左线
        while (cur_point - 2 > 0)
        {
            left_line[i] = 0;
            if (src_pixel_mat[i][cur_point] < threshold_val && src_pixel_mat[i][cur_point - 1] < threshold_val
                    && src_pixel_mat[i][cur_point - 2] < threshold_val)
            {
                left_line[i] = cur_point;
                break;
            }
            --cur_point;
        }
        // 扫描右线
        cur_point = mid_point;
        while (cur_point + 2 < src_cols)
        {
            right_line[i] = src_cols - 1;
            if (src_pixel_mat[i][cur_point] < threshold_val && src_pixel_mat[i][cur_point + 1] < threshold_val
                    && src_pixel_mat[i][cur_point + 2] < threshold_val)
            {
                right_line[i] = cur_point;
                break;
            }
            ++cur_point;
        }

        // 先判断一边几乎全白，另一边存在尖角的情况
        bool _is_one_no_boundary_one_have_start_len_wise = false;
        uint8_t _one_no_boundary_one_have_start_len_wise_min_x = 0;
        uint8_t _one_no_boundary_one_have_start_len_wise_max_x = 0;
        // 左边全白情况
        //        if (i < src_rows - 1 && (int16_t)left_line[i] - (int16_t)left_line[i + 1] > 10)
        //        {
        //            // 统计可能全白部分
        //            //lcd_showuint16(0, 4, 1);
        //            uint16_t _num_no_left_boundary = 0;
        //            for (int16_t _j = i + 1; _j < src_rows; ++_j)
        //            {
        //                if (left_line[_j] == 0)
        //                {
        //                    _num_no_left_boundary++;
        //                }
        //            }
        //            lcd_showuint16(0, 4, _num_no_left_boundary);
        //            // 查看右线是否符合
        //            if (_num_no_left_boundary > (src_rows - i - 5))
        //            {
        //
        //                uint16_t _num_no_right_boundary = 0;
        //                int min_y_x = right_line[i];
        //                for (int16_t _j = i + 1; _j < src_rows; ++_j)
        //                {
        //                    if (right_line[_j] == src_cols - 1)
        //                    {
        //                        _num_no_right_boundary++;
        //                    }
        //                    if (min_y_x > right_line[_j])
        //                    {
        //                        min_y_x = right_line[_j];
        //                    }
        //                }
        //                if (_num_no_right_boundary < (src_rows - i) / 2 && (int)right_line[i] - min_y_x  > 10 && (int)right_line[src_rows - 1] - min_y_x > 10)
        //                {
        //                    _is_one_no_boundary_one_have_start_len_wise = true;
        //                    _one_no_boundary_one_have_start_len_wise_min_x = 0;
        //                    _one_no_boundary_one_have_start_len_wise_max_x = min_y_x;
        //                }
        //            }
        //        }
        //        // 右边全白情况
        //        if (i < src_rows - 1 && (int16_t)right_line[i + 1] - (int16_t)right_line[i] > 10)
        //        {
        //            // 统计可能全白部分
        //            uint16_t _num_no_right_boundary = 0;
        //            for (int16_t _j = i + 1; _j < src_rows; ++_j)
        //            {
        //                if (right_line[_j] == src_cols - 1)
        //                {
        //                    _num_no_right_boundary++;
        //                }
        //            }
        //            // 查看右线是否符合
        //            if (_num_no_right_boundary > (src_rows - i - 5))
        //            {
        //                uint16_t _num_no_left_boundary = 0;
        //                uint8_t max_y_x = left_line[i];
        //                for (int16_t _j = i + 1; _j < src_rows; ++_j)
        //                {
        //                    if (left_line[_j] == 0)
        //                    {
        //                        _num_no_left_boundary++;
        //                    }
        //                    if (max_y_x < left_line[_j])
        //                    {
        //                        max_y_x = left_line[_j];
        //                    }
        //                }
        //                if (_num_no_left_boundary < (src_rows - i) / 2 && (int)max_y_x - left_line[i] > 10 && (int)max_y_x - left_line[src_rows - 1] > 10)
        //                {
        //                    _is_one_no_boundary_one_have_start_len_wise = true;
        //                    _one_no_boundary_one_have_start_len_wise_min_x = max_y_x;
        //                    _one_no_boundary_one_have_start_len_wise_max_x = src_cols - 1;
        //                }
        //            }
        //        }

        if ((((right_line[i] - (int16_t) left_line[i] > min_dist + min_dist / 4)
                || (i < src_rows / 2 && right_line[i] - (int16_t) left_line[i] > src_cols - 20))
                && left_right_miss_point == 0) || (_is_one_no_boundary_one_have_start_len_wise))
        {
            left_right_miss_point = i + 2;
            uint8_t now_min = src_rows - 1;
            uint8_t now_min_col = src_cols >> 1;
            uint8_t begine_fine_point = 42;
            uint8_t end_find_point = src_cols - begine_fine_point;
            if (_is_one_no_boundary_one_have_start_len_wise)
            {
                begine_fine_point = _one_no_boundary_one_have_start_len_wise_min_x;
                end_find_point = _one_no_boundary_one_have_start_len_wise_max_x;
                lcd_showuint16(0, 5, _one_no_boundary_one_have_start_len_wise_min_x);
                lcd_showuint16(6, 5, _one_no_boundary_one_have_start_len_wise_max_x);
            }

            // 开启纵向寻线
            for (int j = begine_fine_point; j < end_find_point; ++j)
            {
                for (int k = left_right_miss_point; k > 0; --k)
                {
                    if (src_pixel_mat[k][j] > threshold_val && src_pixel_mat[k - 1][j] > threshold_val)
                    {
                        if (now_min > k)
                        {
                            now_min = k;
                            now_min_col = j;
                        }
                    }
                    else
                        break;
                }
            }
            mid_point = now_min_col;
            for (int j = now_min; j < left_right_miss_point; ++j)
            {
                cur_point = now_min_col;
                while (cur_point - 2 > 0)
                {
                    left_line[j] = 0;
                    if (src_pixel_mat[j][cur_point] < threshold_val && src_pixel_mat[j][cur_point - 1] < threshold_val
                            && src_pixel_mat[j][cur_point - 2] < threshold_val)
                    {
                        left_line[j] = cur_point;
                        break;
                    }
                    --cur_point;
                }
                // 扫描右线
                cur_point = now_min_col;
                while (cur_point + 2 < src_cols)
                {
                    right_line[j] = src_cols - 1;
                    if (src_pixel_mat[j][cur_point] < threshold_val && src_pixel_mat[j][cur_point + 1] < threshold_val
                            && src_pixel_mat[j][cur_point + 2] < threshold_val)
                    {
                        right_line[j] = cur_point;
                        break;
                    }
                    ++cur_point;
                }
                mid_line[j] = (right_line[j] + left_line[j]) >> 1;
                mid_point = mid_line[j];
            }
            i = now_min;
        }

        mid_line[i] = (right_line[i] + left_line[i]) >> 1;
        if (left_line[i] != 0 || right_line[i] != src_cols - 1)
        {
            mid_point = mid_line[i];
        }
        else
        {
            mid_point = src_cols >> 1;
        }
        int16_t temp_mid_dist = right_line[i] - left_line[i];
        if (temp_mid_dist > 0 && temp_mid_dist < min_dist)
        {
            min_dist = temp_mid_dist;
        }
    }
}

/* 范围检测 */
#ifdef UPPER_COMPUTER
#define CHECK_RANGE(_data_for_check, _min, _max, _msg_out) \
    if (_data_for_check < _min || _data_for_check > _max)  \
    {                                                      \
        _msg_out += "error: find range error!";            \
    }
#endif // UPPER_COMPUTER

float Sqrt (float number)
{
    long i;
    float x, y;
    const float f = 1.5F;

    x = number * 0.5F;
    y = number;
    i = *(long*) &y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*) &i;
    y = y * (f - (x * y * y));
    y = y * (f - (x * y * y));
    return number * y;
}
// ΣXi^n
#ifdef UPPER_COMPUTER
int32_t SumNPowX(size_t *arr, uint8_t start_index, uint8_t end_index, uint8_t pow_n)
#endif // UPPER_COMPUTER
#ifdef LOWER_COMPUTER
int32_t SumNPowX (uint8_t *arr, uint8_t start_index, uint8_t end_index, uint8_t pow_n)
#endif // LOWER_COMPUTER
{
    int32_t ret = 0;
    for (uint8_t i = start_index; i <= end_index; ++i)
    {
        ret += pow(arr[i], pow_n);
    }
    return ret;
}
// ΣYi^n
#ifdef UPPER_COMPUTER
int32_t SumNPowY(size_t *arr, uint8_t start_index, uint8_t end_index, uint8_t pow_n)
#endif // UPPER_COMPUTER
#ifdef LOWER_COMPUTER
int32_t SumNPowY (uint8_t *arr, uint8_t start_index, uint8_t end_index, uint8_t pow_n)
#endif // LOWER_COMPUTER
{
    int32_t ret = 0;
    for (uint8_t i = start_index; i <= end_index; ++i)
    {
        ret += pow(i, pow_n);
    }
    return ret;
}
// ΣXi^n*Yi
#ifdef UPPER_COMPUTER
int32_t SumXNPowY(size_t *arr, uint8_t start_index, uint8_t end_index, uint8_t pow_n)
#endif // UPPER_COMPUTER
#ifdef LOWER_COMPUTER
int32_t SumXNPowY (uint8_t *arr, uint8_t start_index, uint8_t end_index, uint8_t pow_n)
#endif // LOWER_COMPUTER
{
    int32_t ret = 0;
    for (uint8_t i = start_index; i <= end_index; ++i)
    {
        ret += (pow(arr[i], pow_n) * i);
    }
    return ret;
}
// ΣYi^n*Xi
#ifdef UPPER_COMPUTER
int32_t SumYNPowX(size_t *arr, uint8_t start_index, uint8_t end_index, uint8_t pow_n)
#endif // UPPER_COMPUTER
#ifdef LOWER_COMPUTER
int32_t SumYNPowX (uint8_t *arr, uint8_t start_index, uint8_t end_index, uint8_t pow_n)
#endif // LOWER_COMPUTER
{
    int32_t ret = 0;
    for (uint8_t i = start_index; i <= end_index; ++i)
    {
        ret += (pow(i, pow_n) * arr[i]);
    }
    return ret;
}
// 二次曲线拟合, x 与 y 互换
#ifdef UPPER_COMPUTER
QuadraticCoeffic QuadraticCurveFit(size_t *mid_line_arr, uint16_t start_index, uint16_t end_index)
#endif // UPPER_COMPUTER
#ifdef LOWER_COMPUTER
QuadraticCoeffic QuadraticCurveFit (uint8_t *mid_line_arr, uint16_t start_index, uint16_t end_index)
#endif // LOWER_COMPUTER
{
    int32_t sum_y_1 = SumNPowY(mid_line_arr, start_index, end_index, 1);
    int32_t sum_y_2 = SumNPowY(mid_line_arr, start_index, end_index, 2);
    int32_t sum_y_3 = SumNPowY(mid_line_arr, start_index, end_index, 3);
    int32_t sum_y_4 = SumNPowY(mid_line_arr, start_index, end_index, 4);
    int32_t sum_x_1 = SumNPowX(mid_line_arr, start_index, end_index, 1);
    int32_t sum_y_1_x = SumYNPowX(mid_line_arr, start_index, end_index, 1);
    int32_t sum_y_2_x = SumYNPowX(mid_line_arr, start_index, end_index, 2);
    uint16_t num_elem = end_index - start_index + 1;
    float k = (float) sum_y_1 / (float) num_elem;
    float k_1 = (float) sum_y_2 / (float) num_elem;
    float k_2 = (float) (sum_y_3 - k_1 * sum_y_1) / (float) (sum_y_2 - k * sum_y_1);
    QuadraticCoeffic ret;
    ret.a2 = (float) (sum_y_2_x - k_1 * sum_x_1 - k_2 * (sum_y_1_x - k * sum_x_1))
            / (float) ((sum_y_4 - k_1 * sum_y_2) - k_2 * (sum_y_3 - k * sum_y_2));
    ret.a1 = (float) (sum_y_1_x - k * sum_x_1 - ret.a2 * (sum_y_3 - k * sum_y_2)) / (float) (sum_y_2 - k * sum_y_1);
    ret.a0 = (float) (sum_x_1 - sum_y_2 * ret.a2 - sum_y_1 * ret.a1) / (float) num_elem;
    return ret;
}

// 计算曲率, x 与 y 不互换
#ifdef UPPER_COMPUTER
float CurvatureCal(size_t *mid_line_arr, int16_t start_index, int16_t end_index)
#endif // UPPER_COMPUTER
#ifdef LOWER_COMPUTER
float CurvatureCal (uint8_t *mid_line_arr, int16_t start_index, int16_t end_index)
#endif // LOWER_COMPUTER
{
    int16_t mid_x = ((mid_line_arr[end_index] + mid_line_arr[start_index]) >> 1);
    int16_t mid_y = ((end_index + start_index) >> 1);
    float l1 = Sqrt(pow(mid_line_arr[end_index] - mid_line_arr[start_index], 2) + pow(end_index - start_index, 2));
    float l2 = Sqrt(pow(mid_x - mid_line_arr[start_index], 2) + pow(mid_y - start_index, 2));
    float l3 = Sqrt(pow(mid_line_arr[end_index] - mid_x, 2) + pow(end_index - mid_y, 2));
    return ABS(
            (mid_x - (int )mid_line_arr[start_index]) * (end_index - start_index)
                    - ((int )mid_line_arr[end_index] - (int )mid_line_arr[start_index]) * (mid_y - start_index))
            * 2 / (l1 * l2 * l3);
}

// 一次直线拟合 x 与 y 互换
#ifdef UPPER_COMPUTER
StraightLineCoeffic LinearRegress(size_t *mid_line_arr, uint16_t start_index, uint16_t end_index)
#endif // UPPER_COMPUTER
#ifdef LOWER_COMPUTER
StraightLineCoeffic LinearRegress (uint8_t *mid_line_arr, uint16_t start_index, uint16_t end_index)
#endif // LOWER_COMPUTER
{
    if (end_index < start_index)
    {
        StraightLineCoeffic ret;
        ret.k = 0;
        ret.a = 0;
        return ret;
    }
    uint16_t num_elem = end_index - start_index + 1;
    int32_t sum_y2 = SumNPowY(mid_line_arr, start_index, end_index, 2);
    int32_t sum_y1 = SumNPowY(mid_line_arr, start_index, end_index, 1);
    int32_t sum_x1 = SumNPowX(mid_line_arr, start_index, end_index, 1);
    int32_t sum_y_x = SumYNPowX(mid_line_arr, start_index, end_index, 1);
    int32_t a = num_elem * sum_y2 - sum_y1 * sum_y1;
    float a0 = (float) (sum_x1 * sum_y2 - sum_y_x * sum_y1) / (float) a;
    float a1 = (float) (num_elem * sum_y_x - sum_y1 * sum_x1) / (float) a;
    StraightLineCoeffic ret;
    ret.k = a1;
    ret.a = a0;
    return ret;
}
// 修正左右线噪点
#ifdef UPPER_COMPUTER
void CorrectLRLine(size_t *left_line, size_t *right_line, size_t src_rows, size_t src_cols)
#endif // UPPER_COMPUTER
#ifdef LOWER_COMPUTER
void CorrectLRLine (uint8_t *left_line, uint8_t *right_line, uint8_t src_rows, uint8_t src_cols)
#endif // LOWER_COMPUTER
{
    for (uint8_t i = 1; i < src_rows - 1; ++i)
    {
        if ((left_line[i - 1] > left_line[i] && left_line[i + 1] > left_line[i])
                || (left_line[i - 1] < left_line[i] && left_line[i + 1] < left_line[i]))
        {
            left_line[i] = ((left_line[i - 1] + left_line[i + 1]) >> 1);
        }
        if ((right_line[i - 1] > right_line[i] && right_line[i + 1] > right_line[i])
                || (right_line[i - 1] < right_line[i] && right_line[i + 1] < right_line[i]))
        {
            right_line[i] = ((right_line[i - 1] + right_line[i + 1]) >> 1);
        }
    }
}

// 在遇到一边赛道大幅度凹陷时，可以通过检测另一边是否也存在大幅度凹陷，如果存在则可认为遇到十字路口; 否则，使用霍夫变换检测不凹陷一侧是否为直线，若为直线则可认为遇到环道。

RoadType road_type = NO_FIX_ROAD;
// 用于运动控制代码的处理
RoadTypeForControl road_type_for_control = NO_FIX_ROAD;

#define NO_DEFINE_VALUE_FIX_POINT MT9V03X_H
bool is_stop = true;
uint8_t go_in_rotary_stage_left = 0;
uint8_t go_in_rotary_stage_right = 0;
#ifdef UPPER_COMPUTER
void FixRoad(unsigned char **src_pixel_mat, std::string &output, size_t *left_line, size_t *right_line, size_t src_rows, size_t src_cols, uint8_t end_src_rows, unsigned char threshold_val)
#endif // UPPER_COMPUTER
#ifdef LOWER_COMPUTER
void FixRoad (uint8_t *left_line, uint8_t *right_line, uint8_t src_rows, uint8_t src_cols, uint8_t end_src_rows,
        unsigned char threshold_val)
#endif // LOWER_COMPUTER
{
#define NO_DEFINE_VALUE src_rows
    if (road_type == PREPARE_TO_OUT_CARBARN)
    {
        int16_t left_consecutive_point_offset[69];
        int16_t right_consecutive_point_offset[69];
        for (int16_t j = src_rows - 2; j >= 0; --j)
        {
            left_consecutive_point_offset[j] = left_line[j + 1] - left_line[j];
            right_consecutive_point_offset[j] = right_line[j + 1] - right_line[j];
        }
        int16_t i_find_out = src_rows - 1;
        for (i_find_out; i_find_out >= 0; --i_find_out)
        {
            // if (ABS(right_line[i_find_out] - (int16_t)left_line[i_find_out]) < 10)
            // {
            //  break;
            // }
            if (ABS(left_consecutive_point_offset[i_find_out])
                    > 20|| ABS(right_consecutive_point_offset[i_find_out] > 20))
            {
                break;
            }
        }
        if (i_find_out > (src_rows >> 1) + 18)
        {
            road_type = OUT_CARBARN;
        }
        for (uint8_t i = 0; i < src_rows - 1; i++)
        {
            left_line[i] = 20;
            right_line[i] = src_cols - 20;
        }
        return;
    }
    // 寻找终点线
    if (road_type != PREPARE_TO_OUT_CARBARN || road_type != OUT_CARBARN)
    {
        uint8_t half_of_rows = (src_rows >> 1) - 6;
        uint8_t eight_of_one_cols = (src_cols >> 3);
        uint8_t eight_of_seven_cols = (src_cols - eight_of_one_cols);
        for (int16_t i_find_end_line = src_rows - 1; i_find_end_line > half_of_rows; --i_find_end_line)
        {
            bool is_find_black = false;
            uint8_t num_of_black = 0;
            uint8_t total_num_black = 0;
            for (int16_t j_find_end_line = eight_of_one_cols; j_find_end_line < eight_of_seven_cols; ++j_find_end_line)
            {
                if (src_pixel_mat[i_find_end_line][j_find_end_line] == 0)
                {
                    is_find_black = true;
                    num_of_black++;
                }
                else
                {
                    if (is_find_black && num_of_black > 3 && num_of_black < 11)
                    {
                        total_num_black++;
                    }
                    is_find_black = false;
                    num_of_black = 0;
                }
                if (total_num_black > 5)
                {
                    is_stop = true;
                    road_type = IN_CARBARN;
                }
            }
        }
    }

    FixPoint fix_left_head;
    FixPoint fix_left_tail;
    FixPoint fix_right_head;
    FixPoint fix_right_tail;
    fix_left_head.fix_point_type = NO_TYPE;
    fix_left_head.pos_col = NO_DEFINE_VALUE;
    fix_left_tail.fix_point_type = NO_TYPE;
    fix_left_tail.pos_col = NO_DEFINE_VALUE;
    fix_right_head.fix_point_type = NO_TYPE;
    fix_right_head.pos_col = NO_DEFINE_VALUE;
    fix_right_tail.fix_point_type = NO_TYPE;
    fix_right_tail.pos_col = NO_DEFINE_VALUE;

    // 对于特殊路段，重新以新的方案找边界线
    if (road_type == LEFT_ROTARY_IN_FIRST_SUNKEN || road_type == RIGHT_ROTARY_IN_FIRST_SUNKEN
            || road_type == LEFT_ROTARY_IN_SECOND_SUNKEN || road_type == RIGHT_ROTARY_IN_SECOND_SUNKEN
            || road_type == IN_LEFT_T_ING || road_type == IN_RIGHT_T_ING || road_type == OUT_LEFT_ING
            || road_type == OUT_RIGHT_ING || road_type == IN_LEFT_T_ED || road_type == IN_RIGHT_T_ED)
    {
        int16_t mid_point = (right_line[src_rows - 1] + left_line[src_rows - 1]) / 2;
        for (int16_t i = src_rows - 2; i >= 0; --i)
        {
            size_t cur_point = mid_point;
            // 扫描左线
            while (cur_point - 2 > 0)
            {
                left_line[i] = 0;
                if (src_pixel_mat[i][cur_point] < threshold_val && src_pixel_mat[i][cur_point - 1] < threshold_val
                        && src_pixel_mat[i][cur_point - 2] < threshold_val)
                {
                    left_line[i] = cur_point;
                    break;
                }
                --cur_point;
            }
            // 扫描右线
            cur_point = mid_point;
            while (cur_point + 2 < src_cols)
            {
                right_line[i] = src_cols - 1;
                if (src_pixel_mat[i][cur_point] < threshold_val && src_pixel_mat[i][cur_point + 1] < threshold_val
                        && src_pixel_mat[i][cur_point + 2] < threshold_val)
                {
                    right_line[i] = cur_point;
                    break;
                }
                ++cur_point;
            }
            if (road_type == LEFT_ROTARY_IN_FIRST_SUNKEN
                    || (road_type == LEFT_ROTARY_IN_SECOND_SUNKEN && go_in_rotary_stage_left == 0)
                    || (road_type == RIGHT_ROTARY_IN_SECOND_SUNKEN && go_in_rotary_stage_right == 1)
                    || road_type == IN_LEFT_T_ING || road_type == OUT_LEFT_ING || road_type == IN_LEFT_T_ED)
            {
                mid_point = right_line[i] - 15;
            }
            else if (road_type == RIGHT_ROTARY_IN_FIRST_SUNKEN
                    || (road_type == RIGHT_ROTARY_IN_SECOND_SUNKEN && go_in_rotary_stage_right == 0)
                    || (road_type == LEFT_ROTARY_IN_SECOND_SUNKEN && go_in_rotary_stage_left == 1)
                    || road_type == IN_RIGHT_T_ING || road_type == OUT_RIGHT_ING || road_type == IN_RIGHT_T_ED)
            {
                mid_point = left_line[i] + 15;
            }
        }
    }
    // 左右线相邻点点差, 从靠近车头方向开始！！
    int16_t left_consecutive_point_offset[69];
    int16_t right_consecutive_point_offset[69];
    for (int16_t j = src_rows - 2; j >= 0; --j)
    {
        left_consecutive_point_offset[j] = left_line[j + 1] - left_line[j];
        right_consecutive_point_offset[j] = right_line[j + 1] - right_line[j];
        // 降噪
        //        if (j < src_rows - 4)
        //        {
        //            if (left_consecutive_point_offset[j + 1] * left_consecutive_point_offset[j + 2] < 0
        //                    && left_consecutive_point_offset[j + 1] * left_consecutive_point_offset[j] >0)
        //            {
        //                left_line[j + 1] = (left_line[j + 2] + left_line[j]) / 2;
        //                left_consecutive_point_offset[j + 1] = left_line[j + 2] - left_line[j + 1];
        //                left_consecutive_point_offset[j] = left_line[j + 1] - left_line[j];
        //            }
        //            if (right_consecutive_point_offset[j + 1] * right_consecutive_point_offset[j + 2] < 0
        //                    && right_consecutive_point_offset[j + 1] * right_consecutive_point_offset[j] )
        //            {
        //                right_line[j + 1] = (right_line[j + 2] + right_line[j]) / 2;
        //                right_consecutive_point_offset[j + 1] = right_line[j + 2] - right_line[j + 1];
        //                right_consecutive_point_offset[j] = right_line[j + 1] - right_line[j];
        //            }
        //        }
    }

    // 重新找end_src_rows
    for (int16_t i = src_rows - 1; i >= 0; --i)
    {
        if (left_line[i] == right_line[i])
        {
            end_src_rows = i + 1;
            break;
        }
    }
    // 一些常数
    const static int16_t LEFT_LINE_HEAD_OFFSET_THRESHOLD_NORMAL = 11;
    const static int16_t LEFT_LINE_TAIL_OFFSET_THRESHOLD_NORMAL = -11;

    const static int16_t RIGHT_LINE_HEAD_OFFSET_THRESHOLD_NORMAL = -11;
    const static int16_t RIGHT_LINE_TAIL_OFFSET_THRESHOLD_NORMAL = 11;
    // 左、右线生效点（小于或等于的点视为有效点）
    int8_t left_valid_point = src_rows - 5;
    int8_t right_valid_point = src_rows - 5;
    if (road_type != IN_LEFT_ROTARY || road_type != IN_RIGHT_ROTARY)
    {
        for (int i = src_rows - 5; i >= 0 && right_line[i] - (int16_t) left_line[i] >= 10; --i)
        {
            // 左线
            if (fix_left_head.pos_col == NO_DEFINE_VALUE)
            {
                // 此情况下处在某个元素之中，不做具体元素判别
                if (left_line[i + 1] == 0 && left_line[i + 2] == 0 && left_line[i + 3] == 0 && i > src_rows - 10) // 左线不可见时[暂时]视为遇到在某个特殊元素中
                {
                    uint8_t num_little_offset = 0;
                    bool is_continue_little_offset = true;
                    for (int16_t j = i; j >= 5; --j)
                    {
                        if (ABS(left_consecutive_point_offset[j]) > 3)
                        {
                            is_continue_little_offset = false;
                            break;
                        }
                        else if (ABS(left_consecutive_point_offset[j]) < 4 && left_consecutive_point_offset[j] != 0)
                        {
                            num_little_offset++;
                            if (num_little_offset > 4)
                                break;
                        }
                    }

                    if (!is_continue_little_offset)
                    {
                        fix_left_head.pos_col = i + 3;
                        fix_left_head.fix_point_type = NO_DEFINE_VALUE_FIX_POINT;
                        if (road_type == LEFT_ROTARY_IN_FIRST_SUNKEN)
                        {
                            // 往后找最大点
                            int _max = 0;
                            int _max_col = i;
                            for (int16_t j = i; j >= end_src_rows; --j)
                            {
                                if (left_line[j] > _max)
                                {
                                    _max = left_line[j];
                                    _max_col = j;
                                }
                                if (_max > left_line[j] + 10)
                                    break;
                            }
                            fix_left_tail.fix_point_type = ARC_LEFT;
                            fix_left_tail.pos_col = _max_col;
                        }
                        else
                        {
                            // 如果不处于环道第一状态，找悬崖点
                            bool is_meet_left_cliff_threshold_offset = false;
                            int16_t _j_find_cliff = i;
                            for (_j_find_cliff; _j_find_cliff >= end_src_rows; --_j_find_cliff)
                            {
                                if (left_consecutive_point_offset[_j_find_cliff]
                                        < LEFT_LINE_TAIL_OFFSET_THRESHOLD_NORMAL - 6)
                                {
                                    is_meet_left_cliff_threshold_offset = true;
                                    break;
                                }
                            }
                            if (is_meet_left_cliff_threshold_offset)
                            {
                                // 接下来找符合作为悬崖点的点
                                for (_j_find_cliff; _j_find_cliff >= end_src_rows; --_j_find_cliff)
                                {
                                    if (ABS(left_consecutive_point_offset[_j_find_cliff]) < 2)
                                    {
                                        fix_left_tail.fix_point_type = CLIFF;
                                        fix_left_tail.pos_col = _j_find_cliff;
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }
                else if (left_consecutive_point_offset[i] > LEFT_LINE_HEAD_OFFSET_THRESHOLD_NORMAL) // 落差临界点，深入扫描确认是否为悬崖或者圆弧
                {

                    // 回溯寻找落差小于3的两个以上的点
                    int16_t left_head_time = NO_DEFINE_VALUE; // 暂时补线头点
                    int16_t left_tail_time = NO_DEFINE_VALUE; // 暂时补线尾点

                    // 先判断是否是圆弧
                    uint8_t _num_positive_offset = 0;
                    int _j_find_arc_max = i;
                    for (_j_find_arc_max; _j_find_arc_max < src_rows - 3; ++_j_find_arc_max)
                    {
                        if (left_consecutive_point_offset[_j_find_arc_max] < 0)
                            break;
                        if (left_consecutive_point_offset[_j_find_arc_max] > 0)
                            _num_positive_offset++;
                    }
                    if (_num_positive_offset > 7)
                    {
                        fix_left_head.fix_point_type = ARC_LEFT;
                        fix_left_head.pos_col = _j_find_arc_max;
                    }
                    //              以下是 不是圆弧
                    else
                    {
                        for (int16_t k_ = i; k_ < i + 6 && k_ < src_rows - 3; ++k_) // 必须能在往回的五个点内找到，否则视为不存在.后一个条件是为了避免溢出
                        {
                            if (ABS(left_consecutive_point_offset[k_]) < 3
                                    && ABS(left_consecutive_point_offset[k_ + 1]) < 3)
                            {
                                left_head_time = k_ + 1;
                                break; // 找到并返回
                            }
                        }
                        if (left_head_time != NO_DEFINE_VALUE) // 成功找到暂时的补线头点
                        {
                            uint8_t train_ele_num = 5;
                            int16_t k_ = i - 1;
                            for (k_; k_ >= train_ele_num; k_--)
                            {
                                int16_t sum_train_ele = 0;
                                for (int16_t _t = k_; _t > k_ - train_ele_num; --_t)
                                {
                                    sum_train_ele += left_consecutive_point_offset[_t];
                                }
                                if (ABS(sum_train_ele) < 7
                                        && left_line[i + 1] - (int16_t) left_line[k_ - train_ele_num / 2] > 10)
                                {
                                    fix_left_head.pos_col = i + 1;
                                    fix_left_head.fix_point_type = CLIFF;
                                    break;
                                }
                                if (sum_train_ele < -12)
                                {
                                    break;
                                }
                            }
                            // 向前寻找
                            if (fix_left_head.pos_col != NO_DEFINE_VALUE) // 补线头点被成功找到，接下来寻找补线尾点
                            {

                                // 先判断是否是悬崖点
                                bool _is_meet_threshold_offset = false;
                                for (k_ = k_ - 1; k_ > 2; --k_)
                                {
                                    if (left_consecutive_point_offset[k_] < LEFT_LINE_TAIL_OFFSET_THRESHOLD_NORMAL)
                                    {
                                        _is_meet_threshold_offset = true;
                                        break;
                                    }
                                }
                                if (_is_meet_threshold_offset && k_ - 5 > 0)
                                {
                                    // 往后几个点找最后的大落差
                                    int16_t _point_threshold = k_;
                                    for (int16_t j = k_; j > k_ - 5; --j)
                                    {
                                        if (left_consecutive_point_offset[j] < LEFT_LINE_TAIL_OFFSET_THRESHOLD_NORMAL)
                                        {
                                            _point_threshold = j;
                                        }
                                    }
                                    if (_point_threshold > 6)
                                    {
                                        // 如果是悬崖，往后几个几个点的平均落差必须小于2,而且落差均为0或者负数
                                        float _total_offset = 0;
                                        bool _is_continue_polarity = true;
                                        for (int16_t j = _point_threshold - 1; j > _point_threshold - 6; --j)
                                        {
                                            if (left_consecutive_point_offset[j] > 0)
                                            {
                                                _is_continue_polarity = false;
                                                break;
                                            }
                                            _total_offset += left_consecutive_point_offset[j];
                                        }
                                        // 判断为悬崖
                                        float _average_offset = _total_offset / 5;
                                        if (_is_continue_polarity && _average_offset > -1.5 && _average_offset <= 0)
                                        {
                                            fix_left_tail.fix_point_type = CLIFF;
                                            fix_left_tail.pos_col = _point_threshold - 2;
                                        }
                                        // 判断为圆弧
                                        else if (_average_offset < -2)
                                        {
                                            fix_left_tail.fix_point_type = ARC_LEFT;
                                            // 找最大的
                                            int _max_cols = left_line[_point_threshold];
                                            for (int16_t j = _point_threshold; j > 0; --j)
                                            {
                                                if (left_line[j] > _max_cols)
                                                {
                                                    _max_cols = left_line[j];
                                                    fix_left_tail.pos_col = j;
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                // 对于左线，如何出现正常的负落差非常大的点，则可视为需要补线
                if (fix_left_tail.pos_col == NO_DEFINE_VALUE && left_consecutive_point_offset[i] < -30
                        && left_line[i] - (int16_t) left_line[i + 2] > 30)
                {
                    // 往后验证看看是否这个落差大的地方是正常的
                    bool is_normal = true;
                    bool is_arc = false;
                    uint8_t max_left_point_tail_col = left_line[i];
                    uint8_t max_left_point_tail_row = i;
                    for (int16_t j = i - 1; j > 0 && j > i - 10; --j)
                    {
                        if (right_line[j] - (int16_t) left_line[j] < 10)
                        {
                            is_normal = false;
                            is_arc = false;
                            break;
                        }
                        else if (left_consecutive_point_offset[j] > 4)
                        {
                            is_normal = false;
                            // 验证是否遇到圆环
                            for (int16_t k_ = j; k_ >= 5; k_--)
                            {
                                int16_t three_left_avea = (left_line[k_] + left_line[k_ - 1] + left_line[k_ - 2]) / 3;
                                if (max_left_point_tail_col - three_left_avea > 20)
                                {
                                    is_arc = true;
                                    is_normal = true;
                                }
                            }
                        }
                        if (left_line[j] > max_left_point_tail_col - 1)
                        {
                            max_left_point_tail_col = left_line[j];
                            max_left_point_tail_row = j;
                        }
                    }
                    // 重新验证是否真的遇到圆环
                    if (is_arc == false)
                    {
                        for (int16_t k_ = i - 6; k_ >= 5; k_--)
                        {
                            int16_t three_left_avea = (left_line[k_] + left_line[k_ - 1] + left_line[k_ - 2]) / 3;
                            if (max_left_point_tail_col - three_left_avea > 20)
                            {
                                is_arc = true;
                                is_normal = true;
                            }
                        }
                    }
                    // 往摄像头方向找补线头
                    if (is_normal)
                    {
                        uint8_t max_left_point_head_col = 0;
                        uint8_t max_left_point_head_row = src_rows - 1;
                        for (int16_t j = i + 2; j < src_rows; ++j)
                        {
                            if (left_line[j] > max_left_point_head_col)
                            {
                                max_left_point_head_col = left_line[j];
                                max_left_point_head_row = j;
                            }
                        }
                        // 验证最大的点是不是正常点
                        bool is_normal_max_point = true;
                        for (int16_t j = max_left_point_head_row; j < max_left_point_head_row + 4; ++j)
                        {
                            if (ABS(left_consecutive_point_offset[j]) > 4)
                            {
                                is_normal_max_point = false;
                                break;
                            }
                        }
                        if (is_normal_max_point)
                        {
                            fix_left_head.pos_col = max_left_point_head_row;
                            fix_left_head.fix_point_type = CLIFF;
                            fix_left_tail.pos_col = max_left_point_tail_row;
                            if (is_arc)
                                fix_left_tail.fix_point_type = ARC_LEFT;
                            else
                                fix_left_tail.fix_point_type = CLIFF;
                        }
                    }
                }
            }
            // 对于road_type == LEFT_ROTARY_IN_FIRST_SUNKEN || road_type == RIGHT_ROTARY_IN_FIRST_SUNKEN需要特殊找补线点
            if (fix_left_tail.pos_col == NO_DEFINE_VALUE)
            {
                if (((road_type == LEFT_ROTARY_IN_FIRST_SUNKEN
                        || (road_type == LEFT_ROTARY_IN_SECOND_SUNKEN && go_in_rotary_stage_left == 0))
                        && (left_consecutive_point_offset[i] <= 0 && left_consecutive_point_offset[i + 1] <= 0
                                && left_consecutive_point_offset[i] + left_consecutive_point_offset[i + 1] < -30))
                        || (go_in_rotary_stage_left == 1
                                && (left_consecutive_point_offset[i] <= 0 && left_consecutive_point_offset[i + 1] <= 0
                                        && left_consecutive_point_offset[i] + left_consecutive_point_offset[i + 1] < -30)))
                {
                    int16_t temp_fix_left_tail_pos_col_rows = -1;
                    // 往后搜直到搜到两点间距为1
                    for (int16_t j = i; j >= 0; --j)
                    {
                        if (left_consecutive_point_offset[j] >= -2)
                        {
                            temp_fix_left_tail_pos_col_rows = j;
                            break;
                        }
                    }
                    if (temp_fix_left_tail_pos_col_rows != -1 && i - temp_fix_left_tail_pos_col_rows <= 3)
                    {
                        // 往车前搜看看是否搜到
                        uint8_t num_left_zero = 0;
                        for (int16_t j = i + 1; j <= i + 7; ++j)
                        {
                            if (left_line[j] == 0)
                            {
                                num_left_zero++;
                                if (num_left_zero >= 3)
                                {
                                    fix_left_head.pos_col = src_rows - 9;
                                    fix_left_head.fix_point_type = ARC_LEFT;
                                    fix_left_tail.pos_col = temp_fix_left_tail_pos_col_rows;
                                    fix_left_tail.fix_point_type = CLIFF;
                                }
                            }
                        }
                    }
                }
            }

            // 右线
            if (fix_right_head.pos_col == NO_DEFINE_VALUE)
            {
                if (right_line[i + 1] == src_cols - 1 && right_line[i + 2] == src_cols - 1
                        && right_line[i + 3] == src_cols - 1 && i > src_rows - 10) // 右线不可见时[暂时]视为遇到在某个特殊元素中
                {
                    // 判断是否含断点
                    uint8_t num_little_offset = 0;
                    bool is_continue_little_offset = true;
                    for (int16_t j = i; j >= 5; --j)
                    {
                        if (ABS(right_consecutive_point_offset[j]) > 3)
                        {
                            is_continue_little_offset = false;
                            break;
                        }
                        else if (ABS(right_consecutive_point_offset[j]) < 4 && right_consecutive_point_offset[j] != 0)
                        {
                            num_little_offset++;
                            if (num_little_offset > 4)
                                break;
                        }
                    }
                    if (!is_continue_little_offset)
                    {
                        fix_right_head.pos_col = i + 3;
                        fix_right_head.fix_point_type = NO_DEFINE_VALUE_FIX_POINT;
                        if (road_type == RIGHT_ROTARY_IN_FIRST_SUNKEN)
                        {
                            // 往后找最小点
                            int _min = src_cols - 1;
                            int _min_col = i;
                            for (int16_t j = i; j >= end_src_rows; --j)
                            {
                                if (right_line[j] < _min)
                                {
                                    _min = right_line[j];
                                    _min_col = j;
                                }
                                if (right_line[j] > _min + 10)
                                    break;
                            }
                            fix_right_tail.fix_point_type = ARC_RIGHT;
                            fix_right_tail.pos_col = _min_col;
                        }
                        else
                        {
                            // 如果不处于环道第一状态, 找悬崖点
                            bool is_meet_right_cliff_threshold_offset = false;
                            int16_t _j_find_cliff = i;
                            for (_j_find_cliff = i; _j_find_cliff >= end_src_rows; --_j_find_cliff)
                            {
                                if (right_consecutive_point_offset[_j_find_cliff]
                                        > RIGHT_LINE_TAIL_OFFSET_THRESHOLD_NORMAL + 6)
                                {
                                    is_meet_right_cliff_threshold_offset = true;
                                    break;
                                }
                            }
                            if (is_meet_right_cliff_threshold_offset)
                            {
                                // 接下来找符合作为悬崖点的点
                                for (_j_find_cliff; _j_find_cliff >= end_src_rows; --_j_find_cliff)
                                {
                                    if (ABS(right_consecutive_point_offset[_j_find_cliff]) < 2)
                                    {
                                        fix_right_tail.fix_point_type = CLIFF;
                                        fix_right_tail.pos_col = _j_find_cliff;
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }
                else if (right_consecutive_point_offset[i] < RIGHT_LINE_HEAD_OFFSET_THRESHOLD_NORMAL) // 落差临界点，深入扫描确认是否为悬崖
                {
                    // 回溯寻找落差小于3的两个以上的点
                    int16_t right_head_time = NO_DEFINE_VALUE; // 暂时补线头点
                    int16_t right_tail_time = NO_DEFINE_VALUE; // 暂时补线尾点

                    // 先判断是否是圆弧
                    uint8_t _num_positive_offset = 0;
                    int _j_find_arc_min = i;
                    for (_j_find_arc_min; _j_find_arc_min < src_rows - 3; ++_j_find_arc_min)
                    {
                        if (right_consecutive_point_offset[_j_find_arc_min] > 0)
                            break;
                        if (right_consecutive_point_offset[_j_find_arc_min] < 0)
                            _num_positive_offset++;
                    }
                    if (_num_positive_offset > 7)
                    {
                        fix_right_head.fix_point_type = ARC_RIGHT;
                        fix_right_head.pos_col = _j_find_arc_min;
                        if (road_type == RIGHT_ROTARY_IN_FIRST_SUNKEN || road_type == RIGHT_ROTARY_IN_SECOND_SUNKEN)
                        {
                            // 找悬崖点
                            int16_t _j_find_cliff = i;
                            bool _is_find_right_cliff = false;
                            for (_j_find_cliff; _j_find_cliff > 5; --_j_find_cliff)
                            {
                                if (right_consecutive_point_offset[_j_find_cliff]
                                        > RIGHT_LINE_TAIL_OFFSET_THRESHOLD_NORMAL)
                                {
                                    _is_find_right_cliff = true;
                                    break;
                                }
                            }
                            if (_is_find_right_cliff)
                            {
                                // 找最后的大落差
                                int16_t _point_threshold = _j_find_cliff;
                                for (int16_t j = _j_find_cliff; j > _j_find_cliff - 5; --j)
                                {
                                    if (right_consecutive_point_offset[j] > RIGHT_LINE_TAIL_OFFSET_THRESHOLD_NORMAL)
                                    {
                                        _point_threshold = j;
                                    }
                                }
                                // 验证是否是悬崖点
                                if (_point_threshold > 6)
                                {
                                    float _total_offset = 0;
                                    bool _is_continue_polarity = true;
                                    for (int16_t j = _point_threshold - 1; j > _point_threshold - 6; --j)
                                    {
                                        if (right_consecutive_point_offset[j] < 0)
                                        {
                                            _is_continue_polarity = false;
                                            break;
                                        }
                                        _total_offset += right_consecutive_point_offset[j];
                                    }
                                    // 判断为悬崖
                                    float _average_offset = _total_offset / 5;
                                    if (_is_continue_polarity && _average_offset < 1.8 && _average_offset <= 0)
                                    {
                                        fix_right_tail.fix_point_type = CLIFF;
                                        fix_right_tail.pos_col = _point_threshold - 2;
                                    }
                                }
                            }
                        }
                    }
                    //          以下是   不是圆弧
                    else
                    {
                        for (int16_t k_ = i; k_ < i + 6 && k_ < src_rows - 3; ++k_) // 必须能在往回的五个点内找到，否则视为不存在.后一个条件是为了避免溢出
                        {
                            if (ABS(right_consecutive_point_offset[k_]) < 3
                                    && ABS(right_consecutive_point_offset[k_ + 1]) < 3)
                            {
                                right_head_time = k_ + 1;
                                break; // 找到并返回
                            }
                        }
                        if (right_head_time != NO_DEFINE_VALUE) // 成功找到暂时的补线头点
                        {
                            uint8_t train_ele_num = 5;
                            int16_t k_ = i - 1;
                            for (k_; k_ >= train_ele_num; k_--)
                            {
                                int16_t sum_train_ele = 0;
                                for (int16_t _t = k_; _t > k_ - train_ele_num; --_t)
                                {
                                    sum_train_ele += right_consecutive_point_offset[_t];
                                }
                                if (ABS(sum_train_ele) < 5
                                        && right_line[k_ - train_ele_num / 2] - (int16_t) right_line[i + 1] > 10)
                                {
                                    fix_right_head.pos_col = i + 1;
                                    fix_right_head.fix_point_type = CLIFF;
                                }
                                if (sum_train_ele > 12)
                                {
                                    break;
                                }
                            }
                            // 向前寻找
                            if (fix_right_head.pos_col != NO_DEFINE_VALUE) // 补线头点被成功找到，接下来寻找补线尾点
                            {
                                // 先判断是否是悬崖点
                                bool _is_meet_threshold_offset = false;
                                for (k_ = k_ - 1; k_ > 2; --k_)
                                {
                                    if (right_consecutive_point_offset[k_] > RIGHT_LINE_TAIL_OFFSET_THRESHOLD_NORMAL)
                                    {
                                        _is_meet_threshold_offset = true;
                                        break;
                                    }
                                }
                                if (_is_meet_threshold_offset && k_ - 5 > 0)
                                {
                                    // 往后几个点找最后的大落差
                                    int16_t _point_threshold = k_;
                                    for (int16_t j = k_; j > k_ - 5; --j)
                                    {
                                        if (right_consecutive_point_offset[j] > RIGHT_LINE_TAIL_OFFSET_THRESHOLD_NORMAL)
                                        {
                                            _point_threshold = j;
                                        }
                                    }
                                    if (_point_threshold > 6)
                                    {
                                        float _total_offset = 0;
                                        bool _is_continue_polarity = true;
                                        for (int16_t j = _point_threshold - 1; j > _point_threshold - 6; --j)
                                        {
                                            if (right_consecutive_point_offset[j] < 0)
                                            {
                                                _is_continue_polarity = false;
                                                break;
                                            }
                                            _total_offset += right_consecutive_point_offset[j];
                                        }
                                        // 判断为悬崖
                                        float _average_offset = _total_offset / 5;
                                        if (_is_continue_polarity && _average_offset < 1.5 && _average_offset <= 0)
                                        {
                                            fix_right_tail.fix_point_type = CLIFF;
                                            fix_right_tail.pos_col = _point_threshold - 2;
                                        }
                                        // 判断为圆弧
                                        else if (_average_offset > 2)
                                        {
                                            fix_right_tail.fix_point_type = ARC_RIGHT;
                                            // 找最小的
                                            int _min_cols = right_line[_point_threshold];
                                            for (int16_t j = _point_threshold; j > 0; --j)
                                            {
                                                if (right_line[j] < _min_cols)
                                                {
                                                    _min_cols = right_line[j];
                                                    fix_right_tail.pos_col = j;
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                // 对于右线，如何出现正常的正落差非常大的点，则可视为需要补线
                if (fix_right_tail.pos_col == NO_DEFINE_VALUE && right_consecutive_point_offset[i] > 30
                        && right_line[i] - (int16_t) right_line[i + 2] < -30)
                {
                    // output += "SIGN_RIGHT_2:" + std::to_string(i) + "\r\n";
                    // 往后验证看看是否这个落差大的地方是正常的
                    bool is_normal = true;
                    bool is_arc = false;
                    uint8_t min_right_point_tail_col = right_line[i];
                    uint8_t min_right_point_tail_row = i;
                    for (int16_t j = i - 1; j > 0 && j > i - 10; --j)
                    {
                        if (right_line[j] - (int16_t) left_line[j] < 10)
                        {
                            is_normal = false;
                            is_arc = false;
                            break;
                        }
                        else if (right_consecutive_point_offset[j] < -4)
                        {
                            // 验证是否遇到圆环
                            is_normal = false;
                            for (int16_t k_ = j; k_ >= 5; k_--)
                            {
                                int16_t three_right_avea = (right_line[k_] + right_line[k_ - 1] + right_line[k_ - 2])
                                        / 3;
                                if (three_right_avea - min_right_point_tail_col > 20)
                                {
                                    is_arc = true;
                                    is_normal = true;
                                }
                            }
                        }
                        if (right_line[j] < min_right_point_tail_col + 1)
                        {
                            min_right_point_tail_col = right_line[j];
                            min_right_point_tail_row = j;
                        }
                    }
                    // 重新验证是否真的遇到圆环
                    if (is_arc = false)
                    {
                        for (int16_t k_ = i - 6; k_ >= 5; k_--)
                        {
                            int16_t three_right_avea = (right_line[k_] + right_line[k_ - 1] + right_line[k_ - 2]) / 3;
                            if (three_right_avea - min_right_point_tail_col > 20)
                            {
                                is_arc = true;
                                is_normal = true;
                            }
                        }
                    }
                    // 往摄像头方向找补线头
                    if (is_normal)
                    {
                        uint8_t min_right_point_head_col = src_cols - 1;
                        uint8_t min_right_point_head_row = src_rows - 1;
                        for (int16_t j = i + 2; j < src_rows; ++j)
                        {
                            if (right_line[j] < min_right_point_head_col)
                            {
                                min_right_point_head_col = right_line[j];
                                min_right_point_head_row = j;
                            }
                        }
                        // 验证最大的点是不是正常点
                        bool is_normal_min_point = true;
                        for (int16_t j = min_right_point_head_row; j < min_right_point_head_row + 4; ++j)
                        {
                            if (ABS(right_consecutive_point_offset[j]) > 4)
                            {
                                is_normal_min_point = false;
                                break;
                            }
                        }
                        if (is_normal_min_point)
                        {
                            fix_right_head.pos_col = min_right_point_head_row;
                            fix_right_head.fix_point_type = CLIFF;
                            fix_right_tail.pos_col = min_right_point_tail_row;
                            if (is_arc)
                                fix_right_tail.fix_point_type = ARC_RIGHT;
                            else
                                fix_right_tail.fix_point_type = CLIFF;
                        }
                    }
                }
            }
            if (fix_right_tail.pos_col == NO_DEFINE_VALUE)
            {
                if (((road_type == RIGHT_ROTARY_IN_FIRST_SUNKEN
                        || (road_type == RIGHT_ROTARY_IN_SECOND_SUNKEN && go_in_rotary_stage_right == 0))
                        && (right_consecutive_point_offset[i] >= 0 && right_consecutive_point_offset[i + 1] >= 0
                                && right_consecutive_point_offset[i] + right_consecutive_point_offset[i + 1] > 30))
                        || (go_in_rotary_stage_right == 1
                                && (right_consecutive_point_offset[i] >= 0 && right_consecutive_point_offset[i + 1] >= 0
                                        && right_consecutive_point_offset[i] + right_consecutive_point_offset[i + 1]
                                                > 30)))
                {
                    // output += "SIGN_14_2:IN_RIGHT\r\n";
                    int16_t temp_fix_right_tail_pos_col_rows = -1;
                    // 往后搜直到搜到两点间距为1
                    for (int16_t j = i; j >= 0; --j)
                    {
                        if (right_consecutive_point_offset[j] <= 2)
                        {
                            temp_fix_right_tail_pos_col_rows = j;
                            break;
                        }
                    }
                    if (temp_fix_right_tail_pos_col_rows != -1 && i - temp_fix_right_tail_pos_col_rows <= 3)
                    {
                        // 往车前搜看看是否搜到连续的右线为src_cols - 1
                        uint8_t num_right_col_ = 0;
                        for (int16_t j = i + 1; j <= i + 7; ++j)
                        {
                            if (right_line[j] == src_cols - 1)
                            {
                                num_right_col_++;
                                if (num_right_col_ >= 3)
                                {
                                    fix_right_head.pos_col = src_rows - 9;
                                    fix_right_head.fix_point_type = ARC_RIGHT;
                                    fix_right_tail.pos_col = temp_fix_right_tail_pos_col_rows;
                                    fix_right_tail.fix_point_type = CLIFF;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    end_src_rows = 0;
    if (road_type == NO_FIX_ROAD || road_type == ONLY_FIX_LEFT_ROAD || road_type == ONLY_FIX_RIGHT_ROAD)
    {
        if (fix_left_tail.pos_col != NO_DEFINE_VALUE && fix_left_head.pos_col != NO_DEFINE_VALUE
                && fix_right_tail.pos_col != NO_DEFINE_VALUE && fix_right_head.pos_col != NO_DEFINE_VALUE)
        {
            road_type = CROSSROAD;
        }
        else if (fix_left_tail.pos_col != NO_DEFINE_VALUE && fix_left_head.pos_col != NO_DEFINE_VALUE
                && (fix_right_tail.pos_col == NO_DEFINE_VALUE || fix_right_head.pos_col == NO_DEFINE_VALUE))
        {
            road_type = ONLY_FIX_LEFT_ROAD;
        }
        else if ((fix_left_tail.pos_col == NO_DEFINE_VALUE || fix_left_head.pos_col == NO_DEFINE_VALUE)
                && fix_right_tail.pos_col != NO_DEFINE_VALUE && fix_right_head.pos_col != NO_DEFINE_VALUE)
        {
            road_type = ONLY_FIX_RIGHT_ROAD;
        }
        else
        {
            road_type = NO_FIX_ROAD;
        }
    }
    /*
     if (road_type == IN_LEFT_T_ING)
     {
     if (fix_left_head.fix_point_type != NO_DEFINE_VALUE_FIX_POINT)
     {
     road_type = IN_LEFT_T_ED;
     }
     }
     if (road_type == IN_RIGHT_T_ING)
     {
     if (fix_right_head.fix_point_type != NO_DEFINE_VALUE_FIX_POINT)
     {
     road_type = IN_RIGHT_T_ED;
     }
     }*/
#ifdef UPPER_COMPUTER
    // output += "ROAD_TYPE:" + std::to_string(road_type) + "\r\n";
#endif
    road_type_for_control = road_type;

    switch (road_type)
    {
        case OUT_CARBARN :
        {
            // 出库打死
            // pwm_duty(PWM1_MODULE3_CHB_D1, pwm_right);
            // pwm_duty(PWM2_MODULE3_CHA_D2, pwm_left); // 8000
            /*
             if (is_right_out)
             pwm_duty(PWM4_MODULE2_CHA_C30, 5080); // 5750中, 6420左, 5080右
             else
             pwm_duty(PWM4_MODULE2_CHA_C30, 6420); // 5750中, 6420左, 5080右
             */
            road_type = NO_FIX_ROAD;
            // float k_line = -2.4;
            // float a_line = (src_rows - 1) * 2.4;
            // // uint8_t right_line_not_define_num = 0;
            // for (uint8_t i = 0; i < src_rows - 1; ++i)
            // {
            //  int16_t temp = i * k_line + a_line;
            //  if (temp >= 0 && temp < src_cols)
            //      left_line[i] = temp;

            //  // if (right_line[i] != src_cols - 1)
            //  //  right_line_not_define_num++;
            //  right_line[i] = src_cols - 1;
            // }
            // // if (right_line_not_define_num > src_rows - 24)
            // //   road_type = NO_FIX_ROAD;
            // bool is_out_ = true;
            // for (uint16_t i = src_rows - 1; i > 45; --i)
            // {
            //  if (ABS(right_consecutive_point_offset[i]) > 20)
            //  {
            //      is_out_ = false;
            //      break;
            //  }
            // }
            // if (is_out_)
            // {
            //  road_type = NO_FIX_ROAD;
            // }

            break;
        }
        case NO_FIX_ROAD :
            break;
        case ONLY_FIX_LEFT_ROAD :
        {
            float k = (float) ((int16_t) left_line[fix_left_head.pos_col] - (int16_t) left_line[fix_left_tail.pos_col])
                    / (float) (fix_left_head.pos_col - fix_left_tail.pos_col);
            float b = left_line[fix_left_head.pos_col] - k * fix_left_head.pos_col;
            for (int t = fix_left_head.pos_col; t >= fix_left_tail.pos_col; --t)
            {
                int temp = k * t + b;
                if (temp >= 0 && temp < src_cols)
                {
                    left_line[t] = temp;
                }
            }
            // 左环道检测入口
            uint8_t right_line_begin_ = src_rows - 1;
            for (int i_find_right_line_begin_ = src_rows - 1; i_find_right_line_begin_ >= 0; --i_find_right_line_begin_)
            {
                if (right_line[i_find_right_line_begin_] != src_cols - 1)
                {
                    right_line_begin_ = i_find_right_line_begin_;
                    break;
                }
            }

            // 左三叉入口检测
            // 先往回找悬崖点
            if (fix_left_tail.fix_point_type == CLIFF)
            {
                // 找最大的悬崖点
                uint16_t _max_cliff_x = left_line[fix_left_tail.pos_col];
                uint16_t _max_cliff_y = fix_left_tail.pos_col;
                uint16_t _max_cliff_offset = 0;
                for (int16_t _j = fix_left_tail.pos_col; _j < src_rows - 1 && left_line[_j] != 0; ++_j)
                {
                    int16_t _tmp_offset = left_line[_j] - (int16_t) left_line[_j + 1];
                    if (_tmp_offset > _max_cliff_offset)
                    {
                        _max_cliff_x = left_line[_j];
                        _max_cliff_y = _j;
                    }
                }
                // 找右线
                bool is_left_T = true;
                if (_max_cliff_y - fix_left_tail.pos_col > 5)
                {
                    uint8_t i_cliff = _max_cliff_y;
                    for (int16_t i = i_cliff; i >= fix_left_tail.pos_col; --i)
                    {
                        bool is_white = false;
                        for (int16_t j = left_line[i]; j >= 3; --j)
                        {
                            if (src_pixel_mat[i][j] > threshold_val && src_pixel_mat[i][j - 1] > threshold_val
                                    && src_pixel_mat[i][j - 2] > threshold_val
                                    && src_pixel_mat[i][j - 3] > threshold_val)
                            {
                                is_white = true;
                            }
                            if (is_white && src_pixel_mat[i][j - 3] < threshold_val)
                            {
                                is_white = false;
                                break;
                            }
                        }
                        if (!is_white)
                        {
                            is_left_T = false;
                            break;
                        }
                    }
                }
                if (fix_left_head.fix_point_type == NO_DEFINE_VALUE_FIX_POINT)
                {
                    if (is_left_T)
                    {
                        // 右线落差不能大而且远离摄像头段的是外倾的线
                        bool _is_out_s = true; // 是否外倾
                        bool _is_have_big_offset = false;
                        int16_t _min_right_line_x = right_line[fix_left_tail.pos_col];
                        int16_t _min_right_line_y = fix_left_tail.pos_col;
                        for (int16_t _j = fix_left_tail.pos_col; _j < src_rows - 2 && !_is_have_big_offset; ++_j)
                        {
                            if (ABS(right_consecutive_point_offset[_j]) > 4)
                                _is_have_big_offset = true;
                            if (right_consecutive_point_offset[_j] > 0 && right_consecutive_point_offset[_j + 1] > 0
                                    && right_consecutive_point_offset[_j + 2] > 0)
                            {
                                _is_out_s = false;
                                _min_right_line_x = right_line[_j];
                                _min_right_line_y = _j;
                            }
                        }
                        if (!_is_have_big_offset && _min_right_line_y - fix_left_tail.pos_col > 25)
                        {
                            road_type = IN_LEFT_T_ING;
                        }
                    }
                }
                else if (fix_left_head.fix_point_type == CLIFF || fix_left_head.fix_point_type == ARC_LEFT)
                {
                    if (is_left_T)
                    {
                        // 右线落差不能大而且远离摄像头段的是外倾的线
                        bool _is_out_s_right = true; // 是否外倾
                        bool _is_have_big_offset_right = false;
                        int16_t _min_right_line_x = right_line[fix_left_tail.pos_col];
                        int16_t _min_right_line_y = fix_left_tail.pos_col;
                        for (int16_t _j = fix_left_tail.pos_col; _j < src_rows - 2 && !_is_have_big_offset_right; ++_j)
                        {
                            if (ABS(right_consecutive_point_offset[_j]) > 4)
                                _is_have_big_offset_right = true;
                            if (right_consecutive_point_offset[_j] > 0 && right_consecutive_point_offset[_j + 1] > 0
                                    && right_consecutive_point_offset[_j + 2] > 0)
                            {
                                _is_out_s_right = false;
                                _min_right_line_x = right_line[_j];
                                _min_right_line_y = _j;
                            }
                        }
                        // 左线落差不能大而且远离摄像头段的是外倾的线
                        bool _is_out_s_left = true; // 是否外倾
                        bool _is_have_big_offset_left = false;
                        int16_t _max_left_line_x = left_line[fix_left_tail.pos_col];
                        int16_t _max_left_line_y = fix_left_tail.pos_col;
                        for (int16_t _j = fix_left_tail.pos_col; _j < src_rows - 2 && !_is_have_big_offset_left; ++_j)
                        {
                            if (ABS(left_consecutive_point_offset[_j]) > 4)
                                _is_have_big_offset_left = true;
                            if (left_consecutive_point_offset[_j] < 0 && left_consecutive_point_offset[_j + 1] < 0
                                    && left_consecutive_point_offset[_j + 2] < 0)
                            {
                                _is_out_s_left = false;
                                _max_left_line_x = left_line[_j];
                                _max_left_line_y = _j;
                            }
                        }
                        if (!_is_have_big_offset_right && !_is_have_big_offset_left
                                && _min_right_line_y - fix_left_tail.pos_col > 18
                                && _max_left_line_y - fix_left_tail.pos_col > 18)
                        {
                            road_type = IN_LEFT_T_ING;
                        }
                    }
                }
            }

            if (end_src_rows < 3)
            {
                // 2022年5月26日
                uint8_t num_err = 0;
                StraightLineCoeffic straight_line_coeffic = LinearRegress(right_line, end_src_rows + 1,
                        right_line_begin_);
                int16_t n = end_src_rows + 1;
                for (n; n < right_line_begin_; ++n)
                {
                    if (ABS(right_line[n] - (straight_line_coeffic.k * n + straight_line_coeffic.a)) > 5)
                    {
                        num_err++;
                    }
                }
                if (road_type == IN_LEFT_T_ING)
                    return;
                road_type = NO_FIX_ROAD;
                if (num_err < 4 && fix_left_head.fix_point_type == CLIFF && fix_left_tail.fix_point_type == ARC_LEFT)
                {
                    road_type = LEFT_ROTARY_IN_FIRST_SUNKEN;
                }
            }
            // 左环道检测出口
            if (road_type == NO_FIX_ROAD)
            {
                fix_left_tail.pos_col = NO_DEFINE_VALUE;
                fix_left_head.pos_col = NO_DEFINE_VALUE;
                road_type = NO_FIX_ROAD;
            }

            break;
        }
        case ONLY_FIX_RIGHT_ROAD :
        {
            float k = (float) ((int16_t) right_line[fix_right_head.pos_col]
                    - (int16_t) right_line[fix_right_tail.pos_col]) / (fix_right_head.pos_col - fix_right_tail.pos_col);
            float b = right_line[fix_right_head.pos_col] - k * fix_right_head.pos_col;
            for (int t = fix_right_head.pos_col; t > fix_right_tail.pos_col; --t)
            {
                int temp = k * t + b;
                if (temp >= 0 && temp < src_cols)
                {
                    right_line[t] = temp;
                }
            }
            // 右环道检测入口
            uint8_t left_line_begin_ = src_rows - 1;
            for (int i_find_left_line_begin_ = src_rows - 1; i_find_left_line_begin_ >= 0; --i_find_left_line_begin_)
            {
                if (left_line[i_find_left_line_begin_] != 0)
                {
                    left_line_begin_ = i_find_left_line_begin_;
                    break;
                }
            }

            //            // 右三叉入口检测
            //            // 先往回找悬崖点
            //            if (fix_right_head.fix_point_type == NO_DEFINE_VALUE_FIX_POINT)
            //            {
            //                uint8_t i_cliff = fix_right_tail.pos_col;
            //                for (uint8_t i = fix_right_tail.pos_col; i < src_rows; ++i)
            //                {
            //                    if (right_consecutive_point_offset[i] > 30)
            //                    {
            //                        i_cliff = i;
            //                    }
            //                }
            //                bool is_right_T = true;     // 是否遇到左环道
            //                for (int16_t i = i_cliff; i >= 0; --i)
            //                {
            //                    bool is_white = false;
            //                    for (int16_t j = right_line[i]; j < src_cols; ++j)
            //                    {
            //                        if (src_pixel_mat[i][j] > threshold_val && src_pixel_mat[i][j - 1] > threshold_val
            //                                && src_pixel_mat[i][j - 2] > threshold_val && src_pixel_mat[i][j - 3] > threshold_val)
            //                        {
            //                            is_white = true;
            //                        }
            //                    }
            //                    if (!is_white)
            //                    {
            //                        is_right_T = false;
            //                        break;
            //                    }
            //                }
            //                if (is_right_T)
            //                {
            //                    road_type = IN_RIGHT_T_ING;
            //                    return;
            //                }
            //            }

            if (end_src_rows < 3)
            {
                // 2022年5月26日
                uint8_t num_err = 0;
                StraightLineCoeffic straight_line_coeffic = LinearRegress(left_line, end_src_rows + 1,
                        left_line_begin_);
                uint8_t n = end_src_rows + 1;
                for (n; n < left_line_begin_; ++n)
                {
                    if (ABS(left_line[n] - (straight_line_coeffic.k * n + straight_line_coeffic.a)) > 3)
                    {
                        num_err++;
                    }
                }
                road_type = NO_FIX_ROAD;
                if (num_err < 4 && fix_right_head.fix_point_type == CLIFF && fix_right_tail.fix_point_type == ARC_RIGHT)
                {
                    road_type = RIGHT_ROTARY_IN_FIRST_SUNKEN;
                }
            }
            // 右环道检测出口
            if (road_type == NO_FIX_ROAD)
            {
                fix_right_head.pos_col = NO_DEFINE_VALUE;
                fix_right_tail.pos_col = NO_DEFINE_VALUE;
                road_type = NO_FIX_ROAD;
            }

            break;
        }
        case IN_LEFT_T_ED :
        {
            // 左三叉入口检测
            // 先往回找悬崖点
            if (fix_left_tail.fix_point_type == CLIFF)
            {
                // 找最大的悬崖点
                uint16_t _max_cliff_x = left_line[fix_left_tail.pos_col];
                uint16_t _max_cliff_y = fix_left_tail.pos_col;
                uint16_t _max_cliff_offset = 0;
                for (int16_t _j = fix_left_tail.pos_col; _j < src_rows - 1 && left_line[_j] != 0; ++_j)
                {
                    int16_t _tmp_offset = left_line[_j] - (int16_t) left_line[_j + 1];
                    if (_tmp_offset > _max_cliff_offset)
                    {
                        _max_cliff_x = left_line[_j];
                        _max_cliff_y = _j;
                    }
                }
                // 找右线
                bool is_left_T = true;
                if (_max_cliff_y - fix_left_tail.pos_col > 5)
                {
                    uint8_t i_cliff = _max_cliff_y;
                    for (int16_t i = i_cliff; i >= fix_left_tail.pos_col; --i)
                    {
                        bool is_white = false;
                        for (int16_t j = left_line[i]; j >= 3; --j)
                        {
                            if (src_pixel_mat[i][j] > threshold_val && src_pixel_mat[i][j - 1] > threshold_val
                                    && src_pixel_mat[i][j - 2] > threshold_val
                                    && src_pixel_mat[i][j - 3] > threshold_val)
                            {
                                is_white = true;
                            }
                            if (is_white && src_pixel_mat[i][j - 3] < threshold_val)
                            {
                                is_white = false;
                                break;
                            }
                        }
                        if (!is_white)
                        {
                            is_left_T = false;
                            break;
                        }
                    }
                }
                if (fix_left_head.fix_point_type == NO_DEFINE_VALUE_FIX_POINT)
                {
                    if (is_left_T)
                    {
                        // 右线落差不能大而且远离摄像头段的是外倾的线
                        bool _is_out_s = true; // 是否外倾
                        bool _is_have_big_offset = false;
                        int16_t _min_right_line_x = right_line[fix_left_tail.pos_col];
                        int16_t _min_right_line_y = fix_left_tail.pos_col;
                        for (int16_t _j = fix_left_tail.pos_col; _j < src_rows - 2 && !_is_have_big_offset; ++_j)
                        {
                            if (ABS(right_consecutive_point_offset[_j]) > 4)
                                _is_have_big_offset = true;
                            if (right_consecutive_point_offset[_j] > 0 && right_consecutive_point_offset[_j + 1] > 0
                                    && right_consecutive_point_offset[_j + 2] > 0)
                            {
                                _is_out_s = false;
                                _min_right_line_x = right_line[_j];
                                _min_right_line_y = _j;
                            }
                        }
                        if (!_is_have_big_offset && _min_right_line_y - fix_left_tail.pos_col > 25)
                        {
                            road_type = OUT_LEFT_ING;
                        }
                    }
                }
                else if (fix_left_head.fix_point_type == CLIFF || fix_left_head.fix_point_type == ARC_LEFT)
                {
                    if (is_left_T)
                    {
                        // 右线落差不能大而且远离摄像头段的是外倾的线
                        bool _is_out_s_right = true; // 是否外倾
                        bool _is_have_big_offset_right = false;
                        int16_t _min_right_line_x = right_line[fix_left_tail.pos_col];
                        int16_t _min_right_line_y = fix_left_tail.pos_col;
                        for (int16_t _j = fix_left_tail.pos_col; _j < src_rows - 2 && !_is_have_big_offset_right; ++_j)
                        {
                            if (ABS(right_consecutive_point_offset[_j]) > 4)
                                _is_have_big_offset_right = true;
                            if (right_consecutive_point_offset[_j] > 0 && right_consecutive_point_offset[_j + 1] > 0
                                    && right_consecutive_point_offset[_j + 2] > 0)
                            {
                                _is_out_s_right = false;
                                _min_right_line_x = right_line[_j];
                                _min_right_line_y = _j;
                            }
                        }
                        // 左线落差不能大而且远离摄像头段的是外倾的线
                        bool _is_out_s_left = true; // 是否外倾
                        bool _is_have_big_offset_left = false;
                        int16_t _max_left_line_x = left_line[fix_left_tail.pos_col];
                        int16_t _max_left_line_y = fix_left_tail.pos_col;
                        for (int16_t _j = fix_left_tail.pos_col; _j < src_rows - 2 && !_is_have_big_offset_left; ++_j)
                        {
                            if (ABS(left_consecutive_point_offset[_j]) > 4)
                                _is_have_big_offset_left = true;
                            if (left_consecutive_point_offset[_j] < 0 && left_consecutive_point_offset[_j + 1] < 0
                                    && left_consecutive_point_offset[_j + 2] < 0)
                            {
                                _is_out_s_left = false;
                                _max_left_line_x = left_line[_j];
                                _max_left_line_y = _j;
                            }
                        }
                        if (!_is_have_big_offset_right && !_is_have_big_offset_left
                                && _min_right_line_y - fix_left_tail.pos_col > 18
                                && _max_left_line_y - fix_left_tail.pos_col > 18)
                        {
                            road_type = OUT_LEFT_ING;
                        }
                    }
                }
            }

            if (fix_left_head.pos_col != NO_DEFINE_VALUE && fix_left_tail.pos_col != NO_DEFINE_VALUE)
            {
                float k = (float) ((int16_t) left_line[fix_left_head.pos_col]
                        - (int16_t) left_line[fix_left_tail.pos_col])
                        / (float) (fix_left_head.pos_col - fix_left_tail.pos_col);
                float b = left_line[fix_left_head.pos_col] - k * fix_left_head.pos_col;
                for (int t = fix_left_head.pos_col; t >= fix_left_tail.pos_col; --t)
                {
                    int temp = k * t + b;
                    if (temp >= 0 && temp < src_cols)
                    {
                        left_line[t] = temp;
                    }
                }
            }
            break;
        }
        case IN_LEFT_T_ING :
        {
            if (fix_left_head.pos_col != NO_DEFINE_VALUE && fix_left_tail.pos_col != NO_DEFINE_VALUE)
            {
                float k = (float) ((int16_t) left_line[fix_left_head.pos_col]
                        - (int16_t) left_line[fix_left_tail.pos_col])
                        / (float) (fix_left_head.pos_col - fix_left_tail.pos_col);
                float b = left_line[fix_left_head.pos_col] - k * fix_left_head.pos_col;
                for (int t = fix_left_head.pos_col; t >= fix_left_tail.pos_col; --t)
                {
                    int temp = k * t + b;
                    if (temp >= 0 && temp < src_cols)
                    {
                        left_line[t] = temp;
                    }
                }
            }
            else
            {
                bool _is_have_big_offset = false;
                for (int16_t i = src_rows; i >= 10; --i)
                {
                    if (left_consecutive_point_offset[i] < -12)
                        _is_have_big_offset = true;
                }
                if (!_is_have_big_offset)
                    road_type = IN_LEFT_T_ED;
            }
            break;
        }
        case OUT_LEFT_ING :
        {
            if (fix_left_head.pos_col != NO_DEFINE_VALUE && fix_left_tail.pos_col != NO_DEFINE_VALUE)
            {
                float k = (float) ((int16_t) left_line[fix_left_head.pos_col]
                        - (int16_t) left_line[fix_left_tail.pos_col])
                        / (float) (fix_left_head.pos_col - fix_left_tail.pos_col);
                float b = left_line[fix_left_head.pos_col] - k * fix_left_head.pos_col;
                for (int t = fix_left_head.pos_col; t >= fix_left_tail.pos_col; --t)
                {
                    int temp = k * t + b;
                    if (temp >= 0 && temp < src_cols)
                    {
                        left_line[t] = temp;
                    }
                }
            }
            else
            {
                bool _is_have_big_offset = false;
                for (int16_t i = src_rows; i >= 10; --i)
                {
                    if (left_consecutive_point_offset[i] < -12)
                        _is_have_big_offset = true;
                }
                if (!_is_have_big_offset)
                    road_type = NO_FIX_ROAD;
            }
            break;
        }
        case IN_RIGHT_T_ED :
        case IN_RIGHT_T_ING :
        case OUT_RIGHT_ING :
        {
            if (road_type == IN_RIGHT_T_ED || road_type == OUT_RIGHT_ING)
            {
                bool is_have_offset = true;
                for (int16_t i = src_rows - 1; i > 10; --i)
                {
                    if (right_consecutive_point_offset[i] + right_consecutive_point_offset[i - 1] > 45)
                    {
                        for (int16_t j = i - 1; j >= i - 15; --j)
                        {
                            bool is_white = false;
                            for (int16_t k = right_line[j]; k < src_cols; ++k)
                            {
                                if (src_pixel_mat[j][k] > threshold_val && src_pixel_mat[j][k - 1] > threshold_val
                                        && src_pixel_mat[j][k - 2] > threshold_val
                                        && src_pixel_mat[j][k - 3] > threshold_val
                                        && src_pixel_mat[j][k - 4] > threshold_val
                                        && src_pixel_mat[j][k - 5] > threshold_val
                                        && src_pixel_mat[j][k - 6] > threshold_val)
                                {
                                    is_white = true;
                                }
                            }
                            if (!is_white)
                            {
                                is_have_offset = false;
                                break;
                            }
                        }
                        if (!is_have_offset)
                        {
                            is_have_offset = true;
                            road_type = OUT_RIGHT_ING;
                            fix_right_head.pos_col = src_rows - 1;
                            fix_right_tail.pos_col = i;
                        }
                    }
                }
                if (!is_have_offset)
                {
                    road_type = NO_FIX_ROAD;
                }
            }
            float k = (float) ((int16_t) right_line[fix_right_head.pos_col]
                    - (int16_t) right_line[fix_right_tail.pos_col]) / (fix_right_head.pos_col - fix_right_tail.pos_col);
            float b = right_line[fix_right_head.pos_col] - k * fix_right_head.pos_col;
            for (int t = fix_right_head.pos_col; t > fix_right_tail.pos_col; --t)
            {
                int temp = k * t + b;
                if (temp >= 0 && temp < src_cols)
                {
                    right_line[t] = temp;
                }
            }
            break;
        }

        case CROSSROAD :
        {
            float k_left = (float) ((int16_t) left_line[fix_left_head.pos_col]
                    - (int16_t) left_line[fix_left_tail.pos_col]) / (fix_left_head.pos_col - fix_left_tail.pos_col);
            float b_left = left_line[fix_left_head.pos_col] - k_left * fix_left_head.pos_col;
            for (int t = fix_left_head.pos_col; t >= fix_left_tail.pos_col; --t)
            {
                int temp = k_left * t + b_left;
                if (temp >= 0 && temp < src_cols)
                {
                    left_line[t] = temp;
                }
            }
            fix_left_tail.pos_col = NO_DEFINE_VALUE;
            fix_left_head.pos_col = NO_DEFINE_VALUE;

            float k_right = (float) ((int16_t) right_line[fix_right_head.pos_col]
                    - (int16_t) right_line[fix_right_tail.pos_col]) / (fix_right_head.pos_col - fix_right_tail.pos_col);
            float b_right = right_line[fix_right_head.pos_col] - k_right * fix_right_head.pos_col;
            for (int t = fix_right_head.pos_col; t > fix_right_tail.pos_col; --t)
            {
                int temp = k_right * t + b_right;
                if (temp >= 0 && temp < src_cols)
                {
                    right_line[t] = temp;
                }
            }
            fix_right_head.pos_col = NO_DEFINE_VALUE;
            fix_right_tail.pos_col = NO_DEFINE_VALUE;
            road_type = NO_FIX_ROAD;
            break;
        }
        case IN_LEFT_ROTARY :
        {
            // 预先找突变点，在突变点之前绝不可以出现left_line[x] == right_line[x]的情况
            bool is_fix_out_rotary = false;
            for (int16_t i = src_rows - 5; i > 0 && left_line[i] != right_line[i]; --i)
            {
                int16_t right_offset = (int16_t) right_line[i] - (int16_t) right_line[i + 3];
                if (right_offset > 8 && i > (src_rows >> 2))
                {
                    is_fix_out_rotary = true;
                }
            }
            if (is_fix_out_rotary)
            {
                road_type = LEFT_ROTARY_OUT_FIRST_SUNKEN;
            }

            break;
        }
        case IN_RIGHT_ROTARY :
        {
            // 预先找突变点
            bool is_fix_out_rotary = false;
            for (int16_t i = src_rows - 5; i > 0 && left_line[i] != right_line[i]; --i)
            {
                int16_t left_offset = (int16_t) left_line[i + 3] - (int16_t) left_line[i];
                if (left_offset > 8 && i > (src_rows >> 2))
                {
                    is_fix_out_rotary = true;
                }
            }
            if (is_fix_out_rotary)
            {
                road_type = RIGHT_ROTARY_OUT_FIRST_SUNKEN;
            }

            break;
        }
        case LEFT_ROTARY_IN_FIRST_SUNKEN :
        {
            if (fix_left_tail.pos_col != NO_DEFINE_VALUE)
            {
                if (fix_left_tail.fix_point_type == ARC_LEFT)
                {
                    float k = (float) ((int16_t) left_line[fix_left_head.pos_col]
                            - (int16_t) left_line[fix_left_tail.pos_col])
                            / (float) (fix_left_head.pos_col - fix_left_tail.pos_col);
                    float b = left_line[fix_left_head.pos_col] - k * fix_left_head.pos_col;
                    for (int t = fix_left_head.pos_col; t >= fix_left_tail.pos_col; --t)
                    {
                        int temp = k * t + b;
                        if (temp >= 0 && temp < src_cols)
                        {
                            left_line[t] = temp;
                        }
                    }
                    fix_left_tail.pos_col = NO_DEFINE_VALUE;
                    fix_left_head.pos_col = NO_DEFINE_VALUE;
                }
                else if (fix_left_head.fix_point_type == ARC_LEFT && fix_left_tail.fix_point_type == CLIFF
                        && fix_left_tail.pos_col > 3)
                {
                    road_type = LEFT_ROTARY_IN_SECOND_SUNKEN;
                }
            }
            break;
        }
        case LEFT_ROTARY_IN_SECOND_SUNKEN :
        {
            if (fix_left_tail.pos_col != NO_DEFINE_VALUE)
            {
                if (go_in_rotary_stage_left == 0 && fix_left_tail.fix_point_type == CLIFF)
                {
                    // 引导线
                    int16_t top_angle_rows = fix_left_tail.pos_col;
                    int16_t top_angle_cols = left_line[top_angle_rows] - 16;
                    int16_t buttom_angle_rows = src_rows - 1;
                    int16_t button_angle_cols = right_line[buttom_angle_rows];
                    float k = (float) (top_angle_cols - button_angle_cols) / (top_angle_rows - buttom_angle_rows);
                    float line_a = top_angle_cols - top_angle_rows * k;
                    for (int16_t j = top_angle_rows; j < src_rows; ++j)
                    {
                        int temp = k * j + line_a;
                        if (temp >= 0 && temp < src_cols)
                        {
                            right_line[j] = temp;
                        }
                    }
                    if (top_angle_rows > src_rows / 2)
                    {
                        go_in_rotary_stage_left = 1;
                    }
                    // 重新找中线
                    int16_t mid_point = top_angle_cols - 6;
                    for (int16_t _j = top_angle_rows; _j >= 0; --_j)
                    {
                        int16_t cur_point = mid_point;
                        // 扫描左线
                        while (cur_point - 2 > 0)
                        {
                            left_line[_j] = 0;
                            if (src_pixel_mat[_j][cur_point] < threshold_val
                                    && src_pixel_mat[_j][cur_point - 1] < threshold_val
                                    && src_pixel_mat[_j][cur_point - 2] < threshold_val)
                            {
                                left_line[_j] = cur_point;
                                break;
                            }
                            --cur_point;
                        }
                        // 扫描右线
                        cur_point = mid_point;
                        while (cur_point + 2 < src_cols)
                        {
                            right_line[_j] = src_cols - 1;
                            if (src_pixel_mat[_j][cur_point] < threshold_val
                                    && src_pixel_mat[_j][cur_point + 1] < threshold_val
                                    && src_pixel_mat[_j][cur_point + 2] < threshold_val)
                            {
                                right_line[_j] = cur_point;
                                break;
                            }
                            ++cur_point;
                        }
                        mid_point = (right_line[_j] + left_line[_j]) / 2;
                    }
                }
                else if (go_in_rotary_stage_left == 1 && fix_right_tail.fix_point_type == CLIFF)
                {
                    int16_t top_angle_rows = fix_right_tail.pos_col;
                    int16_t top_angle_cols = right_line[top_angle_rows] - 16;
                    int16_t buttom_angle_rows = src_rows - 1;
                    int16_t button_angle_cols = right_line[buttom_angle_rows];
                    float k = (float) (top_angle_cols - button_angle_cols) / (top_angle_rows - buttom_angle_rows);
                    float line_a = top_angle_cols - top_angle_rows * k;
                    for (int16_t j = top_angle_rows; j < src_rows; ++j)
                    {
                        int temp = k * j + line_a;
                        if (temp >= 0 && temp < src_cols)
                        {
                            right_line[j] = temp;
                        }
                    }
                    // 重新找中线
                    int16_t mid_point = top_angle_cols - 6;
                    for (int16_t _j = top_angle_rows; _j >= 0; --_j)
                    {
                        int16_t cur_point = mid_point;
                        // 扫描左线
                        while (cur_point - 2 > 0)
                        {
                            left_line[_j] = 0;
                            if (src_pixel_mat[_j][cur_point] < threshold_val
                                    && src_pixel_mat[_j][cur_point - 1] < threshold_val
                                    && src_pixel_mat[_j][cur_point - 2] < threshold_val)
                            {
                                left_line[_j] = cur_point;
                                break;
                            }
                            --cur_point;
                        }
                        // 扫描右线
                        cur_point = mid_point;
                        while (cur_point + 2 < src_cols)
                        {
                            right_line[_j] = src_cols - 1;
                            if (src_pixel_mat[_j][cur_point] < threshold_val
                                    && src_pixel_mat[_j][cur_point + 1] < threshold_val
                                    && src_pixel_mat[_j][cur_point + 2] < threshold_val)
                            {
                                right_line[_j] = cur_point;
                                break;
                            }
                            ++cur_point;
                        }
                        mid_point = (right_line[_j] + left_line[_j]) / 2;
                    }
                }
            }
            else
            {
                road_type = IN_LEFT_ROTARY;
                go_in_rotary_stage_left = 0;
            }
            break;
        }
        case LEFT_ROTARY_OUT_FIRST_SUNKEN :
        {
            uint8_t right_line_begin_ = src_rows - 1;
            for (int i_find_right_line_begin_ = src_rows - 1; i_find_right_line_begin_ >= 0; --i_find_right_line_begin_)
            {
                if (right_line[i_find_right_line_begin_] != src_cols - 1)
                {
                    right_line_begin_ = i_find_right_line_begin_;
                    break;
                }
            }
            if (end_src_rows < 3 && right_line_begin_ > (src_rows - src_rows / 7))
            {
                StraightLineCoeffic straight_line_coeffic = LinearRegress(right_line, end_src_rows + 10,
                        right_line_begin_);
                uint8_t n = end_src_rows + 10;
                uint8_t num_err = 0;
                for (n; n < right_line_begin_; ++n)
                {
                    if (ABS(right_line[n] - (straight_line_coeffic.k * n + straight_line_coeffic.a)) > 4)
                    {
                        num_err++;
                    }
                }
                if (n >= right_line_begin_ - 1 && (n > src_rows >> 1 + 10) && num_err < 5)
                {
                    road_type = NO_FIX_ROAD;
                }
            }

            int16_t left_line_zero_min_rows = 0;
            int16_t j = src_rows - 1;
            float k_out_rotary = 2.9;
            float a_out_rotary = (int16_t) src_cols - k_out_rotary * (src_rows - 1);
            for (int16_t i = left_line_zero_min_rows; i < src_rows; ++i)
            {
                right_line[i] = k_out_rotary * i + a_out_rotary;
            }
            break;
        }
        case LEFT_ROTARY_OUT_SECOND_SUNKEN :
        {
            uint8_t left_line_start_ = src_rows - 1;
            uint8_t right_line_start_ = src_rows - 1;
            for (int16_t i = src_rows - 1; i > 0; --i)
            {
                if (left_line[i] != 0 && left_line[i - 1] != 0)
                {
                    left_line_start_ = i;
                }
                if (right_line[i] != src_cols - 1 && right_line[i - 1] != src_cols - 1)
                {
                    right_line_start_ = i;
                }
            }
            break;
        }
        case RIGHT_ROTARY_IN_FIRST_SUNKEN :
        {
            if (fix_right_tail.pos_col != NO_DEFINE_VALUE)
            {
                if (fix_right_tail.fix_point_type == ARC_RIGHT)
                {
                    float k = (float) ((int16_t) right_line[fix_right_head.pos_col]
                            - (int16_t) right_line[fix_right_tail.pos_col])
                            / (float) (fix_right_head.pos_col - fix_right_tail.pos_col);
                    float b = right_line[fix_right_head.pos_col] - k * fix_right_head.pos_col;
                    for (int t = fix_right_head.pos_col; t >= fix_right_tail.pos_col; --t)
                    {
                        int temp = k * t + b;
                        if (temp >= 0 && temp < src_cols)
                        {
                            right_line[t] = temp;
                        }
                    }
                    fix_right_tail.pos_col = NO_DEFINE_VALUE;
                    fix_right_head.pos_col = NO_DEFINE_VALUE;
                }
                else if (fix_right_head.fix_point_type == ARC_RIGHT && fix_right_tail.fix_point_type == CLIFF
                        && fix_right_tail.pos_col > 6)
                {
                    road_type = RIGHT_ROTARY_IN_SECOND_SUNKEN;
                }
            }
            break;
        }
        case RIGHT_ROTARY_IN_SECOND_SUNKEN :
        {
            if (fix_right_tail.pos_col != NO_DEFINE_VALUE)
            {
                if (go_in_rotary_stage_right == 0 && fix_right_tail.fix_point_type == CLIFF)
                {
                    // 引导线
                    int16_t top_angle_rows = fix_right_tail.pos_col;
                    int16_t top_angle_cols = right_line[top_angle_rows] + 16;
                    int16_t buttom_angle_rows = src_rows - 1;
                    int16_t button_angle_cols = left_line[buttom_angle_rows];
                    float k = (float) (top_angle_cols - button_angle_cols) / (top_angle_rows - buttom_angle_rows);
                    float line_a = top_angle_cols - top_angle_rows * k;
                    for (int16_t j = top_angle_rows; j < src_rows; ++j)
                    {
                        int temp = k * j + line_a;
                        if (temp >= 0 && temp < src_cols)
                        {
                            left_line[j] = temp;
                        }
                    }
                    if (top_angle_rows > src_rows / 2)
                    {
                        go_in_rotary_stage_right = 1;
                    }
                    // 重新找中线
                    int16_t mid_point = top_angle_cols + 6;
                    for (int16_t _j = top_angle_rows; _j >= 0; --_j)
                    {
                        int16_t cur_point = mid_point;
                        // 扫描左线
                        while (cur_point - 2 > 0)
                        {
                            left_line[_j] = 0;
                            if (src_pixel_mat[_j][cur_point] < threshold_val
                                    && src_pixel_mat[_j][cur_point - 1] < threshold_val
                                    && src_pixel_mat[_j][cur_point - 2] < threshold_val)
                            {
                                left_line[_j] = cur_point;
                                break;
                            }
                            --cur_point;
                        }
                        // 扫描右线
                        cur_point = mid_point;
                        while (cur_point + 2 < src_cols)
                        {
                            right_line[_j] = src_cols - 1;
                            if (src_pixel_mat[_j][cur_point] < threshold_val
                                    && src_pixel_mat[_j][cur_point + 1] < threshold_val
                                    && src_pixel_mat[_j][cur_point + 2] < threshold_val)
                            {
                                right_line[_j] = cur_point;
                                break;
                            }
                            ++cur_point;
                        }
                        mid_point = (right_line[_j] + left_line[_j]) / 2;
                    }
                }
                else if (go_in_rotary_stage_right == 1 && fix_left_tail.fix_point_type == CLIFF)
                {
                    int16_t top_angle_rows = fix_left_tail.pos_col;
                    int16_t top_angle_cols = left_line[top_angle_rows] + 16;
                    int16_t buttom_angle_rows = src_rows - 1;
                    int16_t button_angle_cols = left_line[buttom_angle_rows];
                    float k = (float) (top_angle_cols - button_angle_cols) / (top_angle_rows - buttom_angle_rows);
                    float line_a = top_angle_cols - top_angle_rows * k;
                    for (int16_t j = top_angle_rows; j < src_rows; ++j)
                    {
                        int temp = k * j + line_a;
                        if (temp >= 0 && temp < src_cols)
                        {
                            left_line[j] = temp;
                        }
                    }
                    // 重新找中线
                    int16_t mid_point = top_angle_cols + 6;
                    for (int16_t _j = top_angle_rows; _j >= 0; --_j)
                    {
                        int16_t cur_point = mid_point;
                        // 扫描左线
                        while (cur_point - 2 > 0)
                        {
                            left_line[_j] = 0;
                            if (src_pixel_mat[_j][cur_point] < threshold_val
                                    && src_pixel_mat[_j][cur_point - 1] < threshold_val
                                    && src_pixel_mat[_j][cur_point - 2] < threshold_val)
                            {
                                left_line[_j] = cur_point;
                                break;
                            }
                            --cur_point;
                        }
                        // 扫描右线
                        cur_point = mid_point;
                        while (cur_point + 2 < src_cols)
                        {
                            right_line[_j] = src_cols - 1;
                            if (src_pixel_mat[_j][cur_point] < threshold_val
                                    && src_pixel_mat[_j][cur_point + 1] < threshold_val
                                    && src_pixel_mat[_j][cur_point + 2] < threshold_val)
                            {
                                right_line[_j] = cur_point;
                                break;
                            }
                            ++cur_point;
                        }
                        mid_point = (right_line[_j] + left_line[_j]) / 2;
                    }
                }
            }
            else
            {
                road_type = IN_RIGHT_ROTARY;
                go_in_rotary_stage_right = 0;
            }
            break;
        }
        case RIGHT_ROTARY_OUT_FIRST_SUNKEN :
        {
            uint8_t left_line_begin_ = src_rows - 1;
            for (int i_find_left_line_begin_ = src_rows - 1; i_find_left_line_begin_ >= 0; --i_find_left_line_begin_)
            {
                if (left_line[i_find_left_line_begin_] != 0)
                {
                    left_line_begin_ = i_find_left_line_begin_;
                    break;
                }
            }
            if (end_src_rows < 3 && left_line_begin_ > (src_rows - src_rows / 7))
            {
                StraightLineCoeffic straight_line_coeffic = LinearRegress(left_line, end_src_rows + 15,
                        left_line_begin_);
                uint8_t n = end_src_rows + 15;
                uint8_t num_err = 0;
                for (n; n < left_line_begin_; ++n)
                {
                    if (ABS(left_line[n] - (straight_line_coeffic.k * n + straight_line_coeffic.a)) > 4)
                    {
                        num_err++;
                    }
                }
                if (n >= left_line_begin_ - 1 && (n > src_rows >> 1 + 10) && num_err < 5)
                {
                    road_type = NO_FIX_ROAD;
                }
            }

            float k_out_rotary = -2.9;
            float a_out_rotary = -k_out_rotary * (src_rows - 1);
            for (int16_t i = 0; i < src_rows; ++i)
            {
                left_line[i] = k_out_rotary * i + a_out_rotary;
            }
        }
        case RIGHT_ROTARY_OUT_SECOND_SUNKEN :
            break;
        case IN_CARBARN :
        {
            float k_line = 1.9;
            float a_line = 0;

            for (uint8_t i = 0; i < src_rows - 1; ++i)
            {
                int16_t temp = i * k_line + a_line;
                if (temp >= 0 && temp < src_cols)
                    right_line[i] = temp;
                left_line[i] = 0;
            }
            break;
        }
        default :
            break;
    }
}
bool inline IsEdge (uint16_t ConKernel[5], uint16_t color)
{
    if (ConKernel[4] != color)
        return false;
    for (int i = 0; i < 3; i++)
    {
        if (ConKernel[i] != ConKernel[i + 1])
            return true;
    }
    return false;
}

#ifdef UPPER_COMPUTER
void EdgeDetect(uint8_t **binary_img, uint8_t **edge_img, size_t src_rows, size_t src_cols)
#endif // UPPER_COMPUTER
#ifdef LOWER_COMPUTER
void EdgeDetect (uint8_t **binary_img, uint8_t **edge_img, uint8_t src_rows, uint8_t src_cols)
#endif // LOWER_COMPUTER
{
    static uint16_t ConKernel[5]; //边缘卷积核
    for (uint16_t i = 0; i < src_rows; i++)
    {
        for (uint16_t j = 0; j < src_cols; j++)
        {
            if ((i - 1) >= 0 && (i + 1) < src_rows && (j - 1) >= 0 && (j + 1) < src_cols)
            {
                ConKernel[0] = binary_img[i - 1][j];
                ConKernel[1] = binary_img[i + 1][j];
                ConKernel[2] = binary_img[i][j - 1];
                ConKernel[3] = binary_img[i][j + 1];
                ConKernel[4] = binary_img[i][j];
                if (IsEdge(ConKernel, 0))
                    edge_img[i][j] = 0; //边界颜色为黑
                else
                    edge_img[i][j] = 255;
            }
            else
                edge_img[i][j] = 255;
        }
    }
}

#ifdef UPPER_COMPUTER
uint8_t FindStraightLine(size_t *mid_line, size_t src_rows, size_t src_cols, std::string &output)
#endif // UPPER_COMPUTER
#ifdef LOWER_COMPUTER
uint8_t FindStraightLine (uint8_t *mid_line, uint8_t src_rows, uint8_t src_cols)
#endif // LOWER_COMPUTER
{
    // Hough 变换: r = x * cos(angle) + y * sin(angle)
    // 对中线前一段进行霍夫变换
    StraightLineCoeffic straight_line_coeffic = LinearRegress(mid_line, src_rows - 5, src_rows - 1);
    float k = straight_line_coeffic.k;
    float angle = atan(-k);
    float r0 = mid_line[src_rows - 1] * cos(angle) + (src_rows - 1) * sin(angle);
    int i = 2;
    while (i < src_rows - 1)
    {
        float r = mid_line[src_rows - i] * cos(angle) + (src_rows - i) * sin(angle);
        if (ABS(r - r0) > 5)
        {
            if (i > 12)
                return (src_rows - i);
            else
                return src_rows - 1;
        }
        ++i;
    }
    return i;
}

// 确定急转弯路口的方位
uint8_t _kind = 0; // 1 是左边，2是右边

//#define TEST_DEGE_DETECT
#ifdef UPPER_COMPUTER
size_t left_line[70];
size_t mid_line[70];
size_t right_line[70];
UserProcessRet UserProcess(unsigned char **src_pixel_mat, size_t src_rows, size_t src_cols, UserRGB **user_rgb_mat,
                           LineArray *left_line_, LineArray *mid_line_, LineArray *right_line_,
                           bool &is_show_left_line, bool &is_show_mid_line, bool &is_show_right_line,
                           unsigned int &threshold_value,
                           const double *const *perspective_transform_mat,
                           const std::string &user_data_for_input, std::string &user_data_for_output,
                           double *slope)
#endif // UPPER_COMPUTER
#ifdef LOWER_COMPUTER
void UserProcess (uint8_t *left_line, uint8_t *mid_line, uint8_t *right_line, uint8 src_rows, uint8_t src_cols,
        uint8_t threshold_value, float *slope)
#endif // LOWER_COMPUTER
{

#ifdef LOWER_COMPUTER
    uint8_t end_src_rows = 0;
    for (int i = src_rows - 1; i >= 0; --i)
    {
        if (ABS(left_line[i] - (int)right_line[i]) < 3)
        {
            end_src_rows = i;
            break;
        }
    }
#endif // LOWER_COMPUTER

#define MIN_START_CONTINUE_LINE 40
    FixRoad(left_line, right_line, src_rows, src_cols, end_src_rows, threshold_value);

    bool is_finish_end_src = false;
    for (int16_t i = src_rows - 1; i >= 0; --i)
    {
        mid_line[i] = (left_line[i] + right_line[i]) / 2;
        if (!is_finish_end_src && ABS(right_line[i] - (int)left_line[i]) < 3)
        {
            end_src_rows = i;
            is_finish_end_src = true;
        }
    }

    uint8_t start_line_right = 0;
    uint8_t end_line_right = 0;
    uint8_t start_line_left = 0;
    uint8_t end_line_left = 0;
    // CorrectLRLine(left_line, right_line, src_rows, src_cols);
    for (uint8_t i = src_rows - 1; i >= 2; --i)
    {
        if (right_line[i] != src_cols - 1 && right_line[i - 1] != src_cols - 1 && right_line[i - 2] != src_cols - 1
                && start_line_right == 0)
        {
            start_line_right = i;
        }
        else if (right_line[i] == src_cols - 1 && right_line[i - 1] == src_cols - 1 && right_line[i - 2] == src_cols - 1
                && end_line_right == 0 && start_line_right != 0)
        {
            end_line_right = i;
        }
        if (left_line[i] != 0 && left_line[i - 1] != 0 && left_line[i - 2] != 0 && start_line_left == 0)
        {
            start_line_left = i;
        }
        else if (left_line[i] == 0 && left_line[i - 1] == 0 && left_line[i - 2] == 0 && end_line_left == 0
                && start_line_left != 0)
        {
            end_line_left = i;
        }

#ifdef UPPER_COMPUTER
        CHECK_RANGE(left_line[i], 0, src_cols, user_data_for_output);
        CHECK_RANGE(mid_line[i], 0, src_cols, user_data_for_output);
        CHECK_RANGE(right_line[i], 0, src_cols, user_data_for_output);
#endif // UPPER_COMPUTER
    }
    // 注意end_continue_line 小于或等于 start_continue_line
    uint8_t end_continue_line = 0;
    uint8_t start_continue_line =
            start_line_right - end_line_right > start_line_left - end_line_left ?
                    (end_continue_line = end_line_right, start_line_right) :
                    (end_continue_line = end_line_left, start_line_left);
    int16_t steer_ouput = 0;

    uint16_t quadratic_start_index = 0;
    uint16_t quadratic_end_index = 0;
    // 假如start_line 过小(小于某个阈值), 那么可以看为目前小车所在的路段无左右边线, 舵机保持角度
    if (start_continue_line >= MIN_START_CONTINUE_LINE && start_continue_line > end_continue_line)
    {
        // 是否存在直线
#ifdef UPPER_COMPUTER
        uint8_t straightline_end_point = FindStraightLine(mid_line, src_rows, src_cols, user_data_for_output);
        user_data_for_output += "straight_line:" + std::to_string(straightline_end_point) + "\r\n";
#endif // UPPER_COMPUTER
#ifdef LOWER_COMPUTER
        uint8_t straightline_end_point = FindStraightLine(mid_line, src_rows, src_cols);
#endif // LOWER_COMPUTER

        if (straightline_end_point < src_rows - 1)
        {
            StraightLineCoeffic straight_line_coeffic = LinearRegress(mid_line, straightline_end_point, src_rows - 1);
            for (uint8_t i = straightline_end_point; i < src_rows - 1; ++i)
            {
                float temp = straight_line_coeffic.k * i + straight_line_coeffic.a;
                if ((int) temp >= 0 && (int) temp < src_cols)
                {
                    mid_line[i] = temp;
                }
            }
        }

        int16_t se_offset = start_continue_line - end_continue_line;
        quadratic_start_index = end_continue_line <= end_src_rows ? end_src_rows + 1 : start_continue_line; // 注意quadratic_start_index 选择小的线
        quadratic_end_index = start_continue_line;
        if (straightline_end_point != src_rows - 1 && straightline_end_point > quadratic_start_index)
        {
            quadratic_end_index = straightline_end_point;
        }
        if (road_type == IN_LEFT_ROTARY || road_type == IN_RIGHT_ROTARY)
        {
            quadratic_end_index = src_rows >> 1;
        }

        //// test: 拟合二次曲线

        //        if (quadratic_end_index > quadratic_start_index)
        //        {
        //            QuadraticCoeffic coeffic_one = QuadraticCurveFit(mid_line, quadratic_start_index, quadratic_end_index);
        //            for (uint8_t i = quadratic_start_index; i <= quadratic_end_index; ++i)
        //            {
        //                mid_line[i] = coeffic_one.a2 * i * i + coeffic_one.a1 * i + coeffic_one.a0;
        //            }
        //        }

        // 计算赛道中线斜率
        /*if (se_offset > 15)
         {
         quadratic_end_index = quadratic_start_index + se_offset >> 1;
         }
         StraightLineCoeffic straight_line_coeffic = LinearRegress(mid_line, quadratic_start_index, quadratic_end_index);
         for (uint8_t i = quadratic_start_index; i <= quadratic_end_index; ++i)
         {
         mid_line[i] = straight_line_coeffic.k * i + straight_line_coeffic.a;
         }*/
        // 计算曲率
        // float curve = CurvatureCal(mid_line, quadratic_start_index, quadratic_end_index);


        // 急转弯识别
        _kind = 0;
        if (road_type_for_control == NO_FIX_ROAD && end_src_rows > 3
                && ABS(right_line[end_src_rows] - (int16_t)left_line[end_src_rows]) < 3 &&
                ABS(right_line[end_src_rows - 1] - (int16_t)left_line[end_src_rows - 1]) < 3 &&
                ABS(right_line[end_src_rows - 2] - (int16_t)left_line[end_src_rows - 2]) < 3)
        {
            uint8_t _num_white_boundary = 0;

            for (int16_t i = end_src_rows + 1; i < src_rows && i < src_rows; ++i)
            {
                if (_kind == 0 && left_line[i] == 0 && left_line[i + 1] == 0)
                {
                    _kind = 1;
                }
                if (_kind == 0 && right_line[i] == src_cols - 1 && right_line[i + 1] == src_cols - 1)
                {
                    _kind = 2;
                }
               if (_kind == 1)
               {
                   if (left_line[i] == 0)
                   {
                       _num_white_boundary++;
                   }
                   else
                       break;
               }
               if (_kind == 2)
               {
                   if (right_line[i] == src_cols - 1)
                   {
                       _num_white_boundary++;
                   }
                   else
                       break;
               }

            }
            if (_kind == 1)
            {
                // 查看右边线是否符合
                int16_t _negative_offset_num = 0;
                for (int16_t j = end_src_rows; j < src_rows - 1; ++j)
                {
                    if (right_line[j] - (int16_t) right_line[j + 1] > 0)
                    {
                        _negative_offset_num++;
                    }
                    if (_negative_offset_num > (src_rows - end_src_rows) / 8)
                    {
                        _kind = 0;
                        break;
                    }
                }
                if (_kind == 1)
                {
//                    int16_t _end_white_boundary = end_src_rows + _num_white_boundary + 2;
//                    if (_end_white_boundary > src_rows)
//                        _end_white_boundary = src_rows;
//                    int16_t _pos_tmp_end_src_rows = end_src_rows;
//                    int16_t _offset_line = right_line[_end_white_boundary] - mid_line[_end_white_boundary];
//                    for (int16_t i = end_src_rows; i < _end_white_boundary; ++i)
//                    {
//                        int _tmp_mid_line = right_line[i] - _offset_line;
//                        if (_tmp_mid_line < 0)
//                        {
//                            mid_line[i] = 0;
//                            _pos_tmp_end_src_rows = i;
//                        }
//                        else
//                        {
//                            mid_line[i] = _tmp_mid_line;
//                        }
//                    }
//                    end_src_rows = _pos_tmp_end_src_rows;
                    SteerPidChange(1, pid_steer.I, pid_steer.D);
                   // pwm_duty(ATOM0_CH4_P02_4, pwm_right );    // 右轮前进
                   // pwm_duty(ATOM0_CH5_P02_5, pwm_left - 900);    // 左轮前进
                }
            }
            if (_kind == 2)
            {
                // 查看左边线是否符合
                int16_t _posivtive_offset_num = 0;
                for (int16_t j = end_src_rows; j < src_rows - 1; ++j)
                {
                    if (left_line[j + 1] - (int16_t) left_line[j] > 0)
                    {
                        _posivtive_offset_num++;
                    }
                    if (_posivtive_offset_num > (src_rows - end_src_rows) / 8)
                    {
                        _kind = 0;
                        break;
                    }
                }
                if (_kind == 2)
                {
//                    int16_t _end_white_boundary = end_src_rows + _num_white_boundary + 2;
//                    if (_end_white_boundary > src_rows)
//                        _end_white_boundary = src_rows;
//                    int16_t _pos_tmp_end_src_rows = end_src_rows;
//                    int16_t _offset_line = mid_line[_end_white_boundary] - left_line[_end_white_boundary];
//                    for (int16_t i = end_src_rows; i < _end_white_boundary; ++i)
//                    {
//                        int _tmp_mid_line = left_line[i] + _offset_line;
//                        if (_tmp_mid_line > src_cols - 1)
//                        {
//                            mid_line[i] = src_cols - 1;
//                            _pos_tmp_end_src_rows = i;
//                        }
//                        else
//                        {
//                            mid_line[i] = _tmp_mid_line;
//                        }
//                    }
//                    end_src_rows = _pos_tmp_end_src_rows;
                    SteerPidChange(1, pid_steer.I, pid_steer.D);
                   // pwm_duty(ATOM0_CH4_P02_4, pwm_right - 900);    // 右轮前进
                   // pwm_duty(ATOM0_CH5_P02_5, pwm_left);    // 左轮前进
                }
            }

        }
        lcd_showint16(0, 5, end_src_rows);
        lcd_showint16(50, 5, _kind);
        // 透视变换中线
        for (uint8_t i = 0; i < src_rows; ++i)
        {
            float x, y, w;
            x = perspective_transform_mat[0][0] * mid_line[i] + perspective_transform_mat[0][1] * i
                    + perspective_transform_mat[0][2];
            y = perspective_transform_mat[1][0] * mid_line[i] + perspective_transform_mat[1][1] * i
                    + perspective_transform_mat[1][2];
            w = perspective_transform_mat[2][0] * mid_line[i] + perspective_transform_mat[2][1] * i
                    + perspective_transform_mat[2][2];
            int dst_col = (x / w);
            int dst_row = (y / w);
            {
                mid_line_perspective_transform[i].x = dst_col;
                mid_line_perspective_transform[i].y = dst_row;
            }
        }

        // 计算斜率
        uint8_t cur_cal_end_point = (src_rows >> 2) * 3 - 15;
        uint8_t cur_cal_start_point = src_rows - 2;
        cur_cal_end_point = (cur_cal_end_point < end_src_rows) ? (end_src_rows - 2) : cur_cal_end_point;
        if (cur_cal_end_point < (cur_cal_start_point - 15))
        {
            cur_cal_start_point = cur_cal_end_point + 15;
            cur_cal_start_point = (cur_cal_start_point > src_rows - 2) ? (src_rows - 2) : cur_cal_start_point;
        }

        int temp_curve_m = (mid_line_perspective_transform[cur_cal_start_point].y
                - mid_line_perspective_transform[(cur_cal_end_point)].y);

        float curve = 0;
        if (temp_curve_m != 0)
        {
            curve = (float) (src_cols / 2 - mid_line_perspective_transform[cur_cal_end_point].x) / temp_curve_m;
        }
        else
        {
            curve = (float) (mid_line_perspective_transform[cur_cal_start_point].x
                    - mid_line_perspective_transform[cur_cal_end_point].x) / 1;
        }

        int16_t offset = 0;
        for (int j = cur_cal_end_point; j < src_rows - 2; ++j)
        {
            offset += ((int16_t) (src_cols >> 1) - (int16_t) mid_line_perspective_transform[j].x);
        }

        curve = 1.9 * curve * curve * curve / 3 + 0.9 * curve / 3.0;
        curve += (offset
                / (float) ((mid_line_perspective_transform[(cur_cal_start_point)].y
                        - mid_line_perspective_transform[(cur_cal_end_point)].y)
                        * (mid_line_perspective_transform[(cur_cal_start_point)].y
                                - mid_line_perspective_transform[(cur_cal_end_point)].y))) / 4.4;
        // 提前入弯
        //        uint8_t _zero_line = end_src_rows;
        //        uint8_t _line_kind = 0;             // 1是左线，2是右线
        //        for (int16_t _i = end_src_rows; _i < src_rows; ++_i)
        //        {
        //            if (_line_kind == 0)
        //            {
        //                if (left_line[_i] == 0)
        //                    _zero_line = _i;
        //                else if (right_line[_i] == src_cols - 1)
        //                    _zero_line = _i;
        //                else
        //            }
        //        }

        // 检测是否缺线, 右正，左负
        int16_t no_line = 0;
        for (int16_t i = cur_cal_start_point; i >= cur_cal_end_point; --i)
        {
            if (left_line[i] == 0)
                --no_line;
            if (right_line[i] == src_cols - 1)
                ++no_line;
        }
        if ((cur_cal_start_point - cur_cal_end_point) - ABS(no_line) < 5 && road_type_for_control == NO_FIX_ROAD)
        {
            // 触发急转弯处理
            int _total_right_line_x = 0;
            if (no_line < 0)
            {
                // 统计右线靠摄像头方位的横坐标平均值
                for (int16_t _j = src_rows; _j > src_rows - 10; --_j)
                {
                    _total_right_line_x += right_line[_j];
                }
                _total_right_line_x -= 1400;
                curve += PID_Increase(&error_sharp_bend, &pid_sharp_bend, (float) _total_right_line_x, 0);
            }
            else
            {
                // 统计左线靠摄像头方位的横坐标平均值
                for (int16_t _j = src_rows; _j > src_rows - 10; --_j)
                {
                    _total_right_line_x += left_line[_j];
                }
                //_total_right_line_x -= 1400;
                _total_right_line_x -= 300;
                curve -= PID_Increase(&error_sharp_bend, &pid_sharp_bend, _total_right_line_x, 0);
            }

            // lcd_showint16(0, 5, _total_right_line_x);
        }
        curve *= (1.878 * ABS((float )no_line / (float )(cur_cal_start_point - cur_cal_end_point)));

        *slope = curve;
    }
}
