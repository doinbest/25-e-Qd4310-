#include "bsp_encoder.h"
#include "tim.h"

#define SAMPLE_PERIOD_S   0.01f     // 10ms采样一次
#define ENCODER_PPR       11.0f     // 编码器每圈线数，按你的实际参数修改
#define GEAR_RATIO        50.0f     // 减速比，按你的实际参数修改
#define COUNT_PER_REV     (ENCODER_PPR * 4.0f * GEAR_RATIO)
// 如果编码器模式是4倍频，这里乘4

#define WHEEL_DIAMETER_MM 65.0f
#define PI_F 3.1415926f
#define WHEEL_CIRCUMFERENCE_MM (PI_F * WHEEL_DIAMETER_MM)   // 3.14 * 65 == 204.2mm

/*
 * 编码器方向修正：
 * 如果小车前进时某一侧 total_cnt 是负数，就把对应方向改成 -1。
 */
#define LEFT_ENCODER_DIR -1
#define RIGHT_ENCODER_DIR 1

Encoder_t encoder_left;
Encoder_t encoder_right;

int16_t Motor1_Speed, Motor2_Speed; // 速度环调试用
int16_t Basic_Speed;
int16_t Distance; // 累计距离，单位 mm
/**
  * @brief  读取编码器当前增量，并清零计数器
  * @param  htim: 编码器对应定时器
  * @retval 本采样周期内的编码器增量
  */
static int16_t Encoder_ReadCnt(TIM_HandleTypeDef *htim)
{
    int16_t cnt;

    cnt = (int16_t)__HAL_TIM_GET_COUNTER(htim);
    __HAL_TIM_SET_COUNTER(htim, 0);

    return cnt;
}

void Encoder_Init(void)
{
    // 左编码器：TIM2
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    // 右编码器：TIM4
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim4, 0);

    encoder_left.total_cnt = 0;
    encoder_left.diff_cnt = 0;
    encoder_left.speed_rps = 0.0f;
    encoder_left.speed_rpm = 0.0f;

    encoder_right.total_cnt = 0;
    encoder_right.diff_cnt = 0;
    encoder_right.speed_rps = 0.0f;
    encoder_right.speed_rpm = 0.0f;
}

void Encoder_Update(void)
{
    // 读取本周期增量
    encoder_left.diff_cnt =  LEFT_ENCODER_DIR  * Encoder_ReadCnt(&htim2);
    encoder_right.diff_cnt = RIGHT_ENCODER_DIR * Encoder_ReadCnt(&htim4);

    // 累加总计数
    encoder_left.total_cnt  += encoder_left.diff_cnt;
    encoder_right.total_cnt += encoder_right.diff_cnt;

    // 计算转速
    encoder_left.speed_rps  = (float)encoder_left.diff_cnt / COUNT_PER_REV / SAMPLE_PERIOD_S;
    encoder_right.speed_rps = (float)encoder_right.diff_cnt / COUNT_PER_REV / SAMPLE_PERIOD_S;                              

    encoder_left.speed_rpm  = encoder_left.speed_rps * 60.0f;
    encoder_right.speed_rpm = encoder_right.speed_rps * 60.0f;

    // pid_speed.actual_val = encoder_left.speed_rps;
    // pid_speed2.actual_val = encoder_right.speed_rps; 
    Motor1_Speed = encoder_left.diff_cnt;
    Motor2_Speed = encoder_right.diff_cnt;
    Distance = Encoder_GetDistanceMm();   
}

Encoder_t* Encoder_GetLeft(void)
{
    return &encoder_left;
}

Encoder_t* Encoder_GetRight(void)
{
    return &encoder_right;
}

/**
 * @brief  编码器计数转换为轮子行驶距离
 * @param  cnt: 编码器累计计数
 * @retval 距离，单位 mm
 */
float Encoder_CountToDistanceMm(int32_t cnt)
{
    return ((float)cnt / COUNT_PER_REV) * WHEEL_CIRCUMFERENCE_MM;
}

/**
 * @brief  距离转换为编码器目标计数
 * @param  distance_mm: 目标距离，单位 mm
 * @retval 对应的编码器计数
 */
int32_t Encoder_DistanceMmToCount(float distance_mm)
{
    return (int32_t)(distance_mm / WHEEL_CIRCUMFERENCE_MM * COUNT_PER_REV);
}

/**
 * @brief  获取左轮累计距离
 * @retval 左轮距离，单位 mm
 */
float Encoder_GetLeftDistanceMm(void)
{
    return Encoder_CountToDistanceMm(encoder_left.total_cnt);
}

/**
 * @brief  获取右轮累计距离
 * @retval 右轮距离，单位 mm
 */
float Encoder_GetRightDistanceMm(void)
{
    return Encoder_CountToDistanceMm(encoder_right.total_cnt);
}

/**
 * @brief  获取小车中心累计距离
 * @note   差速车中心距离 = 左右轮距离平均值
 * @retval 小车行驶距离，单位 mm
 */
float Encoder_GetDistanceMm(void)
{
    float left_distance;
    float right_distance;

    left_distance = Encoder_GetLeftDistanceMm();
    right_distance = Encoder_GetRightDistanceMm();

    return (left_distance + right_distance) / 2.0f;
}

/**
 * @brief  获取左右轮平均编码器计数
 * @retval 平均计数
 */
int32_t Encoder_GetAverageCount(void)
{
    return (encoder_left.total_cnt + encoder_right.total_cnt) / 2;
}

/**
 * @brief  清零距离累计
 */
// void Encoder_ResetDistance(void)
// {
//     __HAL_TIM_SET_COUNTER(&htim2, 0);
//     __HAL_TIM_SET_COUNTER(&htim4, 0);

//     encoder_left.total_cnt = 0;
//     encoder_left.diff_cnt = 0;
//     encoder_left.speed_rps = 0.0f;
//     encoder_left.speed_rpm = 0.0f;

//     encoder_right.total_cnt = 0;
//     encoder_right.diff_cnt = 0;
//     encoder_right.speed_rps = 0.0f;
//     encoder_right.speed_rpm = 0.0f;

//     pid_location.integral = 0;
//     pid_location.err_last = 0;
//     pid_location.output = 0;
// }

void Encoder_ResetDistance(void)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);

    encoder_left.total_cnt = 0;
    encoder_left.diff_cnt = 0;
    encoder_left.speed_rps = 0.0f;
    encoder_left.speed_rpm = 0.0f;

    encoder_right.total_cnt = 0;
    encoder_right.diff_cnt = 0;
    encoder_right.speed_rps = 0.0f;
    encoder_right.speed_rpm = 0.0f;

    Motor1_Speed = 0;
    Motor2_Speed = 0;
    Distance = 0;

    pid_location.actual_val = 0.0f;
    pid_location.err = 0.0f;
    pid_location.err_last = 0.0f;
    pid_location.err_prev = 0.0f;
    pid_location.integral = 0.0f;
    pid_location.output = 0.0f;
}
