#ifndef __BSP_ENCODER_H
#define __BSP_ENCODER_H

#include "headfile.h"

typedef struct
{
    int32_t total_cnt;      // 总计数
    int16_t diff_cnt;       // 本次采样增量
    int16_t last_cnt;       // 上次CNT
    float speed_rps;        // 转每秒
    float speed_rpm;        // 转每分钟
    float speed_mm_s;       // 车轮线速度，单位 mm/s
} Encoder_t;

void Encoder_Init(void);
void Encoder_Update(void);
float Encoder_CountToDistanceMm(int32_t count);
float Encoder_CountToSpeedMmS(int16_t count);
float Encoder_GetLeftDistanceMm(void);
float Encoder_GetRightDistanceMm(void);
float Encoder_GetDistanceMm(void);
int32_t Encoder_GetAverageCount(void);
void Encoder_ResetDistance(void);


Encoder_t* Encoder_GetLeft(void);
Encoder_t* Encoder_GetRight(void);

extern Encoder_t encoder_left;
extern Encoder_t encoder_right;

extern int16_t Motor1_Speed, Motor2_Speed; // 速度环调试用
extern int16_t Basic_Speed; // 基础速度
extern int16_t Distance; // 累计距离，单位 mm

#endif
