#ifndef __BSP_PID_H
#define	__BSP_PID_H
#include "stm32f1xx.h"
#include "usart.h"
#include "bsp_debug.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

typedef struct
{
    float target_val; // 目标值
    float actual_val; // 实际值

    float err;      // 当前偏差 e(k)
    float err_last; // 上一次偏差 e(k-1)
    float err_prev;   // 上上次偏差 e(k-2)，增量式PID使用

    float Kp, Ki, Kd; // PID参数

    float integral; // 积分值，位置式PID使用
    float output;   // PID输出值



} _pid;

extern _pid pid_speed, pid_speed2;
extern _pid pid_location;
extern _pid pid_angle;
extern _pid pid_turn;
extern _pid pid_maixcam;
void PID_param_init(void);
float Limit_Float(float val, float min, float max);
void PID_Reset(_pid *pid);

void set_pid_target(_pid *pid, float temp_val);
float get_pid_target(_pid *pid);
void set_p_i_d(_pid *pid, float p, float i, float d);


float location_pid_realize(_pid *pid, float actual_val);
float speed_pid_realize(_pid *pid, float actual_val);
float angle_pid_realize(_pid *pid, float actual_val);
float Angle_Error(float target, float actual);
#endif
