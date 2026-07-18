#ifndef __CONTROL_H
#define __CONTROL_H

#include "headfile.h"

#define CAR_LENGTH 140      // 车长 14cm
#define DASH_LINE_OFFSET 50 // 虚线距离 5cm
#define CENTER_OFFSET 150   // 中心距离 15cm

#define NEAR_STRAIGHT_DISTANCE 600  // 近端直线距离 60cm
#define NEAR_TO_MIDDLE_DISTANCE 400 // 近中端转弯后直线距离 40cm

#define NEAR_RUN1_DISTANCE 850
// \(NEAR_STRAIGHT_DISTANCE + CENTER_OFFSET + CAR_LENGTH - DASH_LINE_OFFSET - 40)    // 60 + 15 + 14 - 5 = 84cm

#define NEAR_RUN2_DISTANCE 360
// \(NEAR_TO_MIDDLE_DISTANCE + CENTER_OFFSET - CAR_LENGTH - DASH_LINE_OFFSET)

#define NEAR_BACK1_DISTANCE -360

#define NEAR_BACK2_DISTANCE 565 
// \(NEAR_STRAIGHT_DISTANCE + CENTER_OFFSET - CAR_LENGTH - DASH_LINE_OFFSET - 15) // 60 + 15 - 14 - 5 = 56cm

#define MIDDLE_RUN1_DISTANCE 1800
#define MIDDLE_RUN2_DISTANCE 360
#define MIDDLE_BACK1_DISTANCE -360
#define MIDDLE_BACK2_DISTANCE 1465

#define MIDDLE_TO_FAR_DISTANCE 850

#define FAR_SCAN_FRONT_DISTANCE 650
#define FAR_SCAN_REMAIN_DISTANCE (MIDDLE_TO_FAR_DISTANCE - FAR_SCAN_FRONT_DISTANCE)
#define FAR_SCAN_TIME_MS 1000

#define FAR_RUN1_DISTANCE 910   // 第一次转弯后，直行到路口的距离
#define FAR_RUN2_DISTANCE 360   // 第二次转弯后，直行到病房的距离
#define FAR_BACK1_DISTANCE -430 // 后退的距离
#define FAR_BACK2_DISTANCE 900 // 从远端病房返回到转弯点的距离
#define FAR_BACK3_DISTANCE 2365 // 从转弯点返回到药房的距离


uint8_t Distance_task(int16_t distance);
void task_1(void);
void task_2(void);
void task_search_middle_then_far(void);
void Angle_Hold_Reset_To_Zero(void);

extern uint8_t task_state;
extern int result_angle; // 角度结果
extern uint8_t car_go;
extern uint8_t Location_flag;   
extern uint8_t Angle_flag;
extern uint8_t Gray_flag; // 灰度控制标志位
extern uint8_t number_flag; // 目标数字识别标志位
extern float wit_output; // wit转向修正量
extern uint8_t far_first_side;
extern uint8_t special_flag; // 特殊循迹调整标志位
extern int SpeedL;         // 左轮速度
extern int SpeedR;         // 右轮速度
extern int Output_Yaw;     // PID输出
#endif
