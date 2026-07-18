#ifndef __HEADFILE_H
#define	__HEADFILE_H

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "bsp_sys.h"
#include "bsp_delay.h"
#include "bsp_oled.h"
#include "bsp_debug.h"
#include "protocol.h"

#include "control.h"
#include "bsp_oled.h"
#include "bsp_key.h"
#include "bsp_pid.h"	 
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"
#include "zigbee-grey.h"    // 串口5
#include "jy61p.h"  // 串口4
#include "maixcam.h"  // 串口3

#include "drug.h" 

#define Run_Mode 0
#define Speed_Mode 1    // 速度环
#define Speed2_Mode 2    // 速度环
#define Location_Mode 3 // 位置环（定距离）
#define Angle_Mode 4    // 角度环（定角度）

extern float battery_voltage;
extern uint8_t Car_Mode; // 运行模式
extern uint8_t car_go; // 1 代表启动了，0 代表没有启动

#endif
