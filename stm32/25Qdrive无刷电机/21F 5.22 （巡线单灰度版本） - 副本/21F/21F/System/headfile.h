#ifndef __HEADFILE_H
#define	__HEADFILE_H

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
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

extern float battery_voltage;

/* Wildfire/debug mode order: gimbal modes first, then retained chassis modes. */
#define Run_Mode 0U
#define X_Rate_Mode 1U         /* X IMU differentiated angular-rate PID */
#define Y_Rate_Mode 2U         /* Y IMU differentiated angular-rate PID */
#define X_Mode 3U              /* X pixel outer-loop PID */
#define Y_Mode 4U              /* Y pixel outer-loop PID */
#define PID_MODE_COUNT 10U

#define Speed_Mode 5U
#define Speed2_Mode 6U
#define Location_Mode 7U
#define Angle_Mode 8U
#define Gray_Mode 9U
extern uint8_t Car_Mode; // 运行模式
extern uint8_t control_running; // 1 代表启动了，0 代表没有启动

#endif
