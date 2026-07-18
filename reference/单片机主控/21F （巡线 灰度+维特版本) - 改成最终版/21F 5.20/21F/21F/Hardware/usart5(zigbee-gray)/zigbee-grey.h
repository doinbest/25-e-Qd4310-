#ifndef __ZIGBEE_GREY_H
#define __ZIGBEE_GREY_H

#include "headfile.h"

/*
 * USART5
 * TX : PC12
 * RX : PD2
 */

#define ZIGBEE_GREY_CHANNEL_NUM      8
#define ZIGBEE_GREY_TX_BUF_SIZE      256

/*
 * 是否开启 printf 重定向到 UART5
 * 如果你的 bsp_debug.c 里已经有 fputc，那么这里改成 0
 * 否则会重复定义 fputc
 */
#define ZIGBEE_GREY_PRINTF_ENABLE    0

extern No_MCU_Sensor ZigbeeGrey_Sensor;

extern unsigned short ZigbeeGrey_Analog[ZIGBEE_GREY_CHANNEL_NUM];
extern unsigned short ZigbeeGrey_Normal[ZIGBEE_GREY_CHANNEL_NUM];
extern unsigned char Digtal;
extern unsigned short ZigbeeGrey_White[ZIGBEE_GREY_CHANNEL_NUM];
extern unsigned short ZigbeeGrey_Black[ZIGBEE_GREY_CHANNEL_NUM];

extern int result_angle; // 角度结果

void uart5_send_char(char ch);
void uart5_send_string(char *str);

void ZigbeeGrey_SendByte(uint8_t ch);
void ZigbeeGrey_SendString(char *str);
void ZigbeeGrey_SendArray(uint8_t *buf, uint16_t len);

void ZigbeeGrey_Init(void);
void ZigbeeGrey_Task_WithoutTick(void);
void ZigbeeGrey_Tick(void);
void ZigbeeGrey_Task_WithTick(void);

unsigned char ZigbeeGrey_GetDigital(void);
unsigned char ZigbeeGrey_GetAnalog(unsigned short *result);
unsigned char ZigbeeGrey_GetNormal(unsigned short *result);

#endif
