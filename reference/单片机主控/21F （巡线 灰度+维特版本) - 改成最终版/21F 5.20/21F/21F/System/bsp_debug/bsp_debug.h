#ifndef __BSP_DEBUG_H
#define __BSP_DEBUG_H

#include "headfile.h"

#define UART_RX_BUFFER_SIZE   64

extern unsigned char UART1_RxBuffer[UART_RX_BUFFER_SIZE];
extern volatile unsigned char UART1_RxPtr;
extern uint8_t UART1_RxByte;
extern volatile uint8_t receive_cmd;

void uart1_FlushRxBuffer(void);
void Usart1_SendByte(uint8_t ch);
void Usart1_SendString(uint8_t *str);
void Usart1_SendArray(uint8_t *buf, uint16_t len);
void Uart1_StartReceiveIT(void);
uint8_t Uart1_GetCmdFlag(void);
void Uart1_ClearCmdFlag(void);

void Protocol_Datas_Proc(void);

#endif
