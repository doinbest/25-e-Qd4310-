#include "jy61p.h"

static uint8_t RxBuffer[11];		 /*接收数据数组*/
static volatile uint8_t RxState = 0; /*接收状态标志位*/
static uint8_t RxIndex = 0;			 /*接受数组索引*/
float Roll, Pitch, Yaw;				 /*角度信息，如果只需要整数可以改为整数类型*/

/**
 * @brief       数据包处理函数
 * @param       串口接收的数据RxData
 * @retval      无
 */
void jy61p_ReceiveData(uint8_t RxData)
{
	uint8_t i, sum = 0;
	if (RxState == 0) // 等待包头
	{
		if (RxData == 0x55) // 收到包头
		{
			RxBuffer[RxIndex] = RxData;
			RxState = 1;
			RxIndex = 1; // 进入下一状态
		}
	}
	else if (RxState == 1)
	{
		if (RxData == 0x53) /*判断数据内容，修改这里可以改变要读的数据内容，0x53为角度输出*/
		{
			RxBuffer[RxIndex] = RxData;
			RxState = 2;
			RxIndex = 2; // 进入下一状态
		}
	}
	else if (RxState == 2) // 接收数据
	{
		RxBuffer[RxIndex++] = RxData;
		if (RxIndex == 11) // 接收完成
		{
			for (i = 0; i < 10; i++)
			{
				sum = sum + RxBuffer[i]; // 计算校验和
			}
			if (sum == RxBuffer[10]) // 校验成功
			{
				/*计算数据，根据数据内容选择对应的计算公式*/
				Roll = ((int16_t)((int16_t)RxBuffer[3] << 8 | (int16_t)RxBuffer[2])) / 32768.0f * 180.0f;
				Pitch = ((int16_t)((int16_t)RxBuffer[5] << 8 | (int16_t)RxBuffer[4])) / 32768.0f * 180.0f;
				Yaw = ((int16_t)((int16_t)RxBuffer[7] << 8 | (int16_t)RxBuffer[6])) / 32768.0f * 180.0f;
			}
			RxState = 0;
			RxIndex = 0; // 读取完成，回到最初状态，等待包头
		}
	}
}

/**
 * @brief  向维特模块发送 Z轴归零指令
 * @note   需要你自己补充串口发送函数 (如 USART_SendArray 或 HAL_UART_Transmit)
 */
void JY61p_HardwareZeroYaw(void)
{
	// 1. 解锁配置寄存器 (协议固定: FF AA 69 88 B5)
	uint8_t unlock_cmd[5] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};

	// 2. 发送 Z 轴清零指令 (协议固定: FF AA 01 04 00)
	uint8_t zero_cmd[5] = {0xFF, 0xAA, 0x01, 0x04, 0x00};

	/* ------------- 在下方填入你的串口发送代码 ------------- */
	// 例如STM32 HAL库：
	HAL_UART_Transmit(&huart4, unlock_cmd, 5, 100);
	HAL_Delay(10); // 建议稍微延时等待模块内部解锁
	HAL_UART_Transmit(&huart4, zero_cmd, 5, 100);

	// 例如标准库：
	// for(int i=0; i<5; i++) UART_SendByte(USART1, unlock_cmd[i]);
	// Delay_ms(10);
	// for(int i=0; i<5; i++) UART_SendByte(USART1, zero_cmd[i]);
	/* ---------------------------------------------------- */
}