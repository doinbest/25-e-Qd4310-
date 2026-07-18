#include "bsp_key.h"

uint8_t B1_State, B1_Last_State = 1;	// PB0
uint8_t B2_State, B2_Last_State = 1;	// PB1
uint8_t B3_State, B3_Last_State = 1;	// PB2
uint8_t B4_State = 1, B4_Last_State = 0;	// PC8	RCT6上的K3
uint8_t B5_State = 1, B5_Last_State = 0;	// PC9	RCT6上的K4
uint8_t B4_flag = 0; // 校准白色标志
uint8_t B5_flag = 0; // 校准黑色标志
uint8_t Calib_Step = 0;
uint8_t ZigbeeGrey_White_OK = 0;
uint8_t ZigbeeGrey_Black_OK = 0;
void key_scan(void)
{
	static uint32_t last_scan_time = 0;
	uint32_t current_time = HAL_GetTick();

	if (current_time - last_scan_time < 20)
	{
		return;
	}
	last_scan_time = current_time;

	B1_State = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
	B2_State = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
	B3_State = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
	B4_State = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8);
	B5_State = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9);

	// ================= B1：校准白色 =================
	if (B1_State == 0 && B1_Last_State == 1)
	{
		for (int i = 0; i < ZIGBEE_GREY_CHANNEL_NUM; i++)
		{
			ZigbeeGrey_White[i] = ZigbeeGrey_Analog[i];
		}

		ZigbeeGrey_White_OK = 1;
		Calib_Step = 1;

		uart5_send_string("White Values Calibrated and Saved!\r\n");
		uart5_send_string("Please put sensor on BLACK, then press B2.\r\n");
	}

	// ================= B2：校准黑色 =================
	if (B2_State == 0 && B2_Last_State == 1)
	{
		for (int i = 0; i < ZIGBEE_GREY_CHANNEL_NUM; i++)
		{
			ZigbeeGrey_Black[i] = ZigbeeGrey_Analog[i];
		}

		ZigbeeGrey_Black_OK = 1;
		Calib_Step = 2;

		uart5_send_string("Black Values Calibrated and Saved!\r\n");

		if (ZigbeeGrey_White_OK == 1 && ZigbeeGrey_Black_OK == 1)
		{
			uart5_send_string("White and Black Calibration Finished!\r\n");

			No_MCU_Ganv_Sensor_Init(&ZigbeeGrey_Sensor,
									ZigbeeGrey_White,
									ZigbeeGrey_Black);

			HAL_Delay(100);
		}
		else
		{
			uart5_send_string("Warning: WHITE not calibrated yet!\r\n");
		}
	}

	// ================= B3：重置黑白校准值 =================
	if (B3_State == 0 && B3_Last_State == 1)
	{
		if (B3_State == 0 && B3_Last_State == 1)
		{
			maixcam_SendPacket(0x01);
			task_state = 0;
			uart5_send_string("B3 Pressed: Send MaixCAM Cmd 0x01\r\n");
		}
	}

	// ================= B4：备用 =================
	if (B4_State == 0 && B4_Last_State == 1)
	{
		Calib_Step = 0;
		ZigbeeGrey_White_OK = 0;
		ZigbeeGrey_Black_OK = 0;

		for (int i = 0; i < ZIGBEE_GREY_CHANNEL_NUM; i++)
		{
			ZigbeeGrey_White[i] = 0;
			ZigbeeGrey_Black[i] = 0;
		}

		uart5_send_string("Calibration Reset!\r\n");
		uart5_send_string("Press B1 for WHITE, press B2 for BLACK.\r\n");
		// 备用
	}

	// ================= B5：备用 =================
	if (B5_State == 0 && B5_Last_State == 1)
	{
		// 正式逻辑中不再控制 car_go
	}

	B1_Last_State = B1_State;
	B2_Last_State = B2_State;
	B3_Last_State = B3_State;
	B4_Last_State = B4_State;
	B5_Last_State = B5_State;
}