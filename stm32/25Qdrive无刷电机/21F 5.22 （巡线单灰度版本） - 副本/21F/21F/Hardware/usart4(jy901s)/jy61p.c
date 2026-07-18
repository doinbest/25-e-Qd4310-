#include "jy61p.h"

static uint8_t RxBuffer[11];
static volatile uint8_t RxState = 0;
static uint8_t RxIndex = 0;
float Roll, Pitch, Yaw;

void jy61p_ReceiveData(uint8_t RxData)
{
    uint8_t i;
    uint8_t sum = 0;

    if (RxState == 0)
    {
        if (RxData == 0x55)
        {
            RxBuffer[RxIndex] = RxData;
            RxState = 1;
            RxIndex = 1;
        }
    }
    else if (RxState == 1)
    {
        if (RxData == 0x53)
        {
            RxBuffer[RxIndex] = RxData;
            RxState = 2;
            RxIndex = 2;
        }
        else
        {
            RxState = 0;
            RxIndex = 0;
        }
    }
    else if (RxState == 2)
    {
        RxBuffer[RxIndex++] = RxData;
        if (RxIndex == 11)
        {
            for (i = 0; i < 10; i++)
            {
                sum += RxBuffer[i];
            }

            if (sum == RxBuffer[10])
            {
                Roll = ((int16_t)((int16_t)RxBuffer[3] << 8 | (int16_t)RxBuffer[2])) / 32768.0f * 180.0f;
                Pitch = ((int16_t)((int16_t)RxBuffer[5] << 8 | (int16_t)RxBuffer[4])) / 32768.0f * 180.0f;
                Yaw = ((int16_t)((int16_t)RxBuffer[7] << 8 | (int16_t)RxBuffer[6])) / 32768.0f * 180.0f;
            }

            RxState = 0;
            RxIndex = 0;
        }
    }
    else
    {
        RxState = 0;
        RxIndex = 0;
    }
}

void JY61p_HardwareZeroYaw(void)
{
    uint8_t unlock_cmd[5] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
    uint8_t zero_cmd[5] = {0xFF, 0xAA, 0x01, 0x04, 0x00};

    HAL_UART_Transmit(&huart4, unlock_cmd, 5, 100);
    HAL_Delay(10);
    HAL_UART_Transmit(&huart4, zero_cmd, 5, 100);
}

void JY61p_HardwareZero(void)
{
    uint8_t unlock_cmd[5] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
    uint8_t rp_zero_cmd[5] = {0xFF, 0xAA, 0x01, 0x08, 0x00};
    uint8_t yaw_zero_cmd[5] = {0xFF, 0xAA, 0x01, 0x04, 0x00};

    HAL_UART_Transmit(&huart4, unlock_cmd, 5, 100);
    HAL_Delay(10);

    HAL_UART_Transmit(&huart4, rp_zero_cmd, 5, 100);
    HAL_Delay(10);

    HAL_UART_Transmit(&huart4, yaw_zero_cmd, 5, 100);
}