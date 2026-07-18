#include "zigbee-grey.h"

/*
 * USART5
 * TX : PC12
 * RX : PD2
 */

/******************** 灰度传感器变量 ********************/

No_MCU_Sensor ZigbeeGrey_Sensor;

unsigned short ZigbeeGrey_Analog[ZIGBEE_GREY_CHANNEL_NUM] = {0};
unsigned short ZigbeeGrey_Normal[ZIGBEE_GREY_CHANNEL_NUM] = {0};
unsigned char Digtal = 0;
volatile uint32_t ZigbeeGrey_SampleCount = 0;

static unsigned char ZigbeeGrey_TxBuf[ZIGBEE_GREY_TX_BUF_SIZE] = {0};

/*
 * 黑白校准值
 * 这里先沿用当前工程的默认值
 * 后期你可以根据实际传感器数据修改
 */
unsigned short ZigbeeGrey_White[ZIGBEE_GREY_CHANNEL_NUM] =
    {2189, 2115, 1510, 2500, 2142, 929, 974, 1294};

unsigned short ZigbeeGrey_Black[ZIGBEE_GREY_CHANNEL_NUM] =
    {556, 459, 365, 544, 611, 177, 248, 235};

/******************** UART5 发送函数 ********************/

void uart5_send_char(char ch)
{
    HAL_UART_Transmit(&huart5, (uint8_t *)&ch, 1, 0xffff);
}

void uart5_send_string(char *str)
{
    if (str == NULL)
    {
        return;
    }

    while (*str != '\0')
    {
        uart5_send_char(*str++);
    }
}

void ZigbeeGrey_SendByte(uint8_t ch)
{
    HAL_UART_Transmit(&huart5, &ch, 1, 1000);
}

void ZigbeeGrey_SendString(char *str)
{
    uart5_send_string(str);
}

void ZigbeeGrey_SendArray(uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len == 0)
    {
        return;
    }

    HAL_UART_Transmit(&huart5, buf, len, 1000);
}

/******************** printf 重定向到 UART5 ********************/
/*
 * 注意：
 * 1. 如果 bsp_debug.c 里已经有 fputc，需要删除旧的或者把 ZIGBEE_GREY_PRINTF_ENABLE 改成 0
 * 2. 全工程只能有一个 fputc
 * 3. 这里把 printf 输出重定向到 UART5
 */

#if ZIGBEE_GREY_PRINTF_ENABLE

#pragma import(__use_no_semihosting)

struct __FILE
{
    int handle;
};

FILE __stdout;

void _sys_exit(int x)
{
    x = x;
}

int fputc(int ch, FILE *f)
{
    /*
     * 等待发送数据寄存器为空
     * UART5 是 STM32F103 的串口5外设名
     * 不是 USART5
     */
    while ((UART5->SR & USART_SR_TXE) == 0)
    {
    }

    UART5->DR = (uint8_t)ch;

    return ch;
}

#endif

/******************** 灰度模块初始化 ********************/

void ZigbeeGrey_Init(void)
{
    memset(ZigbeeGrey_TxBuf, 0, ZIGBEE_GREY_TX_BUF_SIZE);

    sprintf((char *)ZigbeeGrey_TxBuf, "hello_world!\r\n");
    // uart5_send_string((char *)ZigbeeGrey_TxBuf);
    memset(ZigbeeGrey_TxBuf, 0, ZIGBEE_GREY_TX_BUF_SIZE);

    /*
     * 第一次初始化，不带黑白校准值
     */
    No_MCU_Ganv_Sensor_Init_Frist(&ZigbeeGrey_Sensor);

    /*
     * 先采集一次原始模拟量
     */
    No_Mcu_Ganv_Sensor_Task_With_tick(&ZigbeeGrey_Sensor);

    Get_Anolog_Value(&ZigbeeGrey_Sensor, ZigbeeGrey_Analog);

    sprintf((char *)ZigbeeGrey_TxBuf,
            "Anolog %d-%d-%d-%d-%d-%d-%d-%d\r\n",
            ZigbeeGrey_Analog[0],
            ZigbeeGrey_Analog[1],
            ZigbeeGrey_Analog[2],
            ZigbeeGrey_Analog[3],
            ZigbeeGrey_Analog[4],
            ZigbeeGrey_Analog[5],
            ZigbeeGrey_Analog[6],
            ZigbeeGrey_Analog[7]);

    // uart5_send_string((char *)ZigbeeGrey_TxBuf);

    HAL_Delay(100);

    memset(ZigbeeGrey_TxBuf, 0, ZIGBEE_GREY_TX_BUF_SIZE);

    No_MCU_Ganv_Sensor_Init(&ZigbeeGrey_Sensor,
                            ZigbeeGrey_White,
                            ZigbeeGrey_Black);
    HAL_Delay(100);
}

/******************** 灰度模块循环任务 ********************/

/**
  * @brief  计算角度偏移量
  * @param  Normal: 输入的8个unsigned short类型数据数组 field：1白底黑场 0黑底白场
  * @retval 计算得到的归一化返回值 返回值理论范围-7*1024~7*1024
  */
int32_t CalculateNormalizedValue(unsigned short Normal[8], uint8_t field)
{
    const short weights[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
    int32_t weighted_sum = 0;
    int32_t original_sum = 0;
    static int last_value = 0;

    for (int i = 0; i < 8; i++)
    {
        weighted_sum += 1024 * (field ? (4096 - Normal[i]) * weights[i] : Normal[i] * weights[i]);
        original_sum += field ? (4096 - Normal[i]) : Normal[i];
    }

    if (original_sum != 0)
    {
        last_value = weighted_sum / original_sum;
        return last_value;
    }
    else
    {
        return last_value;
    }
}

void ZigbeeGrey_Tick(void)
{
    Task_tick(&ZigbeeGrey_Sensor);
}

void ZigbeeGrey_Task_WithTick(void)
{
    uint8_t sample_updated = (ZigbeeGrey_Sensor.Tick >= ZigbeeGrey_Sensor.Time_out) ? 1U : 0U;
    /*
     * 1. 有时基传感器常规任务
     * 内部包含：模拟量采集、数字量转换、归一化处理
     */
    No_Mcu_Ganv_Sensor_Task_With_tick(&ZigbeeGrey_Sensor);

    if (sample_updated != 0U)
    {
        ZigbeeGrey_SampleCount++;
    }

    /*
     * 2. 获取数字量
     */
    Digtal = Get_Digtal_For_User(&ZigbeeGrey_Sensor);
#if ZIGBEE_GREY_PRINTF_ENABLE
    sprintf((char *)ZigbeeGrey_TxBuf,
            "Digtal %d-%d-%d-%d-%d-%d-%d-%d\r\n",
            (Digtal >> 0) & 0x01,
            (Digtal >> 1) & 0x01,
            (Digtal >> 2) & 0x01,
            (Digtal >> 3) & 0x01,
            (Digtal >> 4) & 0x01,
            (Digtal >> 5) & 0x01,
            (Digtal >> 6) & 0x01,
            (Digtal >> 7) & 0x01);
    uart5_send_string((char *)ZigbeeGrey_TxBuf);
    memset(ZigbeeGrey_TxBuf, 0, ZIGBEE_GREY_TX_BUF_SIZE);
#endif

    /*
     * 3. 获取模拟量
     */
    if (Get_Anolog_Value(&ZigbeeGrey_Sensor, ZigbeeGrey_Analog))
    {
#if ZIGBEE_GREY_PRINTF_ENABLE
        sprintf((char *)ZigbeeGrey_TxBuf,
                "Anolog %d-%d-%d-%d-%d-%d-%d-%d\r\n",
                ZigbeeGrey_Analog[0], ZigbeeGrey_Analog[1],
                ZigbeeGrey_Analog[2], ZigbeeGrey_Analog[3],
                ZigbeeGrey_Analog[4], ZigbeeGrey_Analog[5],
                ZigbeeGrey_Analog[6], ZigbeeGrey_Analog[7]);
        uart5_send_string((char *)ZigbeeGrey_TxBuf);
        memset(ZigbeeGrey_TxBuf, 0, ZIGBEE_GREY_TX_BUF_SIZE);
#endif
    }

    /*
     * 4. 获取归一化结果
     */
    if (Get_Normalize_For_User(&ZigbeeGrey_Sensor, ZigbeeGrey_Normal))
    {
#if ZIGBEE_GREY_PRINTF_ENABLE
        sprintf((char *)ZigbeeGrey_TxBuf,
                "Normalize %d-%d-%d-%d-%d-%d-%d-%d\r\n",
                ZigbeeGrey_Normal[0], ZigbeeGrey_Normal[1],
                ZigbeeGrey_Normal[2], ZigbeeGrey_Normal[3],
                ZigbeeGrey_Normal[4], ZigbeeGrey_Normal[5],
                ZigbeeGrey_Normal[6], ZigbeeGrey_Normal[7]);
        uart5_send_string((char *)ZigbeeGrey_TxBuf);
        memset(ZigbeeGrey_TxBuf, 0, ZIGBEE_GREY_TX_BUF_SIZE);
#endif
    }

    result_angle = CalculateNormalizedValue(ZigbeeGrey_Normal, 1);
#if ZIGBEE_GREY_PRINTF_ENABLE
    sprintf((char *)ZigbeeGrey_TxBuf,
            "Angle %d\r\n",
            result_angle);
    uart5_send_string((char *)ZigbeeGrey_TxBuf);
    memset(ZigbeeGrey_TxBuf, 0, ZIGBEE_GREY_TX_BUF_SIZE);
#endif
}

unsigned char ZigbeeGrey_GetDigital(void)
{
    return Digtal;

}

unsigned char ZigbeeGrey_GetAnalog(unsigned short *result)
{
    if (result == NULL)
    {
        return 0;
    }

    memcpy(result, ZigbeeGrey_Analog, sizeof(ZigbeeGrey_Analog));

    return 1;
}

unsigned char ZigbeeGrey_GetNormal(unsigned short *result)
{
    if (result == NULL)
    {
        return 0;
    }

    memcpy(result, ZigbeeGrey_Normal, sizeof(ZigbeeGrey_Normal));

    return 1;
}

// void ZigbeeGrey_Task_WithoutTick(void)
// {
//     No_Mcu_Ganv_Sensor_Task_Without_tick(&ZigbeeGrey_Sensor);
// };
