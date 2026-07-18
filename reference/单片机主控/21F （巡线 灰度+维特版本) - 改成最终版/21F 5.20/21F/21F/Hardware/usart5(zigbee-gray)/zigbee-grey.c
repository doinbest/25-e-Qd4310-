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

static unsigned char ZigbeeGrey_TxBuf[ZIGBEE_GREY_TX_BUF_SIZE] = {0};

/*
 * 黑白校准值
 * 这里先沿用例程里的默认值
 * 后期你可以根据实际传感器数据修改
 */
unsigned short ZigbeeGrey_White[ZIGBEE_GREY_CHANNEL_NUM] =
    // {1600};
    {556,459,365,544,611,177,248,235};

    // Anolog 556-459-365-544-611-177-248-235
    // {2407,2719,2114,3315,2436,2016,1979,1956};
    // {2005,1903,1590,2497,2170,907,1236,1269};
// Anolog 2005-1903-1590-2497-2170-907-1236-1269
unsigned  short ZigbeeGrey_Black[ZIGBEE_GREY_CHANNEL_NUM] =
    // {500};
    {
        2189,2115,1510,2500,2142,929,974,1294};

    // Anolog 2189-2115-1510-2500-2142-929-974-1294
    // {354,259,354,857,598,172,206,213};
//
// Anolog 354 - 259 - 354 - 857 - 598 - 172 - 206 - 213
    // {787,768,615,2399,583,598,504,728};
    // 白 Anolog 2407 - 2719 - 2114 - 3315 - 2436 - 2016 - 1979 - 1956

    // 红 Anolog 787 - 768 - 615 - 2399 - 583 - 598 - 504 - 728

    // unsigned short ZigbeeGrey_White[ZIGBEE_GREY_CHANNEL_NUM] =
    //     {2780, 2707, 2713, 2754, 2793, 2659, 2710, 2418};

    // unsigned short ZigbeeGrey_Black[ZIGBEE_GREY_CHANNEL_NUM] =
    //     {
    //         706, 775, 746, 797, 747, 767, 749, 637};

    // 3296-3220-3271-3217-3274-3235-3254-3253
    //  396-359-426-373-393-353-401-363
    /******************** UART5 发送函数 ********************/

    void uart5_send_char(char ch)
{
    HAL_UART_Transmit(&huart5, (uint8_t *)&ch, 1, 0xffff);
}


void  uart5_send_string(char *str)
{
    if(str == NULL)
    {
        return;
    }

    while(*str != '\0')
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
    if(buf == NULL || len == 0)
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
    while((UART5->SR & USART_SR_TXE) == 0);

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
  * @note   计算步骤：
  *         1. 对每个数据乘以其对应的权值(-7,-5,-3,-1,1,3,5,7)
  *         2. 求加权和
  *         3. 求原始数据和
  *         4. 用加权和除以原始数据和得到返回值
  */
int32_t CalculateNormalizedValue(unsigned short Normal[8],uint8_t field)
{
    // 定义权值数组，从左到右对应-7,-5,-3,-1,1,3,5,7
    const short weights[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
    
    int32_t weighted_sum = 0;          // 加权和
    int32_t original_sum = 0; // 原始数据和
    static int last_value = 0;   // 上一次的值
    
    // 计算加权和和原始数据和
    for (int i = 0; i < 8; i++) {
				weighted_sum += 1024*(field?(4096-Normal[i]) * weights[i]:Normal[i] * weights[i]);  // 每个数据乘以对应权值
				original_sum += field?(4096-Normal[i]):Normal[i];               // 累加原始数据
    }
    
    // 计算并返回归一化值
    if (original_sum != 0) { // 避免除以0的情况
        last_value = weighted_sum / original_sum;
        return last_value;
    } else {
        return last_value; // 如果原始数据和为0，返回上一次的值
    }
}


void ZigbeeGrey_Tick(void)
{
    Task_tick(&ZigbeeGrey_Sensor);
}

void ZigbeeGrey_Task_WithTick(void)
{
    /*
     * 1. 有时基传感器常规任务
     * 内部包含：模拟量采集、数字量转换、归一化处理
     * 注意这里调用的是 With_tick 版本的底层函数
     */
    No_Mcu_Ganv_Sensor_Task_With_tick(&ZigbeeGrey_Sensor);
    /*

     * 2. 获取并打印数字量 (8位0/1状态)
     */
    Digtal = Get_Digtal_For_User(&ZigbeeGrey_Sensor);
    // sprintf((char *)ZigbeeGrey_TxBuf,
    //         "Digtal %d-%d-%d-%d-%d-%d-%d-%d\r\n",
    //         (Digtal >> 0) & 0x01,
    //         (Digtal >> 1) & 0x01,
    //         (Digtal >> 2) & 0x01,
    //         (Digtal >> 3) & 0x01,
    //         (Digtal >> 4) & 0x01,
    //         (Digtal >> 5) & 0x01,
    //         (Digtal >> 6) & 0x01,
    //         (Digtal >> 7) & 0x01);
    // uart5_send_string((char *)ZigbeeGrey_TxBuf);
    // memset(ZigbeeGrey_TxBuf, 0, ZIGBEE_GREY_TX_BUF_SIZE);

    /*
     * 3. 获取并打印模拟量 (原始 ADC 数据)
     */
    if (Get_Anolog_Value(&ZigbeeGrey_Sensor, ZigbeeGrey_Analog))
    {
        sprintf((char *)ZigbeeGrey_TxBuf,
                "Anolog %d-%d-%d-%d-%d-%d-%d-%d\r\n",
                ZigbeeGrey_Analog[0], ZigbeeGrey_Analog[1],
                ZigbeeGrey_Analog[2], ZigbeeGrey_Analog[3],
                ZigbeeGrey_Analog[4], ZigbeeGrey_Analog[5],
                ZigbeeGrey_Analog[6], ZigbeeGrey_Analog[7]);
        // uart5_send_string((char *)ZigbeeGrey_TxBuf);
        memset(ZigbeeGrey_TxBuf, 0, ZIGBEE_GREY_TX_BUF_SIZE);
    }

    /*
     * 4. 获取并打印归一化结果 (经过黑白校准后的数据 0~4096)
     */
    if (Get_Normalize_For_User(&ZigbeeGrey_Sensor, ZigbeeGrey_Normal))
    {
        // sprintf((char *)ZigbeeGrey_TxBuf,
        //         "Normalize %d-%d-%d-%d-%d-%d-%d-%d\r\n",
        //         ZigbeeGrey_Normal[0], ZigbeeGrey_Normal[1],
        //         ZigbeeGrey_Normal[2], ZigbeeGrey_Normal[3],
        //         ZigbeeGrey_Normal[4], ZigbeeGrey_Normal[5],
        //         ZigbeeGrey_Normal[6], ZigbeeGrey_Normal[7]);
        // // uart5_send_string((char *)ZigbeeGrey_TxBuf);
        // memset(ZigbeeGrey_TxBuf, 0, ZIGBEE_GREY_TX_BUF_SIZE);
    }
    result_angle = CalculateNormalizedValue(ZigbeeGrey_Normal, 1);
    sprintf((char *)ZigbeeGrey_TxBuf,
            "Angle %d\r\n",
            result_angle);
    // uart5_send_string((char *)ZigbeeGrey_TxBuf);
    memset(ZigbeeGrey_TxBuf, 0, ZIGBEE_GREY_TX_BUF_SIZE);
    sprintf((char *)ZigbeeGrey_TxBuf,
            "index %f\r\n",
            result_angle / 1024.0);
    // uart5_send_string((char *)ZigbeeGrey_TxBuf);
    memset(ZigbeeGrey_TxBuf, 0, ZIGBEE_GREY_TX_BUF_SIZE);

    // sprintf((char *)ZigbeeGrey_TxBuf,
    //         "trace_value %d-%d-%d-%d-%d-%d-%d-%d\r\n",
    //         trace_value[0], trace_value[1],
    //         trace_value[2], trace_value[3],
    //         trace_value[4], trace_value[5],
    //         trace_value[6], trace_value[7]);
    // uart5_send_string((char *)ZigbeeGrey_TxBuf);
    // memset(ZigbeeGrey_TxBuf, 0, ZIGBEE_GREY_TX_BUF_SIZE);
}

unsigned char ZigbeeGrey_GetDigital(void)
{
    return Digtal;
}


unsigned char ZigbeeGrey_GetAnalog(unsigned short *result)
{
    if(result == NULL)
    {
        return 0;
    }

    memcpy(result, ZigbeeGrey_Analog, sizeof(ZigbeeGrey_Analog));

    return 1;
}


unsigned char ZigbeeGrey_GetNormal(unsigned short *result)
{
    if(result == NULL)
    {
        return 0;
    }

    memcpy(result, ZigbeeGrey_Normal, sizeof(ZigbeeGrey_Normal));

    return 1;
}




//void ZigbeeGrey_Task_WithoutTick(void)
//{
//    /*
//     * 无时基传感器常规任务
//     * 内部包含：
//     * 1. 模拟量采集
//     * 2. 数字量转换
//     * 3. 归一化处理
//     */
//    No_Mcu_Ganv_Sensor_Task_Without_tick(&ZigbeeGrey_Sensor);

//    /*
//     * 获取数字量
//     */
//    ZigbeeGrey_Digital = Get_Digtal_For_User(&ZigbeeGrey_Sensor);

//    sprintf((char *)ZigbeeGrey_TxBuf,
//            "Digtal %d-%d-%d-%d-%d-%d-%d-%d\r\n",
//            (ZigbeeGrey_Digital >> 0) & 0x01,
//            (ZigbeeGrey_Digital >> 1) & 0x01,
//            (ZigbeeGrey_Digital >> 2) & 0x01,
//            (ZigbeeGrey_Digital >> 3) & 0x01,
//            (ZigbeeGrey_Digital >> 4) & 0x01,
//            (ZigbeeGrey_Digital >> 5) & 0x01,
//            (ZigbeeGrey_Digital >> 6) & 0x01,
//            (ZigbeeGrey_Digital >> 7) & 0x01);

//    uart5_send_string((char *)ZigbeeGrey_TxBuf);
//    memset(ZigbeeGrey_TxBuf, 0, ZIGBEE_GREY_TX_BUF_SIZE);

//    /*
//     * 获取模拟量
//     */
//    if(Get_Anolog_Value(&ZigbeeGrey_Sensor, ZigbeeGrey_Analog))
//    {
//        sprintf((char *)ZigbeeGrey_TxBuf,
//                "Anolog %d-%d-%d-%d-%d-%d-%d-%d\r\n",
//                ZigbeeGrey_Analog[0],
//                ZigbeeGrey_Analog[1],
//                ZigbeeGrey_Analog[2],
//                ZigbeeGrey_Analog[3],
//                ZigbeeGrey_Analog[4],
//                ZigbeeGrey_Analog[5],
//                ZigbeeGrey_Analog[6],
//                ZigbeeGrey_Analog[7]);

//        uart5_send_string((char *)ZigbeeGrey_TxBuf);
//        memset(ZigbeeGrey_TxBuf, 0, ZIGBEE_GREY_TX_BUF_SIZE);
//    }

//    /*
//     * 获取归一化结果
//     */
//    if(Get_Normalize_For_User(&ZigbeeGrey_Sensor, ZigbeeGrey_Normal))
//    {
//        sprintf((char *)ZigbeeGrey_TxBuf,
//                "Normalize %d-%d-%d-%d-%d-%d-%d-%d\r\n",
//                ZigbeeGrey_Normal[0],
//                ZigbeeGrey_Normal[1],
//                ZigbeeGrey_Normal[2],
//                ZigbeeGrey_Normal[3],
//                ZigbeeGrey_Normal[4],
//                ZigbeeGrey_Normal[5],
//                ZigbeeGrey_Normal[6],
//                ZigbeeGrey_Normal[7]);

//        uart5_send_string((char *)ZigbeeGrey_TxBuf);
//        memset(ZigbeeGrey_TxBuf, 0, ZIGBEE_GREY_TX_BUF_SIZE);
//    }

//    /*
//     * 原例程是 HAL_Delay(1)
//     * 但是连续打印 3 行数据，1ms 太快，串口很可能刷屏严重
//     * 如果你只是调试，建议改成 50 或 100
//     */
//    HAL_Delay(1);
//};
