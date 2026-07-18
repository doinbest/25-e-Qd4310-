#include "bsp_debug.h"
#include "usart.h"

//加入以下代码,支持printf函数,而不需要选择use MicroLIB
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

/*  要用CubeMX开启USART1中断，并在main中调用 Uart1_StartReceiveIT();  */

uint8_t UART1_RxBuffer[UART_RX_BUFFER_SIZE];
volatile uint8_t UART1_RxPtr = 0;
uint8_t UART1_RxByte = 0;
volatile uint8_t receive_cmd = 0;

/*****************  清空接收缓冲区 **********************/
void uart1_FlushRxBuffer(void)
{
    UART1_RxPtr = 0;
    memset(UART1_RxBuffer, 0, UART_RX_BUFFER_SIZE);
}


/*****************  开启串口1接收中断 **********************/
void Uart1_StartReceiveIT(void)
{
    HAL_UART_Receive_IT(&huart1, &UART1_RxByte, 1);
}


/*****************  获取接收完成标志 **********************/
uint8_t Uart1_GetCmdFlag(void)
{
    return receive_cmd;
}


/*****************  清除接收完成标志 **********************/
void Uart1_ClearCmdFlag(void)
{
    receive_cmd = 0;
}


/*****************  发送单个字节 **********************/
void Usart1_SendByte(uint8_t ch)
{
    HAL_UART_Transmit(&huart1, &ch, 1, 1000);
}


/*****************  发送字符串 **********************/
void Usart1_SendString(uint8_t *str)
{
    while(*str != '\0')
    {
        HAL_UART_Transmit(&huart1, str, 1, 1000);
        str++;
    }
}


/*****************  发送指定长度数组 **********************/
void Usart1_SendArray(uint8_t *buf, uint16_t len)
{
    HAL_UART_Transmit(&huart1, buf, len, 1000);
}

void Protocol_Datas_Proc(void)
{
    int16_t send_data;
    float pid_temp[3] = {0};
    
    if(Car_Mode == Run_Mode)
    {
        send_data = 0;
    }
    else if(Car_Mode == Speed_Mode)
    {
        pid_temp[0] = pid_speed.Kp;
        pid_temp[1] = pid_speed.Ki;
        pid_temp[2] = pid_speed.Kd;

        // set_computer_value(SEND_P_I_D_CMD, CURVES_CH1, pid_temp, 3); // 给通道 1 发送 P I D 值

        send_data = Motor1_Speed;
    }
    else if (Car_Mode == Speed2_Mode)
    {
        pid_temp[0] = pid_speed2.Kp;
        pid_temp[1] = pid_speed2.Ki;
        pid_temp[2] = pid_speed2.Kd;
        set_computer_value(SEND_P_I_D_CMD, CURVES_CH2, pid_temp, 3);    
        send_data = Motor2_Speed;
    }
    else if(Car_Mode == Location_Mode)
    {
        pid_temp[0] = pid_location.Kp;
        pid_temp[1] = pid_location.Ki;
        pid_temp[2] = pid_location.Kd;
        set_computer_value(SEND_P_I_D_CMD, CURVES_CH3, pid_temp, 3);

        send_data = Distance;
    }

    else if(Car_Mode == Angle_Mode)
    {
        pid_temp[0] = pid_location.Kp;
        pid_temp[1] = pid_location.Ki;
        pid_temp[2] = pid_location.Kd;
        set_computer_value(SEND_P_I_D_CMD, CURVES_CH4, pid_temp, 3);

        send_data = Yaw; // 角度环的输出是转向修正量，直接发给上位机观察就好
    }

    set_computer_value(SEND_FACT_CMD, CURVES_CH1, &send_data, 1);
    
    receiving_process();
}