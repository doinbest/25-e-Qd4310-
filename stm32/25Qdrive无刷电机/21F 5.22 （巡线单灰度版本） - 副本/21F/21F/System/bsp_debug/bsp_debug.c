#include "bsp_debug.h"
#include "usart.h"
#include "maixcam.h"

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
uint32_t PID_Debug_RxCount = 0;
uint32_t PID_Debug_CmdCount = 0;
uint8_t PID_Debug_LastCmd = CMD_NONE;

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

static uint8_t PID_Debug_Channel(void)
{
    switch (Car_Mode)
    {
    case Speed_Mode:
        return CURVES_CH1;
    case Speed2_Mode:
        return CURVES_CH2;
    case Location_Mode:
        return CURVES_CH3;
    case Angle_Mode:
        return CURVES_CH4;
    case Gray_Mode:
    case X_Mode:
    case Y_Mode:
        return CURVES_CH1;
    default:
        return CURVES_CH1;
    }
}

static uint8_t PID_Debug_IsVisionMode(void)
{
    return ((Car_Mode == X_Mode) || (Car_Mode == Y_Mode)) ? 1U : 0U;
}

static _pid *PID_Debug_CurrentPid(void)
{
    switch (Car_Mode)
    {
    case Speed_Mode:
        return &pid_speed;
    case Speed2_Mode:
        return &pid_speed2;
    case Location_Mode:
        return &pid_location;
    case Angle_Mode:
        return &pid_angle;
    case Gray_Mode:
        return &pid_graysensor;
    case X_Mode:
        return &pid_dr4310X;
    case Y_Mode:
        return &pid_dr4310Y;
    case X_Rate_Mode:
        return &pid_dr4310X_imu_rate;
    case Y_Rate_Mode:
        return &pid_dr4310Y_imu_rate;
    default:
        return &pid_speed;
    }
}

static uint8_t PID_Debug_IsImuRateMode(void)
{
    return ((Car_Mode == X_Rate_Mode) || (Car_Mode == Y_Rate_Mode)) ? 1U : 0U;
}

void Protocol_Datas_Proc(void)
{
    static uint32_t last_fact_tick = 0;
    static uint8_t last_mode = 0xFF;
    uint32_t now;
    uint8_t mode_changed;
    uint8_t channel;
    int32_t send_data = 0;
    int32_t pwm_data = 0;
    int32_t target_data = 0;
    float pid_temp[3];
    _pid *pid;

    receiving_process();

    now = HAL_GetTick();
    mode_changed = (last_mode != Car_Mode);
    if (mode_changed)
    {
        last_mode = Car_Mode;
        OLED_Clear();
    }

    if (Car_Mode == Run_Mode)
    {
        if (now - last_fact_tick >= 50U)
        {
            send_data = 0;
            last_fact_tick = now;
            set_computer_value(SEND_FACT_CMD, CURVES_CH1, &send_data, 1);
        }
        return;
    }

    if (PID_Debug_IsVisionMode() != 0U)
    {
        pid = PID_Debug_CurrentPid();
        if (mode_changed)
        {
            pid_temp[0] = pid->Kp;
            pid_temp[1] = pid->Ki;
            pid_temp[2] = pid->Kd;
            set_computer_value(SEND_P_I_D_CMD, CURVES_CH1, pid_temp, 3);
        }

        if (now - last_fact_tick >= 50U)
        {
            /* The PC owns the target value; only actual pixel feedback is streamed. */
            send_data = (Car_Mode == X_Mode) ?
                        (int32_t)gimbal_get_x_pixel_px() :
                        (int32_t)gimbal_get_y_pixel_px();
            last_fact_tick = now;
            set_computer_value(SEND_FACT_CMD, CURVES_CH1, &send_data, 1);
        }
        return;
    }

    if (PID_Debug_IsImuRateMode() != 0U)
    {
        pid = PID_Debug_CurrentPid();
        if (mode_changed)
        {
            pid_temp[0] = pid->Kp;
            pid_temp[1] = pid->Ki;
            pid_temp[2] = pid->Kd;
            set_computer_value(SEND_P_I_D_CMD, CURVES_CH1, pid_temp, 3);

            /* Initialize the PC target once.  It owns all later target changes. */
            target_data = 0;
            set_computer_value(SEND_TARGET_CMD, CURVES_CH1, &target_data, 1);
        }
        if (now - last_fact_tick >= 50U)
        {
            if (Car_Mode == X_Rate_Mode)
            {
                send_data = (int32_t)gimbal_get_x_rate_rpm();
            }
            else
            {
                send_data = (int32_t)gimbal_get_y_rate_rpm();
            }
            last_fact_tick = now;
            /* CH1 only: actual Euler-difference gimbal speed (rpm).  Command is on OLED. */
            set_computer_value(SEND_FACT_CMD, CURVES_CH1, &send_data, 1);
        }
        return;
    }

    pid = PID_Debug_CurrentPid();
    channel = PID_Debug_Channel();
    if (mode_changed)
    {
        pid_temp[0] = pid->Kp;
        pid_temp[1] = pid->Ki;
        pid_temp[2] = pid->Kd;
        set_computer_value(SEND_P_I_D_CMD, channel, pid_temp, 3);
    }

    if (now - last_fact_tick < 50U)
    {
        return;
    }
    last_fact_tick = now;

    switch (Car_Mode)
    {
    case Speed_Mode:
        send_data = (int32_t)encoder_left.speed_mm_s;
        pwm_data = left_pwm;
        break;
    case Speed2_Mode:
        send_data = (int32_t)encoder_right.speed_mm_s;
        pwm_data = right_pwm;
        break;
    case Location_Mode:
        send_data = Distance;
        break;
    case Angle_Mode:
        send_data = (int32_t)Yaw;
        break;
    case Gray_Mode:
        send_data = result_angle;
        break;
    default:
        send_data = 0;
        break;
    }

    set_computer_value(SEND_FACT_CMD, CURVES_CH1, &send_data, 1);

    /* CH2 displays the applied PWM for the motor under speed-loop tuning. */
    if ((Car_Mode == Speed_Mode) || (Car_Mode == Speed2_Mode))
    {
        set_computer_value(SEND_FACT_CMD, CURVES_CH2, &pwm_data, 1);
    }
}

static void Format_Float2(char *buf, float val)
{
    int32_t scaled;
    int32_t whole;
    int32_t frac;

    scaled = (int32_t)(val * 100.0f + ((val >= 0.0f) ? 0.5f : -0.5f));
    if (scaled < 0)
    {
        scaled = -scaled;
        whole = scaled / 100;
        frac = scaled % 100;
        sprintf(buf, "-%ld.%02ld", (long)whole, (long)frac);
    }
    else
    {
        whole = scaled / 100;
        frac = scaled % 100;
        sprintf(buf, "%ld.%02ld", (long)whole, (long)frac);
    }
}

static void Format_Float1(char *buf, float val)
{
    int32_t scaled;
    int32_t whole;
    int32_t frac;

    scaled = (int32_t)(val * 10.0f + ((val >= 0.0f) ? 0.5f : -0.5f));
    if (scaled < 0)
    {
        scaled = -scaled;
        whole = scaled / 10;
        frac = scaled % 10;
        sprintf(buf, "-%ld.%01ld", (long)whole, (long)frac);
    }
    else
    {
        whole = scaled / 10;
        frac = scaled % 10;
        sprintf(buf, "%ld.%01ld", (long)whole, (long)frac);
    }
}

static void OLED_ShowPidValue(uint8_t row, const char *name, float val)
{
    char line[24];
    char value[14];

    Format_Float2(value, val);
    sprintf(line, "%s:%-14s", name, value);
    line[16] = '\0';
    OLED_ShowString(0, row, (uint8_t *)line, 12);
}

/**
 * @brief 在 OLED 第一行居中显示模式标题。
 * @param title 标题文字，最长显示 16 个字符。
 */
static void OLED_ShowModeTitle(const char *title)
{
    char line[17];
    uint8_t len = (uint8_t)strlen(title);
    uint8_t start;

    if (len > 16U) len = 16U;
    start = (uint8_t)((16U - len) / 2U);
    memset(line, ' ', 16U);
    memcpy(&line[start], title, len);
    line[16] = '\0';
    OLED_ShowString(0, 0, (uint8_t *)line, 12);
}

void PID_Debug_OLED_Show(void)
{
    static uint32_t last_oled_tick = 0;
    static uint8_t last_mode = 0xFF;
    uint32_t now;
    char line[24];
    char value[14];
    _pid *pid;

    now = HAL_GetTick();
    if ((now - last_oled_tick < 100U) && (last_mode == Car_Mode))
    {
        return;
    }
    last_oled_tick = now;

    if (last_mode != Car_Mode)
    {
        last_mode = Car_Mode;
        OLED_Clear();
    }

    pid = PID_Debug_CurrentPid();

    switch (Car_Mode)
    {
    case Run_Mode:
        /* 25E 比赛首页：只显示底盘运行、灰度标定与航向信息。 */
        OLED_ShowModeTitle("25E RUN");
        /* USART3 receive counters (mod 1000): vision / gyro / CRC errors. */
        sprintf(line, "V:%03lu G:%03lu C:%03lu",
                (unsigned long)(maixcam_frame_count % 1000U),
                (unsigned long)(maixcam_imu_frame_count % 1000U),
                (unsigned long)(maixcam_crc_error_count % 1000U));
        OLED_ShowString(0, 1, (uint8_t *)line, 12);
        sprintf(line, "Lap:%d/%d T:%2d", control_get_lap_done(),
                control_get_lap_target(), control_get_turn_done());
        OLED_ShowString(0, 2, (uint8_t *)line, 12);
        sprintf(line, "RUN:%d Cal:%s", (control_running != 0U),
                Key_GetGrayCalibrationText());
        OLED_ShowString(0, 3, (uint8_t *)line, 12);
        Format_Float1(value, control_get_run_distance_mm());
        sprintf(line, "Enc:%7smm", value);
        OLED_ShowString(0, 4, (uint8_t *)line, 12);
        Format_Float1(value, Yaw);
        sprintf(line, "Yaw:%7sdeg", value);
        OLED_ShowString(0, 5, (uint8_t *)line, 12);
        OLED_ShowString(0, 6, (uint8_t *)"K1:LAP K2:RUN   ", 12);
        OLED_ShowString(0, 7, (uint8_t *)"K3:GMBL K4:CAL  ", 12);
        break;

    case Speed_Mode:
        OLED_ShowModeTitle("25E Speed_Mode");
        sprintf(line, "run:%d tar:%4ld  ", control_running, (long)pid_speed.target_val);
        OLED_ShowString(0, 1, (uint8_t *)line, 12);
        sprintf(line, "act:%4d pwm:%3d", Motor1_Speed, left_pwm);
        OLED_ShowString(0, 2, (uint8_t *)line, 12);
        OLED_ShowPidValue(3, "P", pid->Kp);
        OLED_ShowPidValue(4, "I", pid->Ki);
        OLED_ShowPidValue(5, "D", pid->Kd);
        sprintf(line, "m1:%4d m2:%4d ", Motor1_Speed, Motor2_Speed);
        OLED_ShowString(0, 6, (uint8_t *)line, 12);
        Format_Float1(value, Yaw);
        // sprintf(line, "yaw:%7s     ", value);
        // OLED_ShowString(0, 7, (uint8_t *)line, 12);
        sprintf(line, "R:%lu C:%lu %02X",
                PID_Debug_RxCount,
                PID_Debug_CmdCount,
                PID_Debug_LastCmd);

        OLED_ShowString(0, 7, (uint8_t *)line, 12);
        break;

    case Speed2_Mode:
        OLED_ShowModeTitle("25E Speed2_Mode");
        sprintf(line, "run:%d tar:%4ld  ", control_running, (long)pid_speed2.target_val);
        OLED_ShowString(0, 1, (uint8_t *)line, 12);
        sprintf(line, "act:%4d pwm:%3d", Motor2_Speed, right_pwm);
        OLED_ShowString(0, 2, (uint8_t *)line, 12);
        OLED_ShowPidValue(3, "P", pid->Kp);
        OLED_ShowPidValue(4, "I", pid->Ki);
        OLED_ShowPidValue(5, "D", pid->Kd);
        sprintf(line, "m1:%4d m2:%4d ", Motor1_Speed, Motor2_Speed);
        OLED_ShowString(0, 6, (uint8_t *)line, 12);
        Format_Float1(value, Yaw);
        sprintf(line, "yaw:%7s     ", value);
        OLED_ShowString(0, 7, (uint8_t *)line, 12);
        break;

    case Location_Mode:
        OLED_ShowModeTitle("25E Loc_Mode");
        sprintf(line, "run:%d tar:%6ld", control_running, (long)pid_location.target_val);
        OLED_ShowString(0, 1, (uint8_t *)line, 12);
        sprintf(line, "d:%7ld v:%3d ", (long)Distance, speed_target_mm_s);
        OLED_ShowString(0, 2, (uint8_t *)line, 12);
        sprintf(line, "p1:%4d p2:%4d ", left_pwm, right_pwm);
        OLED_ShowString(0, 3, (uint8_t *)line, 12);
        OLED_ShowPidValue(4, "P", pid->Kp);
        OLED_ShowPidValue(5, "I", pid->Ki);
        OLED_ShowPidValue(6, "D", pid->Kd);
        Format_Float1(value, Yaw);
        sprintf(line, "yaw:%7s     ", value);
        OLED_ShowString(0, 7, (uint8_t *)line, 12);
        break;

    case Angle_Mode:
        OLED_ShowModeTitle("25E Angle_Mode");
        sprintf(line, "run:%d tar:%6ld", control_running, (long)pid_angle.target_val);
        OLED_ShowString(0, 1, (uint8_t *)line, 12);
        sprintf(line, "y:%6ld w:%3d  ", (long)Yaw, speed_target_mm_s);
        OLED_ShowString(0, 2, (uint8_t *)line, 12);
        sprintf(line, "p1:%4d p2:%4d ", left_pwm, right_pwm);
        OLED_ShowString(0, 3, (uint8_t *)line, 12);
        OLED_ShowPidValue(4, "P", pid->Kp);
        OLED_ShowPidValue(5, "I", pid->Ki);
        OLED_ShowPidValue(6, "D", pid->Kd);
        sprintf(line, "m1:%4d m2:%4d ", Motor1_Speed, Motor2_Speed);
        OLED_ShowString(0, 7, (uint8_t *)line, 12);
        break;

    case Gray_Mode:
        OLED_ShowModeTitle("25E Gray_Mode");
        sprintf(line, "run:%d spd:%3dcm", control_running,
                (int)(control_get_line_speed_mm_s() / 10.0f));
        OLED_ShowString(0, 1, (uint8_t *)line, 12);
        sprintf(line, "err:%6d out:%3d", result_angle,
                (int)pid_graysensor.output);
        OLED_ShowString(0, 2, (uint8_t *)line, 12);
        OLED_ShowPidValue(3, "P", pid->Kp);
        OLED_ShowPidValue(4, "I", pid->Ki);
        OLED_ShowPidValue(5, "D", pid->Kd);
        sprintf(line, "pwm:%3d,%3d", left_pwm, right_pwm);
        OLED_ShowString(0, 6, (uint8_t *)line, 12);
        break;

    case X_Mode:
        OLED_ShowModeTitle("X PIXEL PID");
        sprintf(line, "aim:%-5s v:%d", gimbal_get_state_text(),
                gimbal_target_valid());
        OLED_ShowString(0, 1, (uint8_t *)line, 12);
        sprintf(line, "tar:%3d act:%3d", (int)gimbal_get_x_pixel_target_px(),
                (int)gimbal_get_x_pixel_px());
        OLED_ShowString(0, 2, (uint8_t *)line, 12);
        sprintf(line, "rpm:%3d/%3d", (int)gimbal_get_x_rate_target_rpm(),
                (int)gimbal_get_x_rate_rpm());
        OLED_ShowString(0, 3, (uint8_t *)line, 12);
        sprintf(line, "spd:%4drpm", (int)gimbal_get_x_cmd_rpm());
        OLED_ShowString(0, 4, (uint8_t *)line, 12);
        OLED_ShowPidValue(5, "P", pid->Kp);
        OLED_ShowPidValue(6, "I", pid->Ki);
        OLED_ShowString(0, 7, (uint8_t *)"B2 X ON/OFF", 12);
        break;

    case Y_Mode:
        OLED_ShowModeTitle("Y PIXEL PID");
        sprintf(line, "aim:%-5s v:%d", gimbal_get_state_text(),
                gimbal_target_valid());
        OLED_ShowString(0, 1, (uint8_t *)line, 12);
        sprintf(line, "tar:%3d act:%3d", (int)gimbal_get_y_pixel_target_px(),
                (int)gimbal_get_y_pixel_px());
        OLED_ShowString(0, 2, (uint8_t *)line, 12);
        sprintf(line, "rpm:%3d/%3d", (int)gimbal_get_y_rate_target_rpm(),
                (int)gimbal_get_y_rate_rpm());
        OLED_ShowString(0, 3, (uint8_t *)line, 12);
        sprintf(line, "spd:%4drpm", (int)gimbal_get_y_cmd_rpm());
        OLED_ShowString(0, 4, (uint8_t *)line, 12);
        OLED_ShowPidValue(5, "P", pid->Kp);
        OLED_ShowPidValue(6, "I", pid->Ki);
        OLED_ShowString(0, 7, (uint8_t *)"B3 Y ON/OFF", 12);
        break;

    case X_Rate_Mode:
        OLED_ShowModeTitle("X IMU RATE RPM PID");
        sprintf(line, "rpm:%4d/%4d", (int)pid_dr4310X_imu_rate.target_val,
                (int)gimbal_get_x_rate_rpm());
        OLED_ShowString(0, 1, (uint8_t *)line, 12);
        sprintf(line, "spd:%4drpm", (int)gimbal_get_x_cmd_rpm());
        OLED_ShowString(0, 2, (uint8_t *)line, 12);
        sprintf(line, "state:%-4s", gimbal_get_state_text());
        OLED_ShowString(0, 3, (uint8_t *)line, 12);
        OLED_ShowPidValue(4, "P", pid->Kp);
        OLED_ShowPidValue(5, "I", pid->Ki);
        OLED_ShowPidValue(6, "D", pid->Kd);
        sprintf(line, "Gx:%4d Gy:%4d", (int)maixcam_gyro_x_dps,
                (int)maixcam_gyro_y_dps);
        OLED_ShowString(0, 7, (uint8_t *)line, 12);
        break;

    case Y_Rate_Mode:
        OLED_ShowModeTitle("Y IMU RATE RPM PID");
        sprintf(line, "rpm:%4d/%4d", (int)pid_dr4310Y_imu_rate.target_val,
                (int)gimbal_get_y_rate_rpm());
        OLED_ShowString(0, 1, (uint8_t *)line, 12);
        sprintf(line, "spd:%4drpm", (int)gimbal_get_y_cmd_rpm());
        OLED_ShowString(0, 2, (uint8_t *)line, 12);
        sprintf(line, "state:%-4s", gimbal_get_state_text());
        OLED_ShowString(0, 3, (uint8_t *)line, 12);
        OLED_ShowPidValue(4, "P", pid->Kp);
        OLED_ShowPidValue(5, "I", pid->Ki);
        OLED_ShowPidValue(6, "D", pid->Kd);
        sprintf(line, "Gy:%4d Gz:%4d", (int)maixcam_gyro_y_dps,
                (int)maixcam_gyro_z_dps);
        OLED_ShowString(0, 7, (uint8_t *)line, 12);
        break;

    default:
        OLED_ShowString(0, 0, (uint8_t *)"MODE:UNKNOWN    ", 12);
        break;
    }
}
