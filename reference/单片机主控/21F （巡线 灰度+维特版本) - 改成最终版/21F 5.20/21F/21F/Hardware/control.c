#include "control.h"

#define VMAX 80 // 最大速度
#define VMIN 20 // 最小速度

#define SPEED_TARGET_MAX 80.0f
#define TURN_TARGET_MAX 50.0f
#define PWM_OUTPUT_MAX 100.0f

// #define VMAX 25
// #define VMIN 12
// int K_V = 2;
// 全局变量声明
int SpeedL;         // 左轮速度
int SpeedR;         // 右轮速度
int Speed_Set = 0;  // 基准速度
float Tarangle = 0; // 目标角度

int pwmL, pwmR;

// 偏航角PID参数
// int Kp_Yaw = 5;  // 比例系数
// int Ki_Yaw = 0;  // 积分系数
// int Kd_Yaw = 15; // 微分系数
int Kp_Yaw = 10;  // 比例系数
int Ki_Yaw = 0;  // 积分系数
int Kd_Yaw = 35; // 微分系数

int Yaw_Now_Correct;  // 当前偏航角校正值
int Integral_Yaw = 0; // 积分项累加器
int Err_Yaw = 0;      // 当前误差
int LastErr_Yaw = 0;  // 上一次误差
int Output_Yaw = 0;   // PID输出
int result_angle = 0; // 角度结果
int max = 50;         // 输出限幅最大值

#define INTEGRAL_MAX 5  // 积分上限
#define INTEGRAL_MIN -5 // 积分下限

// 标志变量 1表示开启，0表示关闭，分别有 距离环，角度环的标志位
uint8_t car_go = 0;
uint8_t Location_flag = 0;
uint8_t Angle_flag = 0;
uint8_t Gray_flag = 0; // 0表示关闭，1表示打开

uint8_t task_state = 0;
int8_t Run_Dir = 1; // 1: 前进，-1: 后退
uint8_t number_flag = 0;
uint8_t special_flag = 0;

float wit_output = 0; // wit转向修正量

static uint8_t far_first_side = 0;

/**
 * @brief  灰度偏航角PID控制
 * @param  Target: 目标角度
 */
void Pid_Yaw(float Target)
{
    Yaw_Now_Correct = result_angle;
    Err_Yaw = Target - Yaw_Now_Correct;

    // 积分项处理
    Integral_Yaw += Err_Yaw;
    if (Integral_Yaw > INTEGRAL_MAX)
        Integral_Yaw = INTEGRAL_MAX;
    if (Integral_Yaw < INTEGRAL_MIN)
        Integral_Yaw = INTEGRAL_MIN;

    // PID计算
    Output_Yaw = (((Kp_Yaw * Err_Yaw) / 1024) +
                  ((Kd_Yaw * (Err_Yaw - LastErr_Yaw)) / 1024) +
                  ((Ki_Yaw * Integral_Yaw) / 1024));

    LastErr_Yaw = Err_Yaw;

    // 输出限幅
    if (Output_Yaw > max)
        Output_Yaw = max;
    if (Output_Yaw < -max)
        Output_Yaw = -max;
}

float PID_Maixcam_realize(float Target)
{
    float maixcam_position = 0.0f;
    uint8_t active_count = 0;

    pid_maixcam.target_val = Target;

    const short maixcam_weights[8] = {-7, -5, -3, -1, 1, 3, 5, 7};

    for (int i = 0; i < 8; i++)
    {
        if (trace_value[i])
        {
            maixcam_position += maixcam_weights[i];
            active_count++;
        }
    }

    /*
     * 如果检测到了红线，计算平均位置
     * 这样可以避免多个区域同时识别时，偏差被放大
     */
    if (active_count > 0)
    {
        maixcam_position /= active_count;
    }
    else
    {
        /*
         * 没检测到线时的处理
         * 可以先保持上一次误差，也可以直接停车
         * 这里暂时设为 0
         */
        maixcam_position = 0.0f;
    }

    pid_maixcam.actual_val = maixcam_position;

    pid_maixcam.err = pid_maixcam.target_val - pid_maixcam.actual_val;

    pid_maixcam.integral += pid_maixcam.err;
    pid_maixcam.integral = Limit_Float(pid_maixcam.integral, -10, 10);

    float output = pid_maixcam.Kp * pid_maixcam.err +
                   pid_maixcam.Ki * pid_maixcam.integral +
                   pid_maixcam.Kd * (pid_maixcam.err - pid_maixcam.err_last);

    pid_maixcam.err_last = pid_maixcam.err;

    return output;
}

void Maixcam_Track_Control(void)
{
    float maixcam_output = PID_Maixcam_realize(0.0f);

    maixcam_output = Limit_Float(maixcam_output, -30, 30);

    pwmL = pwmR = Run_Dir * 30;

    SpeedL = (int)(pwmL - maixcam_output);
    SpeedR = (int)(pwmR + maixcam_output);

    Motor_SetSpeed(MOTOR_LEFT, SpeedL);
    Motor_SetSpeed(MOTOR_RIGHT, SpeedR);
}

void Angle_Hold_Reset_To_Zero(void)
{
    JY61p_HardwareZeroYaw();

    pid_angle.target_val = 0.0f;
    pid_angle.err = 0.0f;
    pid_angle.err_last = 0.0f;
    pid_angle.integral = 0.0f;
    pid_angle.err_prev = 0.0f;
    pid_angle.output = 0.0f;
}

static void Cascade_Speed_Output(float left_target_speed, float right_target_speed)
{
    float left_pwm;
    float right_pwm;

    left_target_speed = Limit_Float(left_target_speed, -SPEED_TARGET_MAX, SPEED_TARGET_MAX);
    right_target_speed = Limit_Float(right_target_speed, -SPEED_TARGET_MAX, SPEED_TARGET_MAX);

    set_pid_target(&pid_speed, left_target_speed);
    set_pid_target(&pid_speed2, right_target_speed);

    /*
     * 这里用 10ms 内编码器增量 diff_cnt 作为速度环实际值
     */
    left_pwm = speed_pid_realize(&pid_speed, (float)encoder_left.diff_cnt);
    right_pwm = speed_pid_realize(&pid_speed2, (float)encoder_right.diff_cnt);

    left_pwm = Limit_Float(left_pwm, -PWM_OUTPUT_MAX, PWM_OUTPUT_MAX);
    right_pwm = Limit_Float(right_pwm, -PWM_OUTPUT_MAX, PWM_OUTPUT_MAX);

    SpeedL = (int)left_pwm;
    SpeedR = (int)right_pwm;

    Motor_SetSpeed(MOTOR_LEFT, SpeedL);
    Motor_SetSpeed(MOTOR_RIGHT, SpeedR);
}

static void Cascade_Stop(void)
{
    PID_Reset(&pid_speed);
    PID_Reset(&pid_speed2);

    Speed_Set = 0;
    SpeedL = 0;
    SpeedR = 0;

    Motor_SetSpeed(MOTOR_LEFT, 0);
    Motor_SetSpeed(MOTOR_RIGHT, 0);
}

/**
 * @brief 电机控制主函数，实现差速控制和PID调节
 */
void Motor_Control(void)
{
    float base_speed;
    float turn_speed = 0.0f;
    float left_target_speed;
    float right_target_speed;

    float gray_turn = 0.0f;
    float gyro_turn = 0.0f;

    /*
     * 1. 位置环：目标距离 - 实际距离 -> 基准目标速度
     * 注意：这里的输出不再直接当 PWM，而是当目标速度
     */
    base_speed = location_pid_realize(&pid_location, pid_location.actual_val);
    base_speed = Limit_Float(base_speed, -SPEED_TARGET_MAX, SPEED_TARGET_MAX);

    Speed_Set = (int)base_speed;

    /*
     * special_flag == 1：
     * 已知 T 口盲走段。
     * 完全忽略灰度，只用陀螺仪保持方向 + 编码器定距离。
     */
    if (Run_Dir == 1 && special_flag == 1)
    {
        gyro_turn = angle_pid_realize(&pid_angle, Yaw);
        gyro_turn = Limit_Float(gyro_turn, -TURN_TARGET_MAX, TURN_TARGET_MAX);

        /*
         * T 口盲走段建议限速，防止冲过转弯中心。
         */
        base_speed = Limit_Float(base_speed, -40.0f, 40.0f);

        left_target_speed = base_speed - gyro_turn;
        right_target_speed = base_speed + gyro_turn;

        Cascade_Speed_Output(left_target_speed, right_target_speed);
        return;
    }

    /*
     * special_flag == 2：
     * 灰度角度环 + 陀螺仪角度环并联作为外环，
     * 两个外环输出相加后，作为左右轮目标速度差，
     * 再串到速度环。
     *
     * 用于远端第一个 T 口左/右转后，沿远端侧边走廊直行这一段。
     */
    if (Run_Dir == 1 && special_flag == 2)
    {
        /*
         * 灰度没有丢线时，使用灰度角度环。
         * 如果灰度全白，说明暂时看不到线，此时灰度环不参与，
         * 只靠陀螺仪保持方向。
         */
        if (Digtal != 0xFF)
        {
            Pid_Yaw(Tarangle);
            gray_turn = (float)Output_Yaw;
        }
        else
        {
            gray_turn = 0.0f;

            result_angle = 0;
            Output_Yaw = 0;
            Err_Yaw = 0;
            LastErr_Yaw = 0;
            Integral_Yaw = 0;
        }

        /*
         * 陀螺仪角度环始终参与，负责抑制车身整体偏航。
         */
        gyro_turn = angle_pid_realize(&pid_angle, Yaw);

        /*
         * 两个外环并联，相加后再统一限幅。
         */
        turn_speed = gray_turn + gyro_turn;
        turn_speed = Limit_Float(turn_speed, -TURN_TARGET_MAX, TURN_TARGET_MAX);

        left_target_speed = base_speed - turn_speed;
        right_target_speed = base_speed + turn_speed;

        Cascade_Speed_Output(left_target_speed, right_target_speed);
        return;
    }

    /*
     * 普通丢线：
     * 非 special 段才执行低速直走。
     */
    if (Gray_flag == 1 && Run_Dir == 1 && Digtal == 0xFF)
    {
        result_angle = 0;
        Output_Yaw = 0;
        Err_Yaw = 0;
        LastErr_Yaw = 0;
        Integral_Yaw = 0;

        left_target_speed = Run_Dir * VMIN;
        right_target_speed = Run_Dir * VMIN;

        Cascade_Speed_Output(left_target_speed, right_target_speed);
        return;
    }

    /*
     * 下面保持你原来的正常控制逻辑
     */
    if (Run_Dir == 1)
    {
        if (Gray_flag == 1)
        {
            if (special_flag == 0)
            {
                Pid_Yaw(Tarangle);
                turn_speed = (float)Output_Yaw;
            }
            else
            {
                turn_speed = angle_pid_realize(&pid_angle, Yaw);
            }
        }
        else
        {
            turn_speed = 0.0f;
        }
    }
    else
    {
        turn_speed = angle_pid_realize(&pid_angle, Yaw);
    }

    turn_speed = Limit_Float(turn_speed, -TURN_TARGET_MAX, TURN_TARGET_MAX);

    left_target_speed = base_speed - turn_speed;
    right_target_speed = base_speed + turn_speed;

    Cascade_Speed_Output(left_target_speed, right_target_speed);
}

static uint16_t ZigbeeGrey_PacketID = 0;

// 定时器中断控制函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5)
    {
        // uint8_t txbuf[35] =
        //     {
        //         0xAA, 0x55,

        //         0x00, 0x00, // 序号位置：高字节、低字节

        //         0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        //         0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E,
        //         0x0F, 0x10, 0x11, 0x12, 0x13, 0x14,
        //         0x15, 0x16, 0x17, 0x18, 0x19, 0x1A,
        //         0x1B, 0x1C, 0x1D, 0x1E,

        //         0xD1, // 不校验，保留原来的 D1
        //         '\r', '\n'};

        // uint16_t id = ZigbeeGrey_PacketID++;

        // txbuf[2] = (id >> 8) & 0xFF; // 序号高字节
        // txbuf[3] = id & 0xFF;        // 序号低字节

        // ZigbeeGrey_SendArray(txbuf, 35);
    }
    if (htim->Instance == TIM6)
    {
        ZigbeeGrey_Tick(); // 1ms调用一次协议栈的 Tick 函数，进行协议栈的定时处理
    }
    if (htim->Instance == TIM7)
    {
        Encoder_Update(); // 10ms调用一次编码器更新函数，计算速度等信息

        // battery_voltage = Get_battery_volt();

        // if (Location_flag == 1)
        // {
        pid_location.actual_val = Encoder_GetDistanceMm(); // 更新位置环的实际值

        // }
        // 按键（代替放置药品） + 距离环打开 + 角度环关闭，才进行电机控制
        // maix_state不能放在这里，二者不是强绑定关系
        if (car_go == 1 && Location_flag == 1 && Angle_flag == 0)
        {
            Motor_Control(); // 10ms调用一次电机控制函数，根据传感器数据和PID计算调整电机输出
        }
        else if (car_go == 1 && Angle_flag == 1 && Location_flag == 0)
        {
            float angle_out;

            angle_out = angle_pid_realize(&pid_angle, Yaw);
            angle_out = Limit_Float(angle_out, -TURN_TARGET_MAX, TURN_TARGET_MAX);

            /*
             * 原地转向：
             * 左轮目标速度 = -angle_out
             * 右轮目标速度 = +angle_out
             * 再交给速度环输出 PWM
             */
            Cascade_Speed_Output(-angle_out, +angle_out);
        }
        else
        {
            Cascade_Stop();
        }
    }
}

#define DIST_STOP_ERR 10.0f
#define DIST_STABLE_CNT 15 // 15次，如果10ms调用一次就是150ms

uint8_t Distance_task(int16_t distance)
{
    static uint16_t stable_cnt = 0;
    static uint8_t back_init = 0;
    static uint16_t back_zero_wait = 0;
    float err;

    /*
     * 先把本次目标写进去。
     * 关键：不能等陀螺仪清零等待结束后再写，
     * 否则可能沿用上一次正距离目标，导致先往前冲。
     */
    pid_location.target_val = (float)distance;

    if (distance >= 0)
    {
        Location_flag = 1;
        Angle_flag = 0;
        Gray_flag = 1;

        Run_Dir = 1;

        back_init = 0;
        back_zero_wait = 0;
    }
    else
    {
        Run_Dir = -1;

        /*
         * 第一次进入倒退任务：
         * 先停车，清零陀螺仪和角度环。
         */
        if (back_init == 0)
        {
            Location_flag = 0;
            Angle_flag = 0;
            Gray_flag = 0;

            Speed_Set = 0;
            SpeedL = 0;
            SpeedR = 0;

            Cascade_Stop();

            Angle_Hold_Reset_To_Zero();

            back_zero_wait = 0;
            back_init = 1;

            return 0;
        }

        /*
         * 等待维特清零后的 Yaw 数据刷新。
         * 等待期间保持停车，不能让 Motor_Control 跑距离环。
         */
        if (back_zero_wait < 20)
        {
            Location_flag = 0;
            Angle_flag = 0;
            Gray_flag = 0;

            Cascade_Stop();

            Encoder_ResetDistance();

            Angle_Hold_Reset_To_Zero();

            back_zero_wait++;
            return 0;
        }

        /*
         * 清零等待完成后，才真正打开距离环倒退。
         */
        Location_flag = 1;
        Angle_flag = 0;
        Gray_flag = 1;
    }

    err = pid_location.target_val - pid_location.actual_val;

    if (err > -DIST_STOP_ERR && err < DIST_STOP_ERR)
    {
        stable_cnt++;
        Gray_flag = 0;

        if (stable_cnt >= DIST_STABLE_CNT)
        {
            stable_cnt = 0;

            Location_flag = 0;
            Angle_flag = 0;
            Gray_flag = 0;

            Run_Dir = 1;

            back_init = 0;
            back_zero_wait = 0;

            Speed_Set = 0;
            SpeedL = 0;
            SpeedR = 0;

            Cascade_Stop();

            Encoder_ResetDistance();

            return 1;
        }
    }
    else
    {
        stable_cnt = 0;
    }

    return 0;
}

uint8_t turn_init = 0;
uint16_t turn_zero_wait = 0;

uint8_t turn_task(int16_t angle)
{
    float angle_err;

    Location_flag = 0;
    Gray_flag = 0;

    if (turn_init == 0)
    {
        JY61p_HardwareZeroYaw();

        pid_angle.target_val = (float)angle;
        pid_angle.err = 0.0f;
        pid_angle.err_last = 0.0f;
        pid_angle.integral = 0.0f;

        Angle_flag = 0;
        turn_zero_wait = 0;
        turn_init = 1;

        return 0;
    }

    // 等待维特模块清零后的 Yaw 数据刷新
    if (turn_zero_wait < 20)
    {
        turn_zero_wait++;
        return 0;
    }

    Angle_flag = 1;

    angle_err = Angle_Error(pid_angle.target_val, Yaw);

    if (angle_err >= -2.0f && angle_err <= 2.0f)
    {
        delay_ms(800);
        Angle_flag = 0;
        turn_init = 0;
        turn_zero_wait = 0;

        Angle_Hold_Reset_To_Zero();
        Encoder_ResetDistance();

        return 1;
    }

    return 0;
}

void task_1(void)
{
    switch (task_state)
    {
    case 0:
    {
        if (Distance_task(NEAR_RUN1_DISTANCE))
        {
            task_state = 1;
        }
        break;
    }

    case 1:
    {
        if (turn_task(90))
        {
            task_state = 2;
        }
        break;
    }

    case 2:
    {
        if (Distance_task(NEAR_RUN2_DISTANCE))
        {
            task_state = 80; // 到达近端病房，等待取药
        }
        break;
    }

    case 3:
    {
        if (Distance_task(NEAR_BACK1_DISTANCE))
        {
            task_state = 4;
        }
        break;
    }

    case 4:
    {
        if (turn_task(90))
        {
            task_state = 5;
        }
        break;
    }

    case 5:
    {
        if (Distance_task(NEAR_BACK2_DISTANCE))
        {
            task_state = 6;
        }
        break;
    }

    case 6:
    {
        Location_flag = 0;
        Angle_flag = 0;
        Gray_flag = 0;

        Cascade_Stop();

        break;
    }
    default:
    {
        task_state = 0;
        break;
    }
    }
}

void task_2(void)
{
    switch (task_state)
    {
    case 0:
    {
        if (Distance_task(NEAR_RUN1_DISTANCE))
        {
            task_state = 1;
        }
        break;
    }

    case 1:
    {
        if (turn_task(-90))
        {
            task_state = 2;
        }
        break;
    }

    case 2:
    {
        if (Distance_task(NEAR_RUN2_DISTANCE))
        {
            task_state = 80; // 到达近端病房，等待取药
        }
        break;
    }

    case 3:
    {
        if (Distance_task(NEAR_BACK1_DISTANCE))
        {
            task_state = 4;
        }
        break;
    }

    case 4:
    {
        if (turn_task(-90))
        {
            task_state = 5;
        }
        break;
    }

    case 5:
    {
        if (Distance_task(NEAR_BACK2_DISTANCE))
        {
            task_state = 6;
        }
        break;
    }

    case 6:
    {
        Location_flag = 0;
        Angle_flag = 0;
        Gray_flag = 0;

        Motor_SetSpeed(MOTOR_LEFT, 0);
        Motor_SetSpeed(MOTOR_RIGHT, 0);
        break;
    }
    default:
    {
        task_state = 0;
        break;
    }
    }
}

void task_search_middle_then_far(void)
{
    static uint8_t middle_scan_init = 0;
    static uint8_t far_scan_init = 0;
    static uint32_t far_scan_start_tick = 0;
    static uint8_t far_branch_scan_init = 0;
    static uint8_t far_straight_init = 0;
    static uint16_t far_straight_zero_wait = 0;

    switch (task_state)
    {
    case 0:
    {
        /*
         * 从药房出发，前往中端识别点。
         * 这里不是到达后才识别，而是一开始往中端走，就开启 MaixCAM 路口识别。
         */
        if (middle_scan_init == 0)
        {
            maix_state = 1; // 1：路口判断左右
            road_side = 0;
            send_flag = 0;
            maixcam_data_buff[2] = 0;

            middle_scan_init = 1;
        }

        if (Distance_task(MIDDLE_RUN1_DISTANCE))
        {
            task_state = 1;
        }

        break;
    }

    case 1:
    {
        /*
         * 到达中端决策点。
         * 此时 MaixCAM_Proc() 已经在 main while 里持续运行，
         * road_side 应该已经被更新为 Turn_Left / Turn_Right / 0。
         */
        maix_state = 2; // 2：空闲，不再继续识别

        if (road_side == Turn_Left)
        {
            task_state = 2; // 中端左转
        }
        else if (road_side == Turn_Right)
        {
            task_state = 3; // 中端右转
        }
        else
        {
            /*
             * 中端没有识别到目标数字，继续前往远端。
             * 进入远端搜索前要清掉中端识别结果。
             */
            road_side = 0;
            send_flag = 0;
            maixcam_data_buff[2] = 0;

            task_state = 10;
        }

        break;
    }

    case 2:
    {
        // 中端左转
        maix_state = 2;

        if (turn_task(90))
        {
            task_state = 4;
        }

        break;
    }

    case 3:
    {
        // 中端右转
        maix_state = 2;

        if (turn_task(-90))
        {
            task_state = 4;
        }

        break;
    }

    case 4:
    {
        // 进入中端病房
        maix_state = 2;

        if (Distance_task(MIDDLE_RUN2_DISTANCE))
        {
            task_state = 81; // 到达中端病房，等待取药
        }

        break;
    }

    case 5:
    {
        // 倒退出中端病房
        maix_state = 2;

        if (Distance_task(MIDDLE_BACK1_DISTANCE))
        {
            task_state = 6;
        }

        break;
    }

    case 6:
    {
        // 转回药房方向
        maix_state = 2;

        if (road_side == Turn_Left)
        {
            if (turn_task(90))
            {
                task_state = 7;
            }
        }
        else if (road_side == Turn_Right)
        {
            if (turn_task(-90))
            {
                task_state = 7;
            }
        }
        else
        {
            task_state = 98;
        }

        break;
    }

    case 7:
    {
        // 从中端返回药房
        maix_state = 2;

        if (Distance_task(MIDDLE_BACK2_DISTANCE))
        {
            task_state = 99;
        }

        break;
    }

    case 10:
    {
        /*
         * 中端没找到目标后，先往远端方向走一段，
         * 到适合左偏扫描的位置。
         */
        maix_state = 2;

        if (Distance_task(FAR_SCAN_FRONT_DISTANCE))
        {
            road_side = 0;
            send_flag = 0;
            maixcam_data_buff[2] = 0;

            far_scan_init = 0;
            far_scan_start_tick = 0;

            task_state = 11;
        }

        break;
    }

    case 11:
    {
        /*
         * 左转 30°，让摄像头看远端左侧两个数字。
         */
        maix_state = 2;

        if (turn_task(25))
        {
            task_state = 12;
        }

        break;
    }

    case 12:
    {
        /*
         * 左偏 30° 后，发送 0x04。
         * 这里只判断画面里有没有目标数字。
         * 如果识别到了，就认为目标在远端左侧。
         * 如果超时没识别到，就认为目标在远端右侧。
         */
        if (far_scan_init == 0)
        {
            maix_state = 3; // maix_state == 3 时发送 0x04
            road_side = 0;
            send_flag = 0;
            maixcam_data_buff[2] = 0;

            far_scan_start_tick = HAL_GetTick();
            far_scan_init = 1;
        }

        if (road_side == Turn_Left)
        {
            /*
             * MaixCAM_Proc 在 0x04 模式下识别到目标后，
             * 会把 road_side 设置为 Turn_Left。
             */
            maix_state = 2;
            send_flag = 0;
            maixcam_data_buff[2] = 0;

            far_scan_init = 0;
            far_scan_start_tick = 0;

            task_state = 13;
        }
        else if (HAL_GetTick() - far_scan_start_tick >= FAR_SCAN_TIME_MS)
        {
            /*
             * 左侧没找到，默认目标在远端右侧。
             */
            road_side = Turn_Right;

            maix_state = 2;
            send_flag = 0;
            maixcam_data_buff[2] = 0;

            far_scan_init = 0;
            far_scan_start_tick = 0;

            task_state = 13;
        }

        break;
    }

    case 13:
    {
        /*
         * 从左偏 30° 回正。
         */
        maix_state = 2;

        if (turn_task(-25))
        {
            task_state = 14;
            special_flag = 1; //
        }

        break;
    }

    case 14:
    {
        /*
         * 回正后继续走剩余距离，到远端第一次转弯点。
         * FAR_SCAN_REMAIN_DISTANCE = 900 - 650 = 250
         */
        maix_state = 2;

        if (Distance_task(FAR_SCAN_REMAIN_DISTANCE))
        {
            task_state = 15;
            special_flag = 0; // 关闭特殊循迹调整，恢复正常循迹
        }

        break;
    }

    case 15:
    {
        /*
         * 到达远端第一次转弯点。
         * 此时 road_side 表示远端大方向：
         * Turn_Left  -> 远端左侧走廊
         * Turn_Right -> 远端右侧走廊
         *
         * 先保存到 far_first_side，因为后面 0x02 会覆盖 road_side。
         */
        maix_state = 2;

        far_first_side = road_side;

        if (far_first_side == Turn_Left)
        {
            task_state = 16;
        }
        else if (far_first_side == Turn_Right)
        {
            task_state = 17;
        }
        else
        {
            task_state = 98;
        }

        break;
    }

    case 16:
    {
        /*
         * 远端第一次左转，进入左侧远端走廊。
         */
        maix_state = 2;

        if (turn_task(90))
        {
            task_state = 18;
        }

        break;
    }

    case 17:
    {
        /*
         * 远端第一次右转，进入右侧远端走廊。
         */
        maix_state = 2;

        if (turn_task(-90))
        {
            task_state = 18;
        }

        break;
    }

    case 18:
    {
        /*
         * 第一次转弯后，沿远端侧边走廊直行。
         * 这里要像起点到中端一样，一边走一边发 0x02，
         * 判断目标房间在当前走廊的左边还是右边。
         */

        if (far_branch_scan_init == 0)
        {
            maix_state = 1; // maix_state == 1 时发送 0x02
            road_side = 0;
            send_flag = 0;
            maixcam_data_buff[2] = 0;

            far_branch_scan_init = 1;
        }

        /*
         * 第一次进入 case 18：
         * 1. 清零维特 Yaw
         * 2. 把角度环目标设为 0
         * 3. 清空角度环历史误差和积分
         *
         * 注意：
         * 这里不打开 special_flag。
         * case 18 仍然使用正常灰度循迹直走，
         * 这段清零维特只是保留原来的稳定处理。
         */
        if (far_straight_init == 0)
        {
            JY61p_HardwareZeroYaw();

            pid_angle.target_val = 0.0f;
            pid_angle.actual_val = 0.0f;
            pid_angle.err = 0.0f;
            pid_angle.err_last = 0.0f;
            pid_angle.err_prev = 0.0f;
            pid_angle.integral = 0.0f;
            pid_angle.output = 0.0f;

            /*
             * 远端第一次左/右转后，沿侧边走廊直行。
             * 这一段使用：
             * 位置环 + 灰度角度环 + 陀螺仪角度环 + 速度环
             */
            special_flag = 2;

            far_straight_zero_wait = 0;
            far_straight_init = 1;

            break;
        }

        if (far_straight_zero_wait < 20)
        {
            far_straight_zero_wait++;
            break;
        }

        if (Distance_task(FAR_RUN1_DISTANCE))
        {
            maix_state = 2;
            far_branch_scan_init = 0;

            /*
             * 这一段直走结束，关闭特殊角度修正。
             * 后面进入第二次左/右转，由 turn_task 自己接管角度环。
             */
            special_flag = 0;
            far_straight_init = 0;
            far_straight_zero_wait = 0;

            if (road_side == Turn_Left)
            {
                task_state = 19;
            }
            else if (road_side == Turn_Right)
            {
                task_state = 20;
            }
            else
            {
                task_state = 98;
            }
        }

        break;
    }

    case 19:
    {
        /*
         * 第二次左转，进入远端目标病房方向。
         */
        maix_state = 2;

        if (turn_task(90))
        {
            task_state = 21;
        }

        break;
    }

    case 20:
    {
        /*
         * 第二次右转，进入远端目标病房方向。
         */
        maix_state = 2;

        if (turn_task(-90))
        {
            task_state = 21;
        }

        break;
    }

    case 21:
    {
        /*
         * 进入远端病房门口。
         */
        maix_state = 2;

        if (Distance_task(FAR_RUN2_DISTANCE))
        {
            task_state = 82;
        }

        break;
    }

    case 22:
    {
        /*
         * 从远端病房倒退出来。
         */
        maix_state = 2;

        if (Distance_task(FAR_BACK1_DISTANCE))
        {
            task_state = 23;
        }

        break;
    }

    case 23:
    {
        /*
         * 从病房倒出来后，转回远端侧边走廊。
         * 这里用 road_side，因为 road_side 是第二次 0x02 判断的方向。
         */
        maix_state = 2;

        if (road_side == Turn_Left)
        {
            if (turn_task(90))
            {
                task_state = 24;
            }
        }
        else if (road_side == Turn_Right)
        {
            if (turn_task(-90))
            {
                task_state = 24;
            }
        }
        else
        {
            task_state = 98;
        }

        break;
    }

    case 24:
    {
        /*
         * 沿远端侧边走廊返回第一次转弯点。
         */
        maix_state = 2;

        if (Distance_task(FAR_BACK2_DISTANCE))
        {
            task_state = 25;
        }

        break;
    }

    case 25:
    {
        /*
         * 从远端侧边走廊转回主线。
         *
         * 注意这里要用 far_first_side，而且方向要和第一次转弯相反。
         *
         * 如果第一次是左转进入左侧走廊，回来时需要右转回主线。
         * 如果第一次是右转进入右侧走廊，回来时需要左转回主线。
         */
        maix_state = 2;

        if (far_first_side == Turn_Left)
        {
            if (turn_task(-90))
            {
                task_state = 26;
            }
        }
        else if (far_first_side == Turn_Right)
        {
            if (turn_task(90))
            {
                task_state = 26;
            }
        }
        else
        {
            task_state = 98;
        }

        break;
    }

    case 26:
    {
        /*
         * 从远端第一次转弯点返回药房。
         */
        maix_state = 2;

        if (Distance_task(FAR_BACK3_DISTANCE))
        {
            task_state = 99;
        }

        break;
    }

    case 98:
    {
        // 识别失败，停车
        maix_state = 2;

        Location_flag = 0;
        Angle_flag = 0;
        Gray_flag = 0;
        Cascade_Stop();

        // 清除中端 / 远端扫描状态，防止下次任务沿用旧状态
        middle_scan_init = 0;
        far_scan_init = 0;
        far_scan_start_tick = 0;
        far_branch_scan_init = 0;
        far_first_side = 0;

        // 清除 MaixCAM 状态
        road_side = 0;
        send_flag = 0;
        receive_flag = 0;
        maixcam_data_buff[2] = 0;

        break;
    }

    case 99:
    {
        // 任务完成，停车
        maix_state = 2;

        Location_flag = 0;
        Angle_flag = 0;
        Gray_flag = 0;

        Cascade_Stop();

        // 清状态，方便下一次重新执行任务
        middle_scan_init = 0;
        far_scan_init = 0;
        far_scan_start_tick = 0;
        far_branch_scan_init = 0;
        far_first_side = 0;

        // 清除 MaixCAM 状态
        road_side = 0;
        send_flag = 0;
        receive_flag = 0;
        maixcam_data_buff[2] = 0;

        // 如果你要重新开始下一轮识别，可以在合适时机再清 target_number
        // target_number = 0;
        // maixcam_SendPacket(0x03);

        break;
    }

    default:
    {
        maix_state = 2;

        middle_scan_init = 0;
        far_scan_init = 0;
        far_scan_start_tick = 0;
        far_branch_scan_init = 0;
        far_first_side = 0;

        road_side = 0;
        send_flag = 0;
        receive_flag = 0;
        maixcam_data_buff[2] = 0;

        task_state = 0;
        break;
    }
    }
}

// Motor_SetSpeed(MOTOR_LEFT, speed_pid_realize(&pid_speed, Motor1_Speed));
// Motor_SetSpeed(MOTOR_RIGHT, speed_pid_realize(&pid_speed2, Motor2_Speed));

// Motor_SetSpeed(MOTOR_LEFT, location_pid_realize(&pid_location, Encoder_GetDistanceMm()));
// Motor_SetSpeed(MOTOR_RIGHT, location_pid_realize(&pid_location, Encoder_GetDistanceMm()));

// Motor_SetSpeed(MOTOR_LEFT, -angle_pid_realize(&pid_angle, Yaw));
// Motor_SetSpeed(MOTOR_RIGHT, +angle_pid_realize(&pid_angle, Yaw));

// 下面是距离环外环串速度环内环的一份代码，测试下来后，我觉得，距离环和速度环本身是同一类型，都是根据编码器的脉冲，所以不用串起来
// Encoder_Update();

// float target_speed;

// // 1. 距离环：输出目标速度
// target_speed = location_pid_realize(&pid_location, Encoder_GetDistanceMm());

// // 2. 限制最大速度
// target_speed = Limit_Float(target_speed, -20.0f, 20.0f);

// // 3. 把距离环输出作为左右轮速度环目标
// pid_speed.target_val = target_speed;
// pid_speed2.target_val = target_speed;

// // 4. 速度环：输出PWM
// int16_t pwm_left = speed_pid_realize(&pid_speed, encoder_left.diff_cnt);
// int16_t pwm_right = speed_pid_realize(&pid_speed2, encoder_right.diff_cnt);

// Motor_SetSpeed(MOTOR_LEFT, pwm_left);
// Motor_SetSpeed(MOTOR_RIGHT, pwm_right);