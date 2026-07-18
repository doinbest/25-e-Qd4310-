#include "control.h"
#include "headfile.h"
#include "qdrive/QD4310.h"

#include <stdbool.h>

/* 所有车体运动量均使用 mm、mm/s；PWM 仍使用百分比。 */
#define SPEED_TARGET_LIMIT_MM_S       1100.0f   // 速度限速
#define LINE_TURN_LIMIT_MM_S           200.0f   // 直线巡线时的转向速度限速
#define PWM_OUTPUT_LIMIT               100.0f   // PWM 输出限幅
#define ZERO_SPEED_BRAKE_PWM_LIMIT      30.0f   // 零速刹车时的 PWM 输出限幅
#define SPEED_ZERO_THRESHOLD_MM_S       20.0f   // 低速判定阈值：小于该速度认为已停止，避免 PID 反接电机。

#define LINE_SPEED_GEAR_COUNT             3U    // 直线巡线速度档位数量
#define TARGET_LAP_MIN                     1U   // 最小计圈数
#define TARGET_LAP_MAX                     5U   // 最大计圈数
#define TURNS_PER_LAP                      4U   // 每圈角点数量
#define LAP_DISTANCE_MM                  4000.0f// 每圈总里程，单位 mm；用于完成计圈后回到起点。

#define CORNER_BLACK_COUNT_MIN             4U   //  角点识别时至少 4 个传感器检测到黑线才确认角点
#define CORNER_CONFIRM_SAMPLE_COUNT        3U   //  角点识别时连续采样 3 次确认角点
#define CORNER_REARM_DISTANCE_MM         200.0f //  角点识别后忽略下一段线的距离，避免连续角点误判
#define TURN_COMPENSATION_MM             123.0f //  原地转向前的前进补偿距离，单位 mm；用于角点识别后继续前进一段距离再转向。
#define TURN_APPROACH_SPEED_MM_S         150.0f //  原地转向前的前进补偿阶段的基础速度，单位 mm/s；用于角点识别后继续前进一段距离再转向。
#define TURN_MIN_SPEED_MM_S               80.0f //  原地转向时的最小速度，单位 mm/s；用于角点识别后继续前进一段距离再转向。
#define TURN_STOP_TOLERANCE_MM            10.0f //  原地转向前的前进补偿阶段允许的误差范围，单位 mm；用于角点识别后继续前进一段距离再转向。
#define TURN_STOP_SPEED_MM_S               20.0f    //  原地转向前的前进补偿阶段的停车速度阈值，单位 mm/s；用于角点识别后继续前进一段距离再转向。
#define TURN_SPEED_MAX_MM_S               200.0f    //  原地转向时的最大速度，单位 mm/s；用于角点识别后继续前进一段距离再转向。 
#define TURN_FINISH_ERROR_DEG               2.0f    //  原地转向完成的航向误差阈值，单位度；用于角点识别后继续前进一段距离再转向。
#define FINISH_SLOW_DISTANCE_MM           200.0f    //  完成计圈后回到起点的减速距离阈值，单位 mm；用于角点识别后继续前进一段距离再转向。
#define FINISH_STOP_TOLERANCE_MM            5.0f    //  完成计圈后回到起点的停车误差范围，单位 mm；用于角点识别后继续前进一段距离再转向。
#define REACQUIRE_TIMEOUT_MS             1000U      //  原地转向后重新获取黑线的超时时间，单位 ms；用于角点识别后继续前进一段距离再转向。   

/* MaixCam视觉外环：X轴输出电流A，Y轴输出速度rpm。 */
#define GIMBAL_PIXEL_FILTER_ALPHA             0.25f       //  MaixCam视觉低通滤波系数，0.0~1.0；越大响应越快，越小越平滑。
/* X axis current command slew limit, executed once per 10 ms inner period. */
#define GIMBAL_IMAGE_WIDTH_PX            640U   // MaixCam视觉图像宽度，单位 px
#define GIMBAL_IMAGE_HEIGHT_PX           480U   //  MaixCam视觉图像高度，单位 px
/* 若靶心越调越偏，只把对应宏由 1.0f 改为 -1.0f。 */

/* 当前陀螺仪：左转 Yaw 增大，右转 Yaw 减小。 */
#define TURN_LEFT                         (1)   //  左转
#define TURN_RIGHT                        (-1)
#define GYRO_LEFT_SIGN                    (1.0f)
/* Cascade(-cmd, cmd) 产生左转时保持 +1；若实车相反改为 -1。 */

typedef struct
{
    float previous_actual;
    uint8_t zero_speed_reached;
} SpeedControlState;

typedef enum
{
    LINE_READY = 0,
    LINE_FOLLOW,
    LINE_APPROACH,
    LINE_STOP_FOR_TURN,
    LINE_TURN,
    LINE_REACQUIRE,
    LINE_LOST,
    LINE_DONE
} LineState;

static const float line_speed_gears[LINE_SPEED_GEAR_COUNT] =
    {300.0f, 400.0f, 500.0f};
static uint8_t speed_gear_index = 0U;
static uint8_t target_lap = 1U;
static uint8_t lap_done = 0U;
static uint8_t turn_done = 0U;
static int8_t run_direction = 0;
static int8_t corner_direction = 0;
static LineState line_state = LINE_READY;
static float corner_start_distance_mm = 0.0f;
static float corner_ignore_until_mm = 0.0f;
static float turn_target_yaw_deg = 0.0f;
static float turn_start_raw_distance_mm = 0.0f;
static float turn_distance_offset_mm = 0.0f;
static uint8_t corner_confirm_count = 0U;
static uint8_t reacquire_confirm_count = 0U;
static uint32_t reacquire_start_tick_ms = 0U;
static uint32_t last_gray_sample = 0U;

static SpeedControlState left_speed_state = {0.0f, 0U};
static SpeedControlState right_speed_state = {0.0f, 0U};
/* X 轴使用 USART2，Y 轴使用 UART5；两台 QD4310 分别独占串口，ID 都是 0。 */
static QD4310_t gimbal_motor_x = {false, 0U, 0.0f, 0.0f, 0.0f, &huart2};
static QD4310_t gimbal_motor_y = {false, 0U, 0.0f, 0.0f, 0.0f, &huart5};
static bool gimbal_active = false;
static bool gimbal_x_enabled = false;
static bool gimbal_y_enabled = false;
static bool gimbal_target_lost = false;
static bool gimbal_rate_debug = false;
static uint8_t gimbal_task_mode = 1U;

/**
 * @brief 获取仅包含直线阶段的累计里程。
 * @note  原地转向期间左右轮不一致造成的净编码器里程会被扣除。
 */
static float get_run_distance_mm(void)
{
    return Encoder_GetDistanceMm() - turn_distance_offset_mm;
}

int16_t left_pwm = 0;
int16_t right_pwm = 0;
int16_t speed_target_mm_s = 0;
int result_angle = 0;

uint8_t control_running = 0;

static void speed_state_reset(SpeedControlState *state)
{
    state->previous_actual = 0.0f;
    state->zero_speed_reached = 0U;
}

static float speed_loop_output(_pid *pid, SpeedControlState *state,
                               float target_speed_mm_s, float actual_speed_mm_s,
                               float output_limit)
{
    float output;

    set_speed_pid_target(pid, target_speed_mm_s);

    if (target_speed_mm_s != 0.0f)
    {
        state->zero_speed_reached = 0U;
    }
    else
    {
        if (state->zero_speed_reached != 0U)
        {
            pid->output = 0.0f;
            return 0.0f;
        }

        if ((fabsf(actual_speed_mm_s) <= SPEED_ZERO_THRESHOLD_MM_S) ||
            ((state->previous_actual > 0.0f) && (actual_speed_mm_s < 0.0f)) ||
            ((state->previous_actual < 0.0f) && (actual_speed_mm_s > 0.0f)))
        {
            PID_Clear_State(pid);
            state->zero_speed_reached = 1U;
            return 0.0f;
        }
    }

    output = speed_pid_realize(pid, actual_speed_mm_s, output_limit);

    /* 非零目标不允许反接电机，测速毛刺时只松油门。 */
    if (((target_speed_mm_s > 0.0f) && (output < 0.0f)) ||
        ((target_speed_mm_s < 0.0f) && (output > 0.0f)))
    {
        output = 0.0f;
    }

    state->previous_actual = actual_speed_mm_s;
    return output;
}

static void set_wheel_speed(float left_target_mm_s, float right_target_mm_s)
{
    float left_output_pwm;
    float right_output_pwm;
    float left_limit;
    float right_limit;

    left_target_mm_s = Limit_Float(left_target_mm_s,
                                   -SPEED_TARGET_LIMIT_MM_S,
                                   SPEED_TARGET_LIMIT_MM_S);
    right_target_mm_s = Limit_Float(right_target_mm_s,
                                    -SPEED_TARGET_LIMIT_MM_S,
                                    SPEED_TARGET_LIMIT_MM_S);

    left_limit = (left_target_mm_s == 0.0f) ? ZERO_SPEED_BRAKE_PWM_LIMIT : PWM_OUTPUT_LIMIT;
    right_limit = (right_target_mm_s == 0.0f) ? ZERO_SPEED_BRAKE_PWM_LIMIT : PWM_OUTPUT_LIMIT;

    left_output_pwm = speed_loop_output(&pid_speed, &left_speed_state,
                                        left_target_mm_s, encoder_left.speed_mm_s, left_limit);
    right_output_pwm = speed_loop_output(&pid_speed2, &right_speed_state,
                                         right_target_mm_s, encoder_right.speed_mm_s, right_limit);

    left_pwm = (int16_t)Limit_Float(left_output_pwm, -left_limit, left_limit);
    right_pwm = (int16_t)Limit_Float(right_output_pwm, -right_limit, right_limit);
    Motor_SetSpeed(MOTOR_LEFT, (int16_t)left_pwm);
    Motor_SetSpeed(MOTOR_RIGHT, (int16_t)right_pwm);
}

static void stop_chassis(void)
{
    PID_Reset(&pid_speed);
    PID_Reset(&pid_speed2);
    speed_state_reset(&left_speed_state);
    speed_state_reset(&right_speed_state);
    speed_target_mm_s = 0;
    left_pwm = 0;
    right_pwm = 0;
    Motor_Stop();
}

static uint8_t get_black_count(void)
{
    uint8_t count = 0U;
    uint8_t i;

    for (i = 0U; i < 8U; i++)
    {
        if ((Digtal & (1U << i)) != 0U)
        {
            count++;
        }
    }
    return count;
}

static int8_t get_corner_direction(void)
{
    /* 传感器 1~8 从左到右；归一化位置小于零即黑线偏左。 */
    return (result_angle < 0) ? TURN_LEFT : TURN_RIGHT;
}

static const char *get_line_state_text(void)
{
    switch (line_state)
    {
    case LINE_FOLLOW: return "FOLLOW";
    case LINE_APPROACH: return "APPROACH";
    case LINE_STOP_FOR_TURN: return "STOP";
    case LINE_TURN: return "TURN";
    case LINE_REACQUIRE: return "FIND";
    case LINE_LOST: return "LOST";
    case LINE_DONE: return "DONE";
    default: return "READY";
    }
}

static void line_follow_update(float base_speed_mm_s)
{
    float turn_speed_mm_s;

    if (Digtal == 0U)
    {
        line_state = LINE_LOST;
        control_running = 0U;
stop_chassis();
        return;
    }
set_pid_target(&pid_graysensor, 0.0f);
    turn_speed_mm_s = gray_pid_realize(&pid_graysensor, (float)result_angle);
    turn_speed_mm_s = Limit_Float(turn_speed_mm_s,
                                  -LINE_TURN_LIMIT_MM_S,
                                  LINE_TURN_LIMIT_MM_S);

    speed_target_mm_s = (int)base_speed_mm_s;
    set_wheel_speed(base_speed_mm_s - turn_speed_mm_s,
                         base_speed_mm_s + turn_speed_mm_s);
}

/**
 * @brief 灰度 PID 上位机调参时的直线巡线控制。
 * @note  不进入计圈、角点识别状态机；基础速度由 B4 选择。
 */
static void gray_debug_update(void)
{
    float base_speed_mm_s;
    float turn_speed_mm_s;

    if (Digtal == 0U)
    {
        control_running = 0U;
stop_chassis();
        return;
    }

    base_speed_mm_s = control_get_line_speed_mm_s();
set_pid_target(&pid_graysensor, 0.0f);
    turn_speed_mm_s = gray_pid_realize(&pid_graysensor, (float)result_angle);
    turn_speed_mm_s = Limit_Float(turn_speed_mm_s,
                                  -LINE_TURN_LIMIT_MM_S,
                                  LINE_TURN_LIMIT_MM_S);

    speed_target_mm_s = (int)base_speed_mm_s;
    set_wheel_speed(base_speed_mm_s - turn_speed_mm_s,
                         base_speed_mm_s + turn_speed_mm_s);
}

static void start_turn(void)
{
    turn_start_raw_distance_mm = Encoder_GetDistanceMm();
    turn_target_yaw_deg = Yaw + (float)corner_direction * GYRO_LEFT_SIGN * 90.0f;
    while (turn_target_yaw_deg > 180.0f) turn_target_yaw_deg -= 360.0f;
    while (turn_target_yaw_deg <= -180.0f) turn_target_yaw_deg += 360.0f;

    PID_Clear_State(&pid_angle);
    set_pid_target(&pid_angle, turn_target_yaw_deg);
    line_state = LINE_TURN;
}

static void finish_turn(void)
{
    /* 删除本次原地转向造成的净编码器里程，巡线总里程只统计直线。 */
    turn_distance_offset_mm +=
        Encoder_GetDistanceMm() - turn_start_raw_distance_mm;

    turn_done++;
    lap_done = turn_done / TURNS_PER_LAP;
    corner_ignore_until_mm = get_run_distance_mm() + CORNER_REARM_DISTANCE_MM;
    reacquire_confirm_count = 0U;
    reacquire_start_tick_ms = HAL_GetTick();
    PID_Clear_State(&pid_graysensor);

    line_state = LINE_REACQUIRE;
}

/**
 * @brief 到达累计目标距离后结束任务，停车点即为起始位置。
 */
static void finish_at_start(void)
{
    lap_done = target_lap;
    line_state = LINE_DONE;
    control_running = 0U;
    stop_chassis();
}

static void line_control_update(void)
{
    float moved_mm;
    float remain_mm;
    float target_speed_mm_s;
    float angle_output_mm_s;

    switch (line_state)
    {
    case LINE_FOLLOW:
        /* 只有完成全部 4 x 圈数 个角点转向后，才开始按总里程回到起点。 */
        if (turn_done >= (uint8_t)(target_lap * TURNS_PER_LAP))
        {
            remain_mm = (float)target_lap * LAP_DISTANCE_MM -
                        get_run_distance_mm();
            if (remain_mm <= FINISH_STOP_TOLERANCE_MM)
            {
                finish_at_start();
            }
            else if (remain_mm < FINISH_SLOW_DISTANCE_MM)
            {
                target_speed_mm_s = Limit_Float(remain_mm * 1.5f,
                                                 TURN_MIN_SPEED_MM_S,
                                                 TURN_APPROACH_SPEED_MM_S);
                line_follow_update(target_speed_mm_s);
            }
            else
            {
                line_follow_update(line_speed_gears[speed_gear_index]);
            }
        }
        else
        {
            line_follow_update(line_speed_gears[speed_gear_index]);
        }
        break;

    case LINE_APPROACH:
        moved_mm = get_run_distance_mm() - corner_start_distance_mm;
        remain_mm = TURN_COMPENSATION_MM - moved_mm;

        /* 单向前进补偿：进入目标前 ±10 mm 范围或略微越过目标即可转向。 */
        if (remain_mm <= TURN_STOP_TOLERANCE_MM)
        {
            line_state = LINE_STOP_FOR_TURN;
            set_wheel_speed(0.0f, 0.0f);
            break;
        }
        /* 从检测到角点起就降低速度，末端进一步按剩余距离减速。 */
        target_speed_mm_s = Limit_Float(remain_mm * 1.5f,
                                        TURN_MIN_SPEED_MM_S,
                                        TURN_APPROACH_SPEED_MM_S);
        speed_target_mm_s = (int)target_speed_mm_s;
        set_wheel_speed(target_speed_mm_s, target_speed_mm_s);
        break;

    case LINE_STOP_FOR_TURN:
        set_wheel_speed(0.0f, 0.0f);
        if ((fabsf(encoder_left.speed_mm_s) <= TURN_STOP_SPEED_MM_S) &&
            (fabsf(encoder_right.speed_mm_s) <= TURN_STOP_SPEED_MM_S))
        {
            start_turn();
        }
        break;

    case LINE_TURN:
        angle_output_mm_s = angle_pid_realize(&pid_angle, Yaw);
        angle_output_mm_s = Limit_Float(angle_output_mm_s,
                                         -TURN_SPEED_MAX_MM_S,
                                         TURN_SPEED_MAX_MM_S);

        if (fabsf(pid_angle.err) <= TURN_FINISH_ERROR_DEG)
        {
            stop_chassis();
            finish_turn();
            break;
        }

        speed_target_mm_s = (int)angle_output_mm_s;
        set_wheel_speed(-angle_output_mm_s, angle_output_mm_s);
        break;

    case LINE_REACQUIRE:
        if ((HAL_GetTick() - reacquire_start_tick_ms) >= REACQUIRE_TIMEOUT_MS)
        {
            line_state = LINE_LOST;
            control_running = 0U;
            stop_chassis();
        }
        else
        {
            /* 转角后允许短暂全白，低速直行等待下一段线进入传感器。 */
            if (Digtal == 0U)
            {
                set_wheel_speed(TURN_APPROACH_SPEED_MM_S,
                                     TURN_APPROACH_SPEED_MM_S);
            }
            else
            {
                line_follow_update(TURN_APPROACH_SPEED_MM_S);
            }
        }
        break;

    case LINE_LOST:
    case LINE_DONE:
    case LINE_READY:
    default:
        stop_chassis();
        break;
    }
}

void control_line_sensor_update(void)
{
    uint8_t black_count;
    float distance_mm;

    if (last_gray_sample == ZigbeeGrey_SampleCount)
    {
        return;
    }
    last_gray_sample = ZigbeeGrey_SampleCount;

    if ((control_running == 0U) || (Car_Mode != Run_Mode))
    {
        return;
    }

    black_count = get_black_count();
    distance_mm = get_run_distance_mm();

    if (line_state == LINE_REACQUIRE)
    {
        if ((Digtal != 0U) && (black_count < CORNER_BLACK_COUNT_MIN))
        {
            reacquire_confirm_count++;
            if (reacquire_confirm_count >= CORNER_CONFIRM_SAMPLE_COUNT)
            {
                line_state = LINE_FOLLOW;
            }
        }
        else
        {
            reacquire_confirm_count = 0U;
        }
        return;
    }

    if ((line_state != LINE_FOLLOW) || (distance_mm < corner_ignore_until_mm))
    {
        return;
    }

    if (black_count >= CORNER_BLACK_COUNT_MIN)
    {
        corner_confirm_count++;
        if (corner_confirm_count >= CORNER_CONFIRM_SAMPLE_COUNT)
        {
            corner_direction = (run_direction == 0) ? get_corner_direction() : run_direction;
            if (run_direction == 0)
            {
                run_direction = corner_direction;
            }

            corner_start_distance_mm = distance_mm;
            corner_confirm_count = 0U;
            PID_Clear_State(&pid_graysensor);
            line_state = LINE_APPROACH;
        }
    }
    else
    {
        corner_confirm_count = 0U;
    }
}

void control_next_speed_gear(void)
{
    speed_gear_index = (uint8_t)((speed_gear_index + 1U) % LINE_SPEED_GEAR_COUNT);
}

uint8_t control_get_speed_gear(void)
{
    return (uint8_t)(speed_gear_index + 1U);
}

float control_get_line_speed_mm_s(void)
{
    return line_speed_gears[speed_gear_index];
}

uint8_t control_next_lap_target(void)
{
    target_lap++;
    if (target_lap > TARGET_LAP_MAX) target_lap = TARGET_LAP_MIN;
    return target_lap;
}

uint8_t control_get_lap_target(void) { return target_lap; }
uint8_t control_get_lap_done(void) { return lap_done; }
uint8_t control_get_turn_done(void) { return turn_done; }
float control_get_run_distance_mm(void) { return get_run_distance_mm(); }
const char *control_get_line_state_text(void) { return get_line_state_text(); }

void control_debug_stop(void)
{
    PID_Clear_State(&pid_speed);
    PID_Clear_State(&pid_speed2);
    PID_Clear_State(&pid_location);
    PID_Clear_State(&pid_angle);
    PID_Clear_State(&pid_graysensor);
    PID_Clear_State(&pid_dr4310X);
    PID_Clear_State(&pid_dr4310Y);
    speed_state_reset(&left_speed_state);
    speed_state_reset(&right_speed_state);
    speed_target_mm_s = 0;
    left_pwm = 0;
    right_pwm = 0;
    Motor_Stop();
}

void control_start(void)    //
{
    if (line_state == LINE_LOST)
    {
        return;
    }

    control_debug_stop();
    /* 每次任务从 B3 启动时仅清零一次航向，后续转角保持连续累计。 */
    JY61p_HardwareZeroYaw();
    Encoder_ResetDistance();
    turn_start_raw_distance_mm = 0.0f;
    turn_distance_offset_mm = 0.0f;
    lap_done = 0U;
    turn_done = 0U;
    run_direction = 0;
    corner_direction = 0;
    corner_confirm_count = 0U;
    corner_ignore_until_mm = CORNER_REARM_DISTANCE_MM;
    line_state = LINE_FOLLOW;

control_running = 1U;
}

void control_stop(void)
{
    control_running = 0U;

if (line_state != LINE_LOST)
    {
        line_state = LINE_READY;
    }
    control_debug_stop();
}

void control_reset(void)
{
    control_running = 0U;
    lap_done = 0U;
    turn_done = 0U;
    run_direction = 0;
    corner_direction = 0;
    corner_confirm_count = 0U;
    line_state = LINE_READY;
    control_debug_stop();
    Encoder_ResetDistance();
    turn_start_raw_distance_mm = 0.0f;
    turn_distance_offset_mm = 0.0f;
    JY61p_HardwareZero();
}

/* K5 competition reset: preserve gray black/white calibration arrays. */
void control_competition_reset(void)
{
    control_reset();
    target_lap = TARGET_LAP_MIN;

Car_Mode = Run_Mode;

    gimbal_disable_all();
    MaixCam_LaserOff();
}

/**
 * @brief  判断 MaixCam 靶心坐标是否新鲜且处于 640x480 图像范围内。
 */
/* Dual-axis external cascade: pixel -> gyro rate -> target motor speed ->
   QD4310 internal speed/current loops. */
#define GIMBAL_INNER_PERIOD_MS             5U   // 5毫秒内部周期时间
#define GIMBAL_IMU_TIMEOUT_MS             100U  // 100毫秒IMU超时
#define GIMBAL_TARGET_TIMEOUT_MS          200U  // 200毫秒目标超时
#define GIMBAL_CONTROL_DT_S                 0.005f  //  5毫秒控制周期时间
#define GIMBAL_MAX_GIMBAL_RPM               60.0f   //  最大云台转速60RPM
#define GIMBAL_MAX_SPEED_RPM                60.0f   //  最大速度60RPM
#define GIMBAL_PIXEL_DEADBAND_PX            1.0f    //  像素死区1.0px
#define GIMBAL_RATE_DEADBAND_RPM            0.05f   //  速度死区0.05RPM
#define GIMBAL_PID_INTEGRAL_LIMIT         300.0f    //  PID积分限幅300.0
#define GIMBAL_RATE_PID_INTEGRAL_LIMIT      20.0f   //  速度PID积分限幅20.0
#define GIMBAL_COMMAND_FAILURE_LIMIT        3U      //  命令失败次数限制3次
#define GIMBAL_FRESH_TIMEOUT_MS             800U    //  启动时等待首个有效IMU帧的超时

/* Board IMU gyro-axis mapping. Change signs only after a low-speed test. */
#define GIMBAL_X_GYRO_AXIS MAIXCAM_GYRO_AXIS_Z
#define GIMBAL_Y_GYRO_AXIS MAIXCAM_GYRO_AXIS_Y
#define GIMBAL_X_GYRO_SIGN (-1.0f)
#define GIMBAL_Y_GYRO_SIGN (1.0f)
#define GIMBAL_X_SPEED_SIGN   (-1.0f)   //  云台X轴速度符号对应MaixCAM2的偏航轴符号
#define GIMBAL_Y_SPEED_SIGN    (1.0f)   //  云台Y轴速度符号对应MaixCAM2的俯仰轴符号
#define GIMBAL_X_PIXEL_SIGN    (1.0f)   //  云台X轴像素符号对应MaixCAM2的偏航轴符号
#define GIMBAL_Y_PIXEL_SIGN    (1.0f)   //  云台Y轴像素符号对应MaixCAM2的俯仰轴符号

typedef enum { GIMBAL_AXIS_X = 0U, GIMBAL_AXIS_Y, GIMBAL_AXIS_COUNT } GimbalAxis;  //  云台轴索引枚举类型
typedef enum
{
    GIMBAL_MODE_CURRENT_TEST = 0U,  //  云台模式：电流测试
    GIMBAL_MODE_IMU_RATE_TEST,  //  云台模式：IMU速率测试
    GIMBAL_MODE_PIXEL_TRACK //  云台模式：像素跟踪
} GimbalMode;

typedef struct
{
    QD4310_t *motor;    //  云台电机结构体指针
    _pid *pixel_pid;    //  云台像素PID结构体指针
    _pid *imu_rate_pid; //  云台IMU速率PID结构体指针
    bool *enabled;   //  云台使能标志指针
    uint8_t gyro_axis;      //  云台角速度轴索引
    float gyro_sign;        //  云台角速度符号
    float speed_sign;       //  云台速度符号
    float pixel_sign;       //  云台像素符号
    float measured_rate_rpm;    //  云台测量速率(RPM)
    uint32_t last_imu_count;     //  云台上一次IMU帧计数
    uint8_t imu_rate_ready;      //  已使用至少一帧有效角速度
    float target_rate_rpm;  //  云台目标速率(RPM)
    float target_speed_rpm; //  云台目标速度(RPM)
    float speed_command_rpm;//  云台速度命令(RPM)
    float test_current_a;// 云台测试电流(A)
    float test_imu_rate_rpm;    //  云台测试IMU速率(RPM)
    float pixel_filtered;       //  云台像素滤波值
    uint8_t pixel_filter_ready; //  云台像素滤波准备标志
    uint32_t last_vision_count; //  云台上一次视觉计数
} GimbalAxisState;    //  云台轴结构体类型

static GimbalAxisState gimbal_axes[GIMBAL_AXIS_COUNT] =   //  云台轴结构体数组
{
    {&gimbal_motor_x, &pid_dr4310X, &pid_dr4310X_imu_rate,  //  云台X轴结构体初始化
     &gimbal_x_enabled, GIMBAL_X_GYRO_AXIS, GIMBAL_X_GYRO_SIGN,
     GIMBAL_X_SPEED_SIGN, GIMBAL_X_PIXEL_SIGN},
    {&gimbal_motor_y, &pid_dr4310Y, &pid_dr4310Y_imu_rate,
     &gimbal_y_enabled, GIMBAL_Y_GYRO_AXIS, GIMBAL_Y_GYRO_SIGN,
     GIMBAL_Y_SPEED_SIGN, GIMBAL_Y_PIXEL_SIGN}
};
    
static GimbalMode gimbal_mode = GIMBAL_MODE_PIXEL_TRACK; //  云台模式初始化为像素跟踪模式
static uint32_t gimbal_last_inner_tick_ms = 0U;    //  云台上一次内部周期时间戳
static bool gimbal_serial_fault = false;        //  云台串口故障标志

typedef enum    //  云台过渡状态枚举类型
{
    GIMBAL_TRANSITION_IDLE = 0U,    //  云台过渡状态：空闲
    GIMBAL_TRANSITION_WAIT_FRESH,   //  云台过渡状态：等待新鲜帧
    GIMBAL_TRANSITION_STOPPED,      //  云台过渡状态：已停止
    GIMBAL_TRANSITION_FAULT         //  云台过渡状态：故障
} GimbalTransition;        //  云台过渡状态类型

static GimbalTransition gimbal_transition = GIMBAL_TRANSITION_IDLE;  //  云台过渡状态初始化为空闲
static bool gimbal_transition_start_mode = false;       //  云台过渡开始模式
static uint8_t gimbal_transition_pending_mode = Run_Mode;   //  云台过渡挂起模式
static uint32_t gimbal_transition_tick_ms = 0U;    //  云台过渡时间戳
static float gimbal_read_gyro_dps(uint8_t axis)
{
    if (axis == MAIXCAM_GYRO_AXIS_X) return maixcam_gyro_x_dps;
    if (axis == MAIXCAM_GYRO_AXIS_Y) return maixcam_gyro_y_dps;
    return maixcam_gyro_z_dps;
}

static uint8_t gimbal_imu_fresh(void)
{
    uint8_t flags = MAIXCAM_IMU_FLAG_VALID | MAIXCAM_IMU_FLAG_CALIBRATED;
    return ((maixcam_imu_frame_count != 0U) &&
            ((maixcam_imu_flags & flags) == flags) &&
            ((HAL_GetTick() - maixcam_last_imu_tick) <= GIMBAL_IMU_TIMEOUT_MS)) ? 1U : 0U;
}

static uint8_t gimbal_vision_fresh(void) //  判断云台视觉数据是否新鲜的函数
{
    uint8_t flags = MAIXCAM_FLAG_TARGET | MAIXCAM_FLAG_MAPPING;
    return ((maixcam_frame_count != 0U) &&
            ((HAL_GetTick() - maixcam_last_rx_tick) <= GIMBAL_TARGET_TIMEOUT_MS) &&
            ((maixcam_vision_flags & flags) == flags)) ? 1U : 0U;
}

static void gimbal_clear_imu_rate(GimbalAxisState *axis)
{
    axis->measured_rate_rpm = 0.0f;
    axis->last_imu_count = maixcam_imu_frame_count;
    axis->imu_rate_ready = 0U;
}

static void gimbal_clear_axis(GimbalAxisState *axis)   //  清除云台轴数据函数
{
    axis->target_rate_rpm = 0.0f;
    axis->target_speed_rpm = 0.0f;
    axis->speed_command_rpm = 0.0f;
    gimbal_clear_imu_rate(axis);
    axis->pixel_filter_ready = 0U;
    axis->last_vision_count = maixcam_frame_count;
    PID_Clear_State(axis->pixel_pid);
    PID_Clear_State(axis->imu_rate_pid);
}

static void gimbal_update_imu_rate(void)
{
    uint8_t index;
    if (gimbal_imu_fresh() == 0U) return;

    for (index = 0U; index < GIMBAL_AXIS_COUNT; index++)
    {
        GimbalAxisState *axis = &gimbal_axes[index];
        if (axis->last_imu_count == maixcam_imu_frame_count) continue;
        axis->last_imu_count = maixcam_imu_frame_count;
        axis->measured_rate_rpm = axis->gyro_sign *
                                  gimbal_read_gyro_dps(axis->gyro_axis) / 6.0f;
        axis->imu_rate_ready = 1U;
    }
}

static uint8_t gimbal_imu_rate_ready(const GimbalAxisState *axis)
{
    return axis->imu_rate_ready;
}

static float gimbal_pixel_actual(GimbalAxis index)  //  获取云台像素实际值函数
{
    /* PID target is zero.  Camera error is aim - target, so PID actual is -error. */
    return (index == GIMBAL_AXIS_X) ? -maixcam_error_x_px : -maixcam_error_y_px;
}

static float gimbal_pixel_target(GimbalAxis index)  //  获取云台像素目标值函数
{
    (void)index;
    return 0.0f;
}

static void gimbal_update_pixel_target(GimbalAxis index) //  更新云台像素目标值函数
{
    GimbalAxisState *axis = &gimbal_axes[index];
    float actual;
    float correction;
    if (axis->last_vision_count == maixcam_frame_count) return;
    axis->last_vision_count = maixcam_frame_count;
    actual = gimbal_pixel_actual(index);
    if (axis->pixel_filter_ready == 0U)
    {
        axis->pixel_filtered = actual;
        axis->pixel_filter_ready = 1U;
    }
    else axis->pixel_filtered += GIMBAL_PIXEL_FILTER_ALPHA * (actual - axis->pixel_filtered);

    set_pid_target(axis->pixel_pid, gimbal_pixel_target(index));
    correction = pid_realize_limited(axis->pixel_pid, axis->pixel_filtered,
                                     GIMBAL_MAX_GIMBAL_RPM,
                                     GIMBAL_PID_INTEGRAL_LIMIT,
                                     GIMBAL_PIXEL_DEADBAND_PX);
    axis->target_rate_rpm = axis->pixel_sign * correction;
}

static void gimbal_command_speed(GimbalAxisState *axis)    //  向云台轴发送速度命令函数
{
    uint8_t ok;
    /* Send the current PID output directly; no per-period slew limiter is used
       so the tuning page observes the actual PID response. */
    axis->speed_command_rpm = Limit_Float(axis->target_speed_rpm,
                                           -GIMBAL_MAX_SPEED_RPM,
                                           GIMBAL_MAX_SPEED_RPM);
    ok = QD4310_SetSpeed(axis->motor, axis->speed_sign * axis->speed_command_rpm);
    if ((ok == 0U) && (axis->motor->consecutive_failures >= GIMBAL_COMMAND_FAILURE_LIMIT))
        gimbal_serial_fault = 1U;
}

/* This PID is used only for the gimbal gyro-rate speed loop.
   Kp: motor-rpm/gimbal-rpm, Ki: motor-rpm/(gimbal-rpm*s),
   Kd: motor-rpm*s/gimbal-rpm. */
static float gimbal_rate_pid_update(_pid *pid, float actual_rpm) //  云台速率PID实现函数
{
    float integral_next;
    float output_next;
    float derivative;

    pid->actual_val = actual_rpm;
    pid->err = pid->target_val - pid->actual_val;
    if (fabsf(pid->err) <= GIMBAL_RATE_DEADBAND_RPM)
    {
        pid->err = 0.0f;
        pid->integral = 0.0f;
        pid->output = 0.0f;
        pid->err_last = 0.0f;
        return 0.0f;
    }

    integral_next = Limit_Float(pid->integral + pid->err * GIMBAL_CONTROL_DT_S,
                                -GIMBAL_RATE_PID_INTEGRAL_LIMIT,
                                GIMBAL_RATE_PID_INTEGRAL_LIMIT);
    derivative = (pid->err - pid->err_last) / GIMBAL_CONTROL_DT_S;
    output_next = pid->Kp * pid->err + pid->Ki * integral_next + pid->Kd * derivative;
    output_next = Limit_Float(output_next, -GIMBAL_MAX_SPEED_RPM, GIMBAL_MAX_SPEED_RPM);

    if (!(((output_next >= GIMBAL_MAX_SPEED_RPM) && (pid->err > 0.0f)) ||
          ((output_next <= -GIMBAL_MAX_SPEED_RPM) && (pid->err < 0.0f))))
    {
        pid->integral = integral_next;
    }

    pid->output = pid->Kp * pid->err + pid->Ki * pid->integral + pid->Kd * derivative;
    pid->output = Limit_Float(pid->output, -GIMBAL_MAX_SPEED_RPM, GIMBAL_MAX_SPEED_RPM);
    pid->err_last = pid->err;
    return pid->output;
}

static void gimbal_stop_outputs(void)    //  停止云台输出函数
{
    uint8_t index;
    for (index = 0U; index < GIMBAL_AXIS_COUNT; index++)
    {
        GimbalAxisState *axis = &gimbal_axes[index];
        axis->target_speed_rpm = 0.0f;
        axis->speed_command_rpm = 0.0f;
        axis->test_current_a = 0.0f;
        axis->test_imu_rate_rpm = 0.0f;
        set_pid_target(axis->pixel_pid, 0.0f);
        set_pid_target(axis->imu_rate_pid, 0.0f);
        PID_Clear_State(axis->pixel_pid);
        PID_Clear_State(axis->imu_rate_pid);
        if (*(axis->enabled) != 0U) (void)QD4310_SetSpeed(axis->motor, 0.0f);
    }
}

static void gimbal_run_axis(GimbalAxisState *axis, GimbalAxis index)  //  运行云台轴函数
{
    if (*(axis->enabled) == 0U) return;
    if (gimbal_mode == GIMBAL_MODE_CURRENT_TEST)
    {
        (void)QD4310_SetCurrent(axis->motor, axis->test_current_a);
        return;
    }
    if (gimbal_imu_rate_ready(axis) == 0U)
    {
        axis->target_speed_rpm = 0.0f;
        axis->speed_command_rpm = 0.0f;
        (void)QD4310_SetSpeed(axis->motor, 0.0f);
        return;
    }
    else if (gimbal_mode == GIMBAL_MODE_IMU_RATE_TEST)
    {
        axis->target_rate_rpm = axis->test_imu_rate_rpm;
        set_pid_target(axis->imu_rate_pid, axis->target_rate_rpm);
        axis->target_speed_rpm = gimbal_rate_pid_update(axis->imu_rate_pid,
                                                        axis->measured_rate_rpm);
    }
    else
    {
        if (gimbal_mode == GIMBAL_MODE_PIXEL_TRACK) gimbal_update_pixel_target(index);
        set_pid_target(axis->imu_rate_pid, axis->target_rate_rpm);
        axis->target_speed_rpm = gimbal_rate_pid_update(axis->imu_rate_pid,
                                                        axis->measured_rate_rpm);
    }
    gimbal_command_speed(axis);
}

static void gimbal_enable_axis(GimbalAxis index)    //  使能云台轴函数
{
    GimbalAxisState *axis = &gimbal_axes[index];
    if (*(axis->enabled) == 0U)
    {
        if (QD4310_Enable(axis->motor) == false) gimbal_serial_fault = 1U;
        *(axis->enabled) = 1U;
    }
    gimbal_clear_axis(axis);
}

static void gimbal_disable_axis(GimbalAxis index)   //  禁用云台轴函数
{
    GimbalAxisState *axis = &gimbal_axes[index];
    /* Always transmit the safe state.  The MCU may have just reset while the
       QD4310 is still enabled, so the software enable flag cannot be trusted. */
    (void)QD4310_SetCurrent(axis->motor, 0.0f);
    (void)QD4310_Disable(axis->motor);
    *(axis->enabled) = 0U;
    gimbal_clear_axis(axis);
}

void gimbal_start(void) //  启动视觉控制函数
{
    if ((gimbal_imu_fresh() == 0U) || (gimbal_vision_fresh() == 0U))
    {
        gimbal_target_lost = 1U;
        return;
    }
    gimbal_mode = GIMBAL_MODE_PIXEL_TRACK;
    gimbal_active = 1U;
    gimbal_target_lost = 0U;
    gimbal_last_inner_tick_ms = 0U;
    gimbal_clear_axis(&gimbal_axes[GIMBAL_AXIS_X]);
    gimbal_clear_axis(&gimbal_axes[GIMBAL_AXIS_Y]);
}

void gimbal_start_both(void)   //  启动视觉控制并使能两个电机函数
{
    gimbal_serial_fault = 0U;
    gimbal_enable_axis(GIMBAL_AXIS_X);
    gimbal_enable_axis(GIMBAL_AXIS_Y);
    gimbal_start();
}

void gimbal_start_axis(uint8_t axis)   //  启动视觉控制并使能指定轴函数
{
    if (axis >= GIMBAL_AXIS_COUNT) return;
    gimbal_serial_fault = 0U;
    gimbal_enable_axis((GimbalAxis)axis);
    gimbal_start();
}

void gimbal_start_pixel_debug(uint8_t axis) { gimbal_start_axis(axis); } //  启动像素调试函数

void gimbal_stop(void)  //  停止视觉控制函数
{
    gimbal_active = 0U;
    gimbal_target_lost = 0U;
    gimbal_rate_debug = 0U;
    gimbal_stop_outputs();
}

void gimbal_toggle(void) { if (gimbal_active != 0U) gimbal_stop(); else gimbal_start(); }   //  切换视觉控制状态函数
void gimbal_toggle_x_motor(void) { if (gimbal_x_enabled != 0U) gimbal_disable_axis(GIMBAL_AXIS_X); else gimbal_enable_axis(GIMBAL_AXIS_X); }  //  切换X轴电机状态函数
void gimbal_toggle_y_motor(void) { if (gimbal_y_enabled != 0U) gimbal_disable_axis(GIMBAL_AXIS_Y); else gimbal_enable_axis(GIMBAL_AXIS_Y); }  //  切换Y轴电机状态函数

void gimbal_start_rate_debug(uint8_t axis) //  启动IMU速率调试函数
{
    float saved_target_rpm;
    if (axis >= GIMBAL_AXIS_COUNT) return;
    saved_target_rpm = gimbal_axes[axis].test_imu_rate_rpm;
    gimbal_stop();
    gimbal_serial_fault = 0U;
    gimbal_enable_axis((GimbalAxis)axis);
    gimbal_axes[axis].test_imu_rate_rpm = Limit_Float(saved_target_rpm,
                                                       -GIMBAL_MAX_GIMBAL_RPM,
                                                       GIMBAL_MAX_GIMBAL_RPM);
    set_pid_target(gimbal_axes[axis].imu_rate_pid, gimbal_axes[axis].test_imu_rate_rpm);
    gimbal_mode = GIMBAL_MODE_IMU_RATE_TEST;
    gimbal_active = 1U;
    gimbal_rate_debug = 1U;
    gimbal_last_inner_tick_ms = 0U;
}
void gimbal_start_current_debug(uint8_t axis, float current_a)    //  启动电流调试函数
{
    if (axis >= GIMBAL_AXIS_COUNT) return;
    gimbal_stop();
    gimbal_serial_fault = 0U;
    gimbal_enable_axis((GimbalAxis)axis);
    gimbal_axes[axis].test_current_a = Limit_Float(current_a, -QD4310_MAX_CURRENT, QD4310_MAX_CURRENT);
    gimbal_mode = GIMBAL_MODE_CURRENT_TEST;
    gimbal_active = 1U;
    gimbal_last_inner_tick_ms = 0U;
}
void gimbal_set_rate_target(uint8_t axis, float target_rpm)   //  设置IMU速率调试目标函数
{
    if (axis >= GIMBAL_AXIS_COUNT) return;
    gimbal_axes[axis].test_imu_rate_rpm = Limit_Float(target_rpm,
                                                       -GIMBAL_MAX_GIMBAL_RPM,
                                                       GIMBAL_MAX_GIMBAL_RPM);
    set_pid_target(gimbal_axes[axis].imu_rate_pid, gimbal_axes[axis].test_imu_rate_rpm);
}
void gimbal_set_pixel_target(uint8_t axis, float target_px) { (void)axis; (void)target_px; }
void gimbal_set_task_mode(uint8_t mode) { gimbal_task_mode = mode; }
void gimbal_next_task_mode(void) { gimbal_task_mode++; }
uint8_t gimbal_get_task_mode(void) { return gimbal_task_mode; }

void gimbal_disable_all(void)  //  禁用所有电机函数
{
    gimbal_stop();
    gimbal_disable_axis(GIMBAL_AXIS_X);
    gimbal_disable_axis(GIMBAL_AXIS_Y);
}

static void gimbal_transition_fault(void)    //  云台过渡故障处理函数
{
    gimbal_disable_all();
    gimbal_transition_start_mode = 0U;
    gimbal_transition = GIMBAL_TRANSITION_FAULT;
}

static void gimbal_start_pending_mode(void)   //  启动挂起模式函数
{
    if (gimbal_transition_pending_mode == X_Rate_Mode)
    {
        gimbal_start_rate_debug(GIMBAL_AXIS_X);
    }
    else if (gimbal_transition_pending_mode == Y_Rate_Mode)
    {
        gimbal_start_rate_debug(GIMBAL_AXIS_Y);
    }
    else if (gimbal_transition_pending_mode == X_Mode)
    {
        if ((gimbal_imu_fresh() == 0U) || (gimbal_vision_fresh() == 0U))
        {
            gimbal_target_lost = 1U;
            gimbal_transition = GIMBAL_TRANSITION_STOPPED;
            return;
        }
        gimbal_start_pixel_debug(GIMBAL_AXIS_X);
    }
    else if (gimbal_transition_pending_mode == Y_Mode)
    {
        if ((gimbal_imu_fresh() == 0U) || (gimbal_vision_fresh() == 0U))
        {
            gimbal_target_lost = 1U;
            gimbal_transition = GIMBAL_TRANSITION_STOPPED;
            return;
        }
        gimbal_start_pixel_debug(GIMBAL_AXIS_Y);
    }
    else
    {
        gimbal_transition = GIMBAL_TRANSITION_STOPPED;
        return;
    }

    gimbal_transition = GIMBAL_TRANSITION_IDLE;
}

static void gimbal_service_transition(void)  //  云台过渡服务函数
{
    uint32_t now = HAL_GetTick();

    if (gimbal_transition != GIMBAL_TRANSITION_WAIT_FRESH) return;
    if ((now - gimbal_transition_tick_ms) > GIMBAL_FRESH_TIMEOUT_MS)
    {
        gimbal_transition_fault();
        return;
    }

    if ((gimbal_imu_fresh() == 0U) ||
        (gimbal_imu_rate_ready(&gimbal_axes[GIMBAL_AXIS_X]) == 0U) ||
        (gimbal_imu_rate_ready(&gimbal_axes[GIMBAL_AXIS_Y]) == 0U))
    {
        return;
    }

    if (gimbal_transition_start_mode != 0U)
    {
        gimbal_start_pending_mode();
    }
    else
    {
        gimbal_transition = GIMBAL_TRANSITION_STOPPED;
    }
}

static void gimbal_request_transition(uint8_t mode, uint8_t start_mode)  //  请求云台过渡函数
{
    /* Repeated page/STOP presses share the in-flight IMU freshness wait.  The
       latest page wins, while STOP always overrides automatic re-enable. */
    if (gimbal_transition == GIMBAL_TRANSITION_WAIT_FRESH)
    {
        gimbal_transition_pending_mode = mode;
        /* STOP is sticky for the active transaction.  A later page key may
           select the next page but cannot re-enable motors until a new
           transaction is started from STOPPED. */
        if (start_mode == 0U)
            gimbal_transition_start_mode = 0U;
        return;
    }

    gimbal_disable_all();
    gimbal_transition_pending_mode = mode;
    gimbal_transition_start_mode = start_mode;
    gimbal_transition_tick_ms = HAL_GetTick();
    gimbal_clear_imu_rate(&gimbal_axes[GIMBAL_AXIS_X]);
    gimbal_clear_imu_rate(&gimbal_axes[GIMBAL_AXIS_Y]);
    gimbal_transition = GIMBAL_TRANSITION_WAIT_FRESH;
}

void gimbal_request_mode(uint8_t mode) //  请求云台模式函数
{
    if (mode == X_Rate_Mode)
    {
        gimbal_set_rate_target(GIMBAL_AXIS_X, 0.0f);
    }
    else if (mode == Y_Rate_Mode)
    {
        gimbal_set_rate_target(GIMBAL_AXIS_Y, 0.0f);
    }
    gimbal_request_transition(mode, 1U);
}

void gimbal_request_stop(void) //  请求云台停止函数
{
    gimbal_request_transition(Car_Mode, 0U);
}

void gimbal_task(void)  //  视觉控制任务函数
{
    uint32_t now = HAL_GetTick();   //  获取当前时间戳
    gimbal_update_imu_rate();
    gimbal_service_transition();     //  服务云台过渡状态
    if (gimbal_transition != GIMBAL_TRANSITION_IDLE) return;  //  如果云台过渡状态不为空闲，则返回
    if (gimbal_active == 0U) return;    //  如果视觉控制不活跃，则返回
    if ((now - gimbal_last_inner_tick_ms) < GIMBAL_INNER_PERIOD_MS) return;    //  如果距离上一次内部周期时间戳小于云台内部周期时间，则返回
    gimbal_last_inner_tick_ms = now;   //  更新云台上一次内部周期时间戳为当前时间戳
    if ((gimbal_imu_fresh() == 0U) || (gimbal_serial_fault != 0U) ||
        ((gimbal_mode == GIMBAL_MODE_PIXEL_TRACK) && (gimbal_vision_fresh() == 0U)))
    {
        gimbal_target_lost = 1U;    //  设置视觉目标丢失标志为1
        /* Fault stop order: speed=0, current=0, then disable both motors. */
        gimbal_stop_outputs();   //  停止云台输出
        gimbal_disable_axis(GIMBAL_AXIS_X);  //  禁用云台X轴
        gimbal_disable_axis(GIMBAL_AXIS_Y);  //  禁用云台Y轴
        gimbal_active = 0U; //  设置视觉控制活跃标志为0
        return; 
    }
    gimbal_run_axis(&gimbal_axes[GIMBAL_AXIS_X], GIMBAL_AXIS_X); //  运行云台X轴
    gimbal_run_axis(&gimbal_axes[GIMBAL_AXIS_Y], GIMBAL_AXIS_Y); //  运行云台Y轴
}

uint8_t gimbal_is_active(void) { return gimbal_active; } //  判断视觉控制是否活跃函数
uint8_t gimbal_target_valid(void) { return gimbal_vision_fresh(); }//  判断视觉目标是否有效函数
uint8_t gimbal_rate_debug_active(void) { return gimbal_rate_debug; }//   判断速率调试是否活跃函数
uint8_t gimbal_x_motor_enabled(void) { return gimbal_x_enabled; } //  判断X轴电机是否使能函数
uint8_t gimbal_y_motor_enabled(void) { return gimbal_y_enabled; } //  判断Y轴电机是否使能函数
const char *gimbal_get_state_text(void)   //  获取视觉控制状态文本函数
{
    if (gimbal_transition == GIMBAL_TRANSITION_FAULT) return "MAIX";  //  云台过渡故障
    if (gimbal_transition == GIMBAL_TRANSITION_WAIT_FRESH) return "FRESH";    //  云台过渡等待新鲜帧
    if (gimbal_transition == GIMBAL_TRANSITION_STOPPED) return "STOP";        //  云台过渡已停止
    if (gimbal_serial_fault != 0U) return "SER";        //  云台串口故障
    if (gimbal_target_lost != 0U) return "LOST";        //  视觉目标丢失
    return (gimbal_active != 0U) ? "RUN" : "OFF";   //  返回视觉控制状态文本
}
float gimbal_get_x_rate_rpm(void) { return gimbal_axes[GIMBAL_AXIS_X].measured_rate_rpm; }    //  获取云台X轴速率函数
float gimbal_get_y_rate_rpm(void) { return gimbal_axes[GIMBAL_AXIS_Y].measured_rate_rpm; }    //  获取云台Y轴速率函数
float gimbal_get_x_cmd_rpm(void) { return gimbal_axes[GIMBAL_AXIS_X].speed_command_rpm; }   //  获取云台X轴电机速度命令函数
float gimbal_get_y_cmd_rpm(void) { return gimbal_axes[GIMBAL_AXIS_Y].speed_command_rpm; }   //  获取云台Y轴电机速度命令函数
float gimbal_get_x_rate_target_rpm(void) { return gimbal_axes[GIMBAL_AXIS_X].target_rate_rpm; }   //  获取云台X轴速率命令函数
float gimbal_get_y_rate_target_rpm(void) { return gimbal_axes[GIMBAL_AXIS_Y].target_rate_rpm; }   //  获取云台Y轴速率命令函数
float gimbal_get_x_pixel_px(void) { return gimbal_pixel_actual(GIMBAL_AXIS_X); }    //  获取云台X轴像素实际值函数
float gimbal_get_y_pixel_px(void) { return gimbal_pixel_actual(GIMBAL_AXIS_Y); }    //  获取云台Y轴像素实际值函数
float gimbal_get_x_pixel_target_px(void) { return gimbal_pixel_target(GIMBAL_AXIS_X); }    //  获取云台X轴像素目标值函数
float gimbal_get_y_pixel_target_px(void) { return gimbal_pixel_target(GIMBAL_AXIS_Y); }    //  获取云台Y轴像素目标值函数

void control_debug_update_10ms(void)   //  PID调试控制函数，每10毫秒执行一次
{
    float output;

    /* Gimbal modes are scheduled by gimbal_task in the main loop. */
    if ((Car_Mode == Run_Mode) || (Car_Mode == X_Rate_Mode) ||
        (Car_Mode == Y_Rate_Mode) || (Car_Mode == X_Mode) ||
        (Car_Mode == Y_Mode)) return;
    if (control_running == 0U)
    {
        control_debug_stop();
        return;
    }

    switch (Car_Mode)
    {
    case Speed_Mode:
        speed_target_mm_s = (int)pid_speed.target_val;
        set_wheel_speed(pid_speed.target_val, 0.0f);
        break;
    case Speed2_Mode:
        speed_target_mm_s = (int)pid_speed2.target_val;
        set_wheel_speed(0.0f, pid_speed2.target_val);
        break;
    case Location_Mode:
        output = location_pid_realize(&pid_location, pid_location.actual_val);
        set_wheel_speed(output, output);
        break;
    case Angle_Mode:
        output = angle_pid_realize(&pid_angle, Yaw);
        set_wheel_speed(-output, output);
        break;
    case Gray_Mode:
        gray_debug_update();
        break;
    default:
        control_debug_stop();
        break;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        /* 1 kHz 灰度传感器时基；实际采样和归一化在主循环中执行。 */
        ZigbeeGrey_Tick();
    }
    else if (htim->Instance == TIM7)
    {
        Encoder_Update();
        pid_location.actual_val = get_run_distance_mm();

        if (Car_Mode == Run_Mode)
        {
            if (control_running != 0U) line_control_update();
            else if ((line_state != LINE_LOST) && (line_state != LINE_DONE)) stop_chassis();
        }
        else
        {
            control_debug_update_10ms();
        }
    }
}
