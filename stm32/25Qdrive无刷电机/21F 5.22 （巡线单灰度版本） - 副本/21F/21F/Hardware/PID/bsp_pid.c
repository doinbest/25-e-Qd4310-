#include "bsp_pid.h"
#include <math.h>

#define SPEED_INTEGRAL_LIMIT  4100.0f
#define SPEED_PWM_LIMIT_MAX   100.0f
#define SPEED_MM_S_PER_COUNT  (300.0f / (11.0f * 4.0f * 50.0f * 0.01f))
#define SPEED_ERROR_DEADBAND_MM_S SPEED_MM_S_PER_COUNT
#define ANGLE_OUTPUT_LIMIT_MM_S 200.0f
#define DR4310_TARGET_X_PX      320.0f
#define DR4310_TARGET_Y_PX      240.0f
#define DR4310_INTEGRAL_LIMIT   200.0f
#define DR4310_OUTPUT_LIMIT_MAX  80.0f

_pid pid_speed;
_pid pid_speed2;
_pid pid_location;
_pid pid_angle;
_pid pid_graysensor;
_pid pid_dr4310X;
_pid pid_dr4310Y;
_pid pid_dr4310X_imu_rate;
_pid pid_dr4310Y_imu_rate;

/**
  * @brief  初始化单个 PID 实例及其参数。
  * @param  pid PID 实例指针。
  * @param  kp 比例系数。
  * @param  ki 积分系数。
  * @param  kd 微分系数。
  * @retval 无。
  */
static void PID_InitInstance(_pid *pid, float kp, float ki, float kd)
{
  pid->target_val = 0.0f;
  pid->actual_val = 0.0f;
  pid->err = 0.0f;
  pid->err_last = 0.0f;
  pid->err_prev = 0.0f;
  pid->Kp = kp;
  pid->Ki = ki; 
  pid->Kd = kd;
  pid->integral = 0.0f;
  pid->output = 0.0f;
}

/**
  * @brief  初始化底盘使用的全部 PID 实例。
  * @retval 无。
  */
void PID_param_init(void)
{
  /* 速度环改用 mm/s，以下数值由旧的“计数/10ms”参数等比例换算。 */
  PID_InitInstance(&pid_speed, 1.0f / SPEED_MM_S_PER_COUNT,
                   0.3f / SPEED_MM_S_PER_COUNT, 0.0f);
  PID_InitInstance(&pid_speed2, 1.0f / SPEED_MM_S_PER_COUNT,
                   0.3f / SPEED_MM_S_PER_COUNT, 0.0f);
  PID_InitInstance(&pid_location, SPEED_MM_S_PER_COUNT, 0.0f, 0.0f);
  PID_InitInstance(&pid_angle, 1.1f * SPEED_MM_S_PER_COUNT,
                   0.0f, 0.1f * SPEED_MM_S_PER_COUNT);
  /* 灰度巡线沿用送药车已验证的 PD 参数。 */
  PID_InitInstance(&pid_graysensor, 10.0f, 0.0f, 25.0f);
  /* 云台视觉外环：像素误差 -> 云台目标 rpm，需在上位机调参。 */
  /* X/Y 均为像素到云台 rpm 的保守起始 P 值。 */
  PID_InitInstance(&pid_dr4310X, 0.30f, 0.0f, 0.0f);
  PID_InitInstance(&pid_dr4310Y, 0.1f, 0.0f, 0.0f);
  /* 0x53 欧拉角两帧差分：云台 rpm 误差 -> QD4310 速度命令 rpm。 */
  PID_InitInstance(&pid_dr4310X_imu_rate, 0.120f, 0.0f, 0.0f);
  PID_InitInstance(&pid_dr4310Y_imu_rate, 0.120f, 0.0f, 0.0f);
  set_pid_target(&pid_dr4310X, DR4310_TARGET_X_PX);
  set_pid_target(&pid_dr4310Y, DR4310_TARGET_Y_PX);
}

float Limit_Float(float val, float min, float max)
{
  if (val > max)
    return max;
  if (val < min)
    return min;
  return val;
}

void PID_Reset(_pid *pid)
{
  pid->target_val = 0.0f; // 目标值也清零

  pid->actual_val = 0.0f;
  pid->err = 0.0f;
  pid->err_last = 0.0f;
  pid->err_prev = 0.0f;
  pid->integral = 0.0f;
  pid->output = 0.0f;
}

void PID_Clear_State(_pid *pid)
{

  pid->actual_val = 0.0f;
  pid->err = 0.0f;
  pid->err_last = 0.0f;
  pid->err_prev = 0.0f;
  pid->integral = 0.0f;
  pid->output = 0.0f;
}

/**
  * @brief  设置目标值
  * @param  val		目标值
	*	@note 	无
  * @retval 无
  */
void set_pid_target(_pid *pid, float temp_val)
{
  pid->target_val = temp_val;    // 设置当前的目标值
}

/**
  * @brief  设置速度环目标，并在进入零目标或反向时清除历史状态。
  * @param  pid 速度环 PID 实例。
  * @param  temp_val 新目标速度，单位为每 10ms 编码器增量。
  * @retval 无。
  */
void set_speed_pid_target(_pid *pid, float temp_val)
{
  float previous_target = pid->target_val;

  if (((previous_target != 0.0f) && (temp_val == 0.0f)) ||
      ((previous_target * temp_val) < 0.0f))
  {
    PID_Clear_State(pid);
  }

  pid->target_val = temp_val;
}

/**
  * @brief  获取目标值
  * @param  无
	*	@note 	无
  * @retval 目标值
  */
float get_pid_target(_pid *pid)
{
  return pid->target_val;    // 设置当前的目标值
}

void set_p_i_d(_pid *pid, float p, float i, float d)
{
  	pid->Kp = p;    // 设置比例系数 P
		pid->Ki = i;    // 设置积分系数 I
		pid->Kd = d;    // 设置微分系数 D
}

/**
  * @brief  位置PID算法实现
  * @param  actual_val:实际值
	*	@note 	无
  * @retval 通过PID计算后的输出
  */
float location_pid_realize(_pid *pid, float actual_val)  //位置环光个Kp好像也可以
{
		/*计算目标值与实际值的误差*/
    pid->err=pid->target_val-actual_val;
  
//    /* 设定闭环死区 */   //外环死区可以不要 
//    if((pid->err >= -0.1) && (pid->err <= 0.1)) 
//    {
//      pid->err = 0;
//      pid->integral = 0;
//    }
    
    pid->integral += pid->err;    // 误差累积

		/*PID算法实现*/
    pid->output = pid->Kp*pid->err
		                  +pid->Ki*pid->integral
		                  +pid->Kd*(pid->err-pid->err_last);
  
		/*误差传递*/
    pid->err_last=pid->err;
    
		/*返回当前实际值*/
    return pid->output;
}

/**
  * @brief  执行一次带积分抗饱和的速度 PID。
  * @param  pid 速度环 PID 实例。
  * @param  actual_val 当前速度，单位为每 10ms 编码器增量。
  * @param  output_limit 本周期 PWM 绝对值上限，范围为 0~100。
  * @retval 限幅后的 PWM 输出，范围为 -output_limit~output_limit。
  */
float speed_pid_realize(_pid *pid, float actual_val, float output_limit)
{
  float derivative;
  float candidate_integral;
  float candidate_output;

  if (output_limit < 0.0f)
  {
    output_limit = -output_limit;
  }
  output_limit = Limit_Float(output_limit, 0.0f, SPEED_PWM_LIMIT_MAX);
  pid->actual_val = actual_val;

  pid->err = pid->target_val - pid->actual_val;
  if ((pid->err > -SPEED_ERROR_DEADBAND_MM_S) &&
      (pid->err < SPEED_ERROR_DEADBAND_MM_S))
  {
    pid->err = 0.0f;
  }

  derivative = pid->err - pid->err_last;
  candidate_integral = Limit_Float(pid->integral + pid->err,
                                   -SPEED_INTEGRAL_LIMIT,
                                   SPEED_INTEGRAL_LIMIT);
  candidate_output = pid->Kp * pid->err +
                     pid->Ki * candidate_integral +
                     pid->Kd * derivative;

  /* 输出饱和且误差仍推动同方向饱和时，暂停积分累加。 */
  if (!(((candidate_output > output_limit) && (pid->err > 0.0f)) ||
        ((candidate_output < -output_limit) && (pid->err < 0.0f))))
  {
    pid->integral = candidate_integral;
  }

  pid->output = pid->Kp * pid->err +
                pid->Ki * pid->integral +
                pid->Kd * derivative;
  pid->output = Limit_Float(pid->output, -output_limit, output_limit);
  pid->err_last = pid->err;

  return pid->output;
}

/**
 * @brief  计算角度误差，自动处理 -180 ~ 180 跳变
 * @param  target: 目标角度，单位：度
 * @param  actual: 实际角度，单位：度
 * @retval 角度误差，范围 -180 ~ 180
 */
float Angle_Error(float target, float actual)
{
  float err;

  /* 先限制 target 到 0~360 */
  while (target >= 360.0f)
  {
    target -= 360.0f;
  }

  while (target < 0.0f)
  {
    target += 360.0f;
  }

  /* 先限制 actual 到 0~360 */
  while (actual >= 360.0f)
  {
    actual -= 360.0f;
  }

  while (actual < 0.0f)
  {
    actual += 360.0f;
  }

  /* 计算误差 */
  err = target - actual;

  /* 处理 0~360 跳变 */
  if (err > 180.0f)
  {
    err -= 360.0f;
  }
  else if (err < -180.0f)
  {
    err += 360.0f;
  }

  return err;
}

/**
 * @brief  角度PID算法实现
 * @param  pid: PID结构体
 * @param  actual_yaw: 实际角度，单位：度
 * @note   角度环输出不是PWM，而是转向修正量 w
 * @retval 角度环输出，作为左右轮速度差使用
 */
float angle_pid_realize(_pid *pid, float actual_yaw)
{
  /* 计算目标角度与实际角度的误差，带角度环绕处理 */
  pid->err = Angle_Error(pid->target_val, actual_yaw);

  /* 误差累积 */
  pid->integral += pid->err;

  /* 积分限幅 */
  if (pid->integral > 10)
  {
    pid->integral = 10;
  }
  else if (pid->integral < -10)
  {
    pid->integral = -10;
  }

  /* PID算法实现 */
  pid->output = pid->Kp * pid->err + pid->Ki * pid->integral + pid->Kd * (pid->err - pid->err_last);

  /* 输出限幅 */
  if (pid->output > ANGLE_OUTPUT_LIMIT_MM_S)
  {
    pid->output = ANGLE_OUTPUT_LIMIT_MM_S;
  }
  else if (pid->output < -ANGLE_OUTPUT_LIMIT_MM_S)
  {
    pid->output = -ANGLE_OUTPUT_LIMIT_MM_S;
  }

  /* 误差传递 */
  pid->err_last = pid->err;

  /* 返回角度环输出 */
  return pid->output;
}




/**
  * @brief  根据灰度位置偏差计算左右轮差速修正量。
  * @param  pid 灰度循迹 PID 实例。
  * @param  actual_val 当前灰度归一化位置，理论范围为 -7*1024~7*1024。
  * @retval 差速修正量，范围限制为 -50~50。
  */
float gray_pid_realize(_pid *pid, float actual_val)
{
  pid->actual_val = actual_val;
  pid->err = pid->target_val - pid->actual_val;
  pid->integral += pid->err;
  pid->integral = Limit_Float(pid->integral, -5.0f, 5.0f);

  pid->output = (pid->Kp * pid->err +
                 pid->Ki * pid->integral +
                 pid->Kd * (pid->err - pid->err_last)) / 1024.0f;
  pid->output = Limit_Float(pid->output, -50.0f, 50.0f);
  pid->err_last = pid->err;

  return pid->output;
}

/**
  * @brief  QD4310 视觉外环 PID，输入像素坐标，输出单位由对应轴决定。
  * @param  output_limit 本轴输出绝对值上限；X轴单位A，Y轴单位rpm。
  * @note   误差方向使用“目标坐标 - 实际坐标”。
  */
float pid_realize_limited(_pid *pid, float actual_val, float output_limit,
                          float integral_limit, float deadband)
{
  float integral_next;
  float output_next;

  if (output_limit < 0.0f)
  {
    output_limit = -output_limit;
  }
  if (integral_limit < 0.0f)
  {
    integral_limit = -integral_limit;
  }

  pid->actual_val = actual_val;
  pid->err = pid->target_val - pid->actual_val;

  if (fabsf(pid->err) <= deadband)
  {
    pid->err = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
    pid->err_last = pid->err;
    return 0.0f;
  }

  integral_next = Limit_Float(pid->integral + pid->err,
                               -integral_limit,
                               integral_limit);
  output_next = pid->Kp * pid->err +
                pid->Ki * integral_next +
                pid->Kd * (pid->err - pid->err_last);
  output_next = Limit_Float(output_next,
                            -output_limit,
                            output_limit);

  if (!(((output_next >= output_limit) && (pid->err > 0.0f)) ||
        ((output_next <= -output_limit) && (pid->err < 0.0f))))
  {
    pid->integral = integral_next;
  }

  pid->output = pid->Kp * pid->err +
                pid->Ki * pid->integral +
                pid->Kd * (pid->err - pid->err_last);
  pid->output = Limit_Float(pid->output,
                            -output_limit,
                            output_limit);
  pid->err_last = pid->err;

  return pid->output;
}

/* 兼容原有调用：视觉外环仍可使用该接口。 */
float dr4310_pid_realize(_pid *pid, float actual_val, float output_limit)
{
  return pid_realize_limited(pid, actual_val, output_limit,
                             DR4310_INTEGRAL_LIMIT, 0.0f);
}
