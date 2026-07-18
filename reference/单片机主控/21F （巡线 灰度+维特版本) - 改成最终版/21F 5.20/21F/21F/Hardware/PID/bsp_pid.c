#include "bsp_pid.h"

//定义全局变量

_pid pid_speed, pid_speed2; // 速度环
_pid pid_location; // 位置环
_pid pid_turn;  // 转向环 Gary
_pid pid_angle; // 角度环  wit
_pid pid_maixcam; // MaixCam巡线PID
/**
  * @brief  PID参数初始化
	*	@note 	无
  * @retval 无
  */
void PID_param_init()
{
  	/* 速度相关初始化参数 */
    pid_speed.target_val=0.0;				
    pid_speed.actual_val=0.0;
    pid_speed.err=0.0;
    pid_speed.err_last=0.0;
    pid_speed.integral=0.0;
    pid_speed.output = 0.0;

		pid_speed.Kp = 1.0;
		pid_speed.Ki = 0.3;
		pid_speed.Kd = 0.0;
		
  	/* 速度相关初始化参数 */
    pid_speed2.target_val=0.0;				
    pid_speed2.actual_val=0.0;
    pid_speed2.err=0.0;
    pid_speed2.err_last=0.0;
    pid_speed2.integral=0.0;
    pid_speed2.output = 0.0;
  
		pid_speed2.Kp = 1.0;
		pid_speed2.Ki = 0.3;
		pid_speed2.Kd = 0.0;

    /* 位置相关初始化参数 */
    pid_location.target_val = 0.0;
    pid_location.actual_val = 0.0;
    pid_location.err = 0.0;
    pid_location.err_last = 0.0;
    pid_location.integral = 0.0;
    pid_location.output = 0.0;

    pid_location.Kp = 3.5;
    pid_location.Ki = 0.0;
    pid_location.Kd = 4.0;

    /* 转向相关初始化参数 */
    pid_turn.target_val=0.0;				
    pid_turn.actual_val=0.0;
    pid_turn.err=0.0;
    pid_turn.err_last=0.0;
    pid_turn.integral=0.0;
    pid_turn.output = 0.0;

    pid_turn.Kp = 0.0;
    pid_turn.Ki = 0.0;
    pid_turn.Kd = 0.0;

    /* 角度相关初始化参数 */
    pid_angle.target_val=0.0;
    pid_angle.actual_val=0.0;
    pid_angle.err=0.0;         
    pid_angle.err_last=0.0;
    pid_angle.integral=0.0;
    pid_angle.output = 0.0;

    pid_angle.Kp = 2.5;
    pid_angle.Ki = 0.01;
    pid_angle.Kd = 5;

    pid_maixcam.target_val = 0.0;
    pid_maixcam.actual_val = 0.0;   
    pid_maixcam.err = 0.0;
    pid_maixcam.err_last = 0.0;
    pid_maixcam.integral = 0.0;
    pid_maixcam.output = 0.0;

    pid_maixcam.Kp = 5.0;
    pid_maixcam.Ki = 0.0;
    pid_maixcam.Kd = 15.0;

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
  pid->target_val = 0.0f;
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
  * @brief  速度PID算法实现
  * @param  actual_val:实际值
	*	@note 	无
  * @retval 通过PID计算后的输出
  */

// float speed_pid_realize(_pid *pid, float actual_val)
// {
//   float delta_output;

//   pid->actual_val = actual_val;

//   /* 计算当前误差 e(k) */
//   pid->err = pid->target_val - pid->actual_val;

//   /*
//    * 目标速度为 0 时，直接清零
//    * 防止停车后 output 残留，电机还继续转
//    */
//   if (pid->target_val == 0.0f)
//   {
//     pid->actual_val = 0.0f;
//     pid->err = 0.0f;
//     pid->err_last = 0.0f;
//     pid->err_prev = 0.0f;
//     pid->integral = 0.0f;
//     pid->output = 0.0f;
//     return 0.0f;
//   }

//   /*
//    * 增量式 PID:
//    *
//    * Δu(k) = Kp * [e(k) - e(k-1)]
//    *       + Ki * e(k)
//    *       + Kd * [e(k) - 2e(k-1) + e(k-2)]
//    */
//   delta_output = pid->Kp * (pid->err - pid->err_last) + pid->Ki * pid->err + pid->Kd * (pid->err - 2.0f * pid->err_last + pid->err_prev);

//   /* 当前输出 = 上一次输出 + 本次增量 */
//   pid->output += delta_output;


//   if (pid->output > 100.0f)
//   {
//     pid->output = 100.0f;
//   }
//   else if (pid->output < -100.0f)
//   {
//     pid->output = -100.0f;
//   }
//   /*
//    * 更新误差
//    * 顺序不能反
//    */
//   pid->err_prev = pid->err_last;
//   pid->err_last = pid->err;

//   return pid->output;
// }

// 位置式 速度环 PID，保留之前的实现，方便调试对比
float speed_pid_realize(_pid *pid, float actual_val)
{
  pid->actual_val = actual_val;

  /*
   * 目标速度为 0 时，直接清零，防止停车后积分残留
   */
  if (pid->target_val == 0.0f)
  {
    PID_Reset(pid);
    return 0.0f;
  }

  pid->err = pid->target_val - pid->actual_val;

  /*
   * 小死区，防止速度误差很小时反复抖动
   * diff_cnt 作为速度输入时，这里用 1 比较合适
   */
  if (pid->err > -1.0f && pid->err < 1.0f)
  {
    pid->err = 0.0f;
  }

  pid->integral += pid->err;

  /*
   * 积分限幅
   */
  pid->integral = Limit_Float(pid->integral, -300.0f, 300.0f);

  pid->output = pid->Kp * pid->err + pid->Ki * pid->integral + pid->Kd * (pid->err - pid->err_last);

  pid->err_last = pid->err;

  /*
   * PWM 输出限幅，和 Motor_SetSpeed 的 -100~100 保持一致
   */
  pid->output = Limit_Float(pid->output, -100.0f, 100.0f);

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
  if (pid->output > 30.0)
  {
    pid->output = 30.0;
  }
  else if (pid->output < -30.0)
  {
    pid->output = -30.0;
  }

  /* 误差传递 */
  pid->err_last = pid->err;

  /* 返回角度环输出 */
  return pid->output;
}

