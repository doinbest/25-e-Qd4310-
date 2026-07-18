#include "bsp_motor.h"
#include "tim.h"
#include "gpio.h"

#define PWM_MAX 100


static int16_t Motor_Limit(int16_t speed)
{
    if(speed > PWM_MAX)  speed = PWM_MAX;
    if(speed < -PWM_MAX) speed = -PWM_MAX;
    return speed;
}

void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);	//PWMA
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);	//PWMB
    Motor_Stop();
}

/**
  * @brief  设置单个电机的方向和 PWM。
  * @param  motor 左轮或右轮电机。
  * @param  speed PWM 指令，范围为 -100~100；0 表示高阻停止。
  * @retval 无。
  */
void Motor_SetSpeed(Motor_Id_t motor, int16_t speed)
{
    speed = Motor_Limit(speed);

    if(motor == MOTOR_LEFT)
    {
        /* 先同时拉低方向脚，换向过程不会经过 IN1=IN2=1。 */
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);

        if(speed > 0)
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
        }
        else if(speed < 0)
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, -speed);
        }
        else
        {
            /* TB6612 Stop：IN1=IN2=0、PWM=1，输出为高阻态。 */
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM_MAX);
        }
    }
    else if(motor == MOTOR_RIGHT)
    {
        /* 先同时拉低方向脚，换向过程不会经过 IN1=IN2=1。 */
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_RESET);

        if (speed > 0)
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);
        }
        else if (speed < 0)
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, -speed);
        }
        else
        {
            /* TB6612 Stop：IN1=IN2=0、PWM=1，输出为高阻态。 */
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, PWM_MAX);
        }
    }
}

/**
  * @brief  使左右电机进入 TB6612 高阻停止状态。
  * @retval 无。
  */
void Motor_Stop(void)
{
    Motor_SetSpeed(MOTOR_LEFT, 0);
    Motor_SetSpeed(MOTOR_RIGHT, 0);
}
