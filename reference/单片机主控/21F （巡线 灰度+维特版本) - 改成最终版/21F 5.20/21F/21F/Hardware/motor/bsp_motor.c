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

}

void Motor_SetSpeed(Motor_Id_t motor, int16_t speed)
{
    speed = Motor_Limit(speed);

    if(motor == MOTOR_LEFT)
    {
        if(speed > 0)
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
        }
        else if(speed < 0)
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, -speed);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        }
    }
    else if(motor == MOTOR_RIGHT)
    {
        if (speed > 0)
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);
        }
        else if (speed < 0)
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, -speed);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
        }
    }
}

void Motor_Stop(void)
{
    Motor_SetSpeed(MOTOR_LEFT, 0);
    Motor_SetSpeed(MOTOR_RIGHT, 0);
}
