#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H

#include "main.h"

typedef enum
{
    MOTOR_LEFT = 0,
    MOTOR_RIGHT
} Motor_Id_t;

void Motor_Init(void);
void Motor_SetSpeed(Motor_Id_t motor, int16_t speed);
void Motor_Stop(void);



#endif


