#include "led.h"

uint8_t red_flag;
uint8_t green_flag;
uint8_t yellow_flag;


void led_show(uint8_t color, uint8_t led_flag)
{
	switch (color)
	{
		case red:
			if(led_flag == 1)
			{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
			}
		break;
		case green:
			if(led_flag == 1)
			{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
			}	
		break;
		default:
		break;
	}
}