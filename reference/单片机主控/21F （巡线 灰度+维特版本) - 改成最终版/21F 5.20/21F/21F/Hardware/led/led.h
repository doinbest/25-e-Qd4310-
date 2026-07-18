#ifndef __LED_H
#define	__LED_H

#include "headfile.h"
#define red 0
#define green 1
#define yellow 2

extern uint8_t red_flag;
extern uint8_t green_flag;
extern uint8_t yellow_flag;

void led_show(uint8_t color, uint8_t led_flag);


#endif

