#ifndef __BSP_KEY_H
#define	__BSP_KEY_H

#include "headfile.h"

void key_scan(void);

extern uint8_t B1_State, B1_Last_State;	// PB0
extern uint8_t B2_State, B2_Last_State;	// PB1
extern uint8_t B3_State, B3_Last_State;	// PB2
extern uint8_t B4_State, B4_Last_State;	// PC8	RCT6上的K3
extern uint8_t B5_State, B5_Last_State;	// PC9	RCT6上的K4
extern uint8_t B4_flag; // 校准白色标志
extern uint8_t B5_flag; // 校准黑色标志

extern uint8_t Calib_Step;
extern uint8_t ZigbeeGrey_White_OK;
extern uint8_t ZigbeeGrey_Black_OK;

#endif

