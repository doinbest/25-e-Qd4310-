#ifndef __BSP_KEY_H
#define __BSP_KEY_H

#include <stdint.h>

void key_scan(void);
uint8_t Key_GetGrayCalibrationState(void);
const char *Key_GetGrayCalibrationText(void);

extern uint8_t B1_State, B1_Last_State;
extern uint8_t B2_State, B2_Last_State;
extern uint8_t B3_State, B3_Last_State;
extern uint8_t B4_State, B4_Last_State;
extern uint8_t B5_State, B5_Last_State;

#endif
