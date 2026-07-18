#ifndef __JY61P_H
#define __JY61P_H

#include "headfile.h"

void jy61p_ReceiveData(uint8_t RxData);
void JY61p_HardwareZeroYaw(void);
void JY61p_HardwareZero(void);

extern float Roll,Pitch, Yaw;

#endif
