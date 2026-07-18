#ifndef __DRUG_H
#define __DRUG_H

#include "headfile.h"

extern uint8_t drug_state;

void drug_scan(void);
uint8_t Drug_IsPresent(void);
uint8_t Drug_WaitPlace(void);
uint8_t Drug_WaitRemove(void);

#endif