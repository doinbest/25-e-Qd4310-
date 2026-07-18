#ifndef NO_MCU_GANV_GRAYSCALE_SENSOR_CONFIG_H_
#define NO_MCU_GANV_GRAYSCALE_SENSOR_CONFIG_H_

#include <string.h>
#include "adc.h"
#include "stm32f1xx_hal.h"

#define Class 0

#define _14Bits 0
#define _12Bits 1
#define _10Bits 2
#define _8Bits  3

#define Sensor_Edition Class
#define Direction 1
#define Sensor_ADCbits _12Bits
#define Use_Timer 1

#define Switch_Address_0(i) ((i) ? (HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)) : (HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)))
#define Switch_Address_1(i) ((i) ? (HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET)) : (HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET)))
#define Switch_Address_2(i) ((i) ? (HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET)) : (HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET)))

#define Get_adc_of_user() adc_getValue()

typedef struct
{
    unsigned short Analog_value[8];
    unsigned short Normal_value[8];
    unsigned short Calibrated_white[8];
    unsigned short Calibrated_line[8];
    unsigned short Line_threshold[8];
    unsigned char Line_is_low[8];
    double Normal_factor[8];
    double bits;
    unsigned char Digtal;
    unsigned char Time_out;
    unsigned char Tick;
    unsigned char ok;
} No_MCU_Sensor;

#ifdef __cplusplus
extern "C" {
#endif

void No_MCU_Ganv_Sensor_Init_Frist(No_MCU_Sensor *sensor);
void No_MCU_Ganv_Sensor_Init(No_MCU_Sensor *sensor,
                             unsigned short *Calibrated_white,
                             unsigned short *Calibrated_line);

#ifndef Use_Timer
void No_Mcu_Ganv_Sensor_Task_Without_tick(No_MCU_Sensor *sensor);
#else
void No_Mcu_Ganv_Sensor_Task_With_tick(No_MCU_Sensor *sensor);
void Task_tick(No_MCU_Sensor *sensor);
#endif

unsigned char Get_Digtal_For_User(No_MCU_Sensor *sensor);
unsigned char Get_Normalize_For_User(No_MCU_Sensor *sensor, unsigned short *result);
unsigned char Get_Anolog_Value(No_MCU_Sensor *sensor, unsigned short *result);

#ifdef __cplusplus
}
#endif

#endif
