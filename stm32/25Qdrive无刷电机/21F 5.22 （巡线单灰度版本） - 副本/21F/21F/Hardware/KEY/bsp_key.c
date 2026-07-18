#include "bsp_key.h"
#include "headfile.h"

uint8_t B1_State = 1U, B1_Last_State = 1U;
uint8_t B2_State = 1U, B2_Last_State = 1U;
uint8_t B3_State = 1U, B3_Last_State = 1U;
uint8_t B4_State = 1U, B4_Last_State = 1U;
uint8_t B5_State = 1U, B5_Last_State = 1U;

typedef enum
{
    GRAY_CAL_READY = 0U,
    GRAY_CAL_WAIT_BLACK
} Gray_Calibration_State_t;

static Gray_Calibration_State_t Gray_Calibration_State = GRAY_CAL_READY;

uint8_t Key_GetGrayCalibrationState(void)
{
    return (uint8_t)Gray_Calibration_State;
}

const char *Key_GetGrayCalibrationText(void)
{
    return (Gray_Calibration_State == GRAY_CAL_WAIT_BLACK) ? "BLACK" : "READY";
}

static void Key_CopyGraySample(unsigned short *destination)
{
    uint8_t index;
    for (index = 0U; index < ZIGBEE_GREY_CHANNEL_NUM; index++)
    {
        destination[index] = ZigbeeGrey_Analog[index];
    }
}

/**
 * @brief Scan the five gimbal keys every 20 ms.
 * K1: set lap target 1..5. K2: reset and start the chassis task.
 * K3: laser on plus both-axis visual closed loop. K4: white/black calibration.
 * K5: competition reset that preserves the gray calibration values.
 */
void key_scan(void)
{
    static uint32_t last_scan_time = 0U;
    uint32_t now = HAL_GetTick();

    if ((now - last_scan_time) < 20U) return;
    last_scan_time = now;

    B1_State = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
    B2_State = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
    B3_State = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
    B4_State = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8);
    B5_State = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9);

    if ((B1_State == 0U) && (B1_Last_State != 0U))
    {
        if (control_running == 0U)
        {
            control_next_lap_target();
        }
    }

    if ((B2_State == 0U) && (B2_Last_State != 0U))
    {
        if (control_running == 0U)
        {
            control_reset();
            control_start();
        }
    }

    if ((B3_State == 0U) && (B3_Last_State != 0U))
    {
        if ((gimbal_x_motor_enabled() != 0U) ||
            (gimbal_y_motor_enabled() != 0U))
        {
            /* B3 second press: stop visual control and de-energize both axes. */
            gimbal_disable_all();
            MaixCam_LaserOff();
        }
        else
        {
            MaixCam_LaserOn();
            gimbal_set_pixel_target(0U, 310.0f);
            gimbal_set_pixel_target(1U, 202.0f);
            gimbal_start_both();
        }
    }

    if ((B4_State == 0U) && (B4_Last_State != 0U))
    {
        control_stop();

        if (Gray_Calibration_State != GRAY_CAL_WAIT_BLACK)
        {
            Key_CopyGraySample(ZigbeeGrey_White);
            Gray_Calibration_State = GRAY_CAL_WAIT_BLACK;
        }
        else
        {
            Key_CopyGraySample(ZigbeeGrey_Black);
            No_MCU_Ganv_Sensor_Init(&ZigbeeGrey_Sensor,
                                    ZigbeeGrey_White,
                                    ZigbeeGrey_Black);
            Gray_Calibration_State = GRAY_CAL_READY;
        }
    }

    if ((B5_State == 0U) && (B5_Last_State != 0U))
    {
        control_competition_reset();
        /* Do not alter ZigbeeGrey_White/Black; only cancel a half-finished K4 flow. */
        Gray_Calibration_State = GRAY_CAL_READY;
    }

    B1_Last_State = B1_State;
    B2_Last_State = B2_State;
    B3_Last_State = B3_State;
    B4_Last_State = B4_State;
    B5_Last_State = B5_State;
}
