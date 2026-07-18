#ifndef __MAIXCAM_H
#define __MAIXCAM_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "headfile.h"

/* MaixCAM2 UART4 <-> STM32 USART3, 115200 8N1.
 * All multi-byte V1 fields use big-endian byte order. */
#define MAIXCAM_SOF0                    0x55U
#define MAIXCAM_SOF1                    0xAAU
#define MAIXCAM_PROTOCOL_VERSION        0x01U

#define MAIXCAM_TYPE_VISION             0x01U
#define MAIXCAM_TYPE_IMU                0x02U
#define MAIXCAM_TYPE_CONTROL            0x81U

#define MAIXCAM_VISION_PAYLOAD_SIZE      9U
#define MAIXCAM_IMU_PAYLOAD_SIZE        12U
#define MAIXCAM_CONTROL_PAYLOAD_SIZE     3U
#define MAIXCAM_MAX_PAYLOAD_SIZE        16U
#define MAIXCAM_FRAME_OVERHEAD           9U
#define MAIXCAM_MAX_FRAME_SIZE          (MAIXCAM_FRAME_OVERHEAD + MAIXCAM_MAX_PAYLOAD_SIZE)
#define MAIXCAM_RX_RING_SIZE           256U

/* Vision FLAGS sent by MaixCAM2. */
#define MAIXCAM_FLAG_TARGET              0x01U
#define MAIXCAM_FLAG_MAPPING             0x02U
#define MAIXCAM_FLAG_LOCKED              0x04U
#define MAIXCAM_FLAG_CONTROL_FRESH       0x08U

/* IMU FLAGS sent by MaixCAM2. */
#define MAIXCAM_IMU_FLAG_VALID           0x01U
#define MAIXCAM_IMU_FLAG_CALIBRATED      0x02U

/* Control FLAGS sent by STM32. */
#define MAIXCAM_CONTROL_FLAG_LASER       0x01U

#define MAIXCAM_GYRO_AXIS_X              0U
#define MAIXCAM_GYRO_AXIS_Y              1U
#define MAIXCAM_GYRO_AXIS_Z              2U

#define MAIXCAM_MODE_IDLE                0U
#define MAIXCAM_MODE_CENTER              1U
#define MAIXCAM_MODE_CIRCLE              2U

extern uint8_t maix_rx_byte;

extern volatile uint8_t maixcam_vision_flags;
extern volatile float maixcam_error_x_px;
extern volatile float maixcam_error_y_px;
extern volatile uint8_t maixcam_vision_control_seq;
extern volatile uint8_t maixcam_new_frame;
extern volatile uint8_t maixcam_vision_seq;
extern volatile uint32_t maixcam_frame_count;
extern volatile uint32_t maixcam_last_rx_tick;

extern volatile uint8_t maixcam_imu_flags;
extern volatile float maixcam_gyro_x_dps;
extern volatile float maixcam_gyro_y_dps;
extern volatile float maixcam_gyro_z_dps;
extern volatile uint8_t maixcam_imu_seq;
extern volatile uint32_t maixcam_imu_frame_count;
extern volatile uint32_t maixcam_last_imu_tick;

extern volatile uint32_t maixcam_crc_error_count;
extern volatile uint32_t maixcam_length_error_count;
extern volatile uint32_t maixcam_rx_overflow_count;
extern volatile uint32_t maixcam_format_error_count;
extern volatile uint8_t maixcam_laser_enabled;

void maixcam_Init(UART_HandleTypeDef *huart);
void maixcam_RxCpltCallback(UART_HandleTypeDef *huart);
void maixcam_RxErrorCallback(UART_HandleTypeDef *huart);
void maixcam_Update(void);

void maixcam_SendControlFrame(uint8_t mode, uint16_t circle_progress,
                              uint8_t laser_enabled);
void MaixCam_SetLaserEnabled(uint8_t enabled);
uint8_t MaixCam_GetLaserEnabled(void);
void MaixCam_LaserOn(void);
void MaixCam_LaserOff(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIXCAM_H */
