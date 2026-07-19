#include "maixcam.h"

#include <math.h>
#include <string.h>

// 反向控制帧仅用于相机模式/激光状态同步，100 ms 足够用于本轮稳定性验证。
#define MAIXCAM_CONTROL_PERIOD_MS 100U
#define MAIXCAM_CONTROL_TX_ENABLE 1U

typedef enum
{
    MAIXCAM_PARSE_SOF0 = 0U,
    MAIXCAM_PARSE_SOF1,
    MAIXCAM_PARSE_BODY
} MaixCamParseState;

static UART_HandleTypeDef *maixcam_huart = NULL;
static volatile uint8_t maixcam_rx_ring[MAIXCAM_RX_RING_SIZE];
static volatile uint8_t maixcam_rx_write = 0U;
static volatile uint8_t maixcam_rx_read = 0U;
static MaixCamParseState maixcam_parse_state = MAIXCAM_PARSE_SOF0;
static uint8_t maixcam_frame[MAIXCAM_MAX_FRAME_SIZE];
static uint8_t maixcam_frame_index = 0U;
static uint8_t maixcam_expected_size = 0U;
#if MAIXCAM_CONTROL_TX_ENABLE
static uint8_t maixcam_control_seq = 0U;
static uint32_t maixcam_last_control_tick = 0U;
#endif

uint8_t maix_rx_byte = 0U;

volatile uint8_t maixcam_vision_flags = 0U;
volatile float maixcam_error_x_px = 0.0f;
volatile float maixcam_error_y_px = 0.0f;
volatile uint8_t maixcam_vision_control_seq = 0U;
volatile uint8_t maixcam_new_frame = 0U;
volatile uint8_t maixcam_vision_seq = 0U;
volatile uint32_t maixcam_frame_count = 0U;
volatile uint32_t maixcam_last_rx_tick = 0U;

volatile uint8_t maixcam_imu_flags = 0U;
volatile float maixcam_gyro_x_dps = 0.0f;
volatile float maixcam_gyro_y_dps = 0.0f;
volatile float maixcam_gyro_z_dps = 0.0f;
volatile uint8_t maixcam_imu_seq = 0U;
volatile uint32_t maixcam_imu_frame_count = 0U;
volatile uint32_t maixcam_last_imu_tick = 0U;

volatile uint32_t maixcam_crc_error_count = 0U;
volatile uint32_t maixcam_length_error_count = 0U;
volatile uint32_t maixcam_rx_overflow_count = 0U;
volatile uint32_t maixcam_format_error_count = 0U;
volatile uint8_t maixcam_laser_enabled = 0U;

static uint16_t maixcam_crc16_ccitt(const uint8_t *data, uint8_t length)
{
    uint16_t crc = 0xFFFFU;
    uint8_t index;
    uint8_t bit;

    for (index = 0U; index < length; index++)
    {
        crc ^= (uint16_t)data[index] << 8;
        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x8000U) != 0U)
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            else
                crc <<= 1;
        }
    }
    return crc;
}

static float maixcam_get_float_be(const uint8_t *data)
{
    uint32_t raw = ((uint32_t)data[0] << 24) |
                   ((uint32_t)data[1] << 16) |
                   ((uint32_t)data[2] << 8) |
                   (uint32_t)data[3];
    float value;

    memcpy(&value, &raw, sizeof(value));
    return value;
}

static void maixcam_reset_parser(void)
{
    maixcam_parse_state = MAIXCAM_PARSE_SOF0;
    maixcam_frame_index = 0U;
    maixcam_expected_size = 0U;
}

static void maixcam_save_vision_frame(void)
{
    const uint8_t *payload = &maixcam_frame[7];
    float error_x = maixcam_get_float_be(&payload[0]);
    float error_y = maixcam_get_float_be(&payload[4]);

    if ((!isfinite(error_x)) || (!isfinite(error_y)))
    {
        maixcam_format_error_count++;
        return;
    }

    maixcam_vision_flags = maixcam_frame[4];
    maixcam_vision_seq = maixcam_frame[5];
    maixcam_error_x_px = error_x;
    maixcam_error_y_px = error_y;
    maixcam_vision_control_seq = payload[8];
    maixcam_last_rx_tick = HAL_GetTick();
    maixcam_frame_count++;
    maixcam_new_frame = 1U;
}

static void maixcam_save_imu_frame(void)
{
    const uint8_t *payload = &maixcam_frame[7];
    float gyro_x = maixcam_get_float_be(&payload[0]);
    float gyro_y = maixcam_get_float_be(&payload[4]);
    float gyro_z = maixcam_get_float_be(&payload[8]);

    if ((!isfinite(gyro_x)) || (!isfinite(gyro_y)) || (!isfinite(gyro_z)))
    {
        maixcam_format_error_count++;
        return;
    }

    maixcam_imu_flags = maixcam_frame[4];
    maixcam_imu_seq = maixcam_frame[5];
    maixcam_gyro_x_dps = gyro_x;
    maixcam_gyro_y_dps = gyro_y;
    maixcam_gyro_z_dps = gyro_z;
    maixcam_last_imu_tick = HAL_GetTick();
    maixcam_imu_frame_count++;
}

static void maixcam_process_frame(void)
{
    uint8_t type = maixcam_frame[3];
    uint8_t payload_size = maixcam_frame[6];
    uint16_t received_crc = ((uint16_t)maixcam_frame[7U + payload_size] << 8) |
                            maixcam_frame[8U + payload_size];
    uint16_t calculated_crc = maixcam_crc16_ccitt(&maixcam_frame[2],
                                                   (uint8_t)(5U + payload_size));

    if (received_crc != calculated_crc)
    {
        maixcam_crc_error_count++;
        return;
    }

    if ((type == MAIXCAM_TYPE_VISION) && (payload_size == MAIXCAM_VISION_PAYLOAD_SIZE))
    {
        maixcam_save_vision_frame();
    }
    else if ((type == MAIXCAM_TYPE_IMU) && (payload_size == MAIXCAM_IMU_PAYLOAD_SIZE))
    {
        maixcam_save_imu_frame();
    }
    else
    {
        maixcam_length_error_count++;
    }
}

static void maixcam_parse_byte(uint8_t data)
{
    if (maixcam_parse_state == MAIXCAM_PARSE_SOF0)
    {
        if (data == MAIXCAM_SOF0)
        {
            maixcam_frame[0] = data;
            maixcam_parse_state = MAIXCAM_PARSE_SOF1;
        }
        return;
    }

    if (maixcam_parse_state == MAIXCAM_PARSE_SOF1)
    {
        if (data == MAIXCAM_SOF1)
        {
            maixcam_frame[1] = data;
            maixcam_frame_index = 2U;
            maixcam_parse_state = MAIXCAM_PARSE_BODY;
        }
        else if (data != MAIXCAM_SOF0)
        {
            maixcam_reset_parser();
        }
        return;
    }

    if (maixcam_frame_index >= MAIXCAM_MAX_FRAME_SIZE)
    {
        maixcam_length_error_count++;
        maixcam_reset_parser();
        return;
    }

    maixcam_frame[maixcam_frame_index++] = data;
    if (maixcam_frame_index == 7U)
    {
        if ((maixcam_frame[2] != MAIXCAM_PROTOCOL_VERSION) ||
            (maixcam_frame[6] > MAIXCAM_MAX_PAYLOAD_SIZE))
        {
            maixcam_length_error_count++;
            maixcam_reset_parser();
            return;
        }
        maixcam_expected_size = (uint8_t)(MAIXCAM_FRAME_OVERHEAD + maixcam_frame[6]);
    }

    if ((maixcam_expected_size != 0U) && (maixcam_frame_index == maixcam_expected_size))
    {
        maixcam_process_frame();
        maixcam_reset_parser();
    }
}

static void maixcam_send_frame(uint8_t type, uint8_t flags,
                                const uint8_t *payload, uint8_t payload_size)
{
#if MAIXCAM_CONTROL_TX_ENABLE
    uint8_t frame[MAIXCAM_MAX_FRAME_SIZE];
    uint16_t crc;
    uint8_t frame_size;

    if ((maixcam_huart == NULL) || (payload_size > MAIXCAM_MAX_PAYLOAD_SIZE)) return;

    frame[0] = MAIXCAM_SOF0;
    frame[1] = MAIXCAM_SOF1;
    frame[2] = MAIXCAM_PROTOCOL_VERSION;
    frame[3] = type;
    frame[4] = flags;
    frame[5] = maixcam_control_seq++;
    frame[6] = payload_size;
    if ((payload != NULL) && (payload_size > 0U))
        memcpy(&frame[7], payload, payload_size);

    crc = maixcam_crc16_ccitt(&frame[2], (uint8_t)(5U + payload_size));
    frame[7U + payload_size] = (uint8_t)(crc >> 8);
    frame[8U + payload_size] = (uint8_t)crc;
    frame_size = (uint8_t)(MAIXCAM_FRAME_OVERHEAD + payload_size);
    (void)HAL_UART_Transmit(maixcam_huart, frame, frame_size, 10U);
#else
    (void)type;
    (void)flags;
    (void)payload;
    (void)payload_size;
#endif
}

void maixcam_Init(UART_HandleTypeDef *huart)
{
    maixcam_huart = huart;
    maixcam_rx_read = 0U;
    maixcam_rx_write = 0U;
    maixcam_reset_parser();

    maixcam_vision_flags = 0U;
    maixcam_error_x_px = 0.0f;
    maixcam_error_y_px = 0.0f;
    maixcam_vision_control_seq = 0U;
    maixcam_new_frame = 0U;
    maixcam_vision_seq = 0U;
    maixcam_frame_count = 0U;
    maixcam_last_rx_tick = 0U;
    maixcam_imu_flags = 0U;
    maixcam_gyro_x_dps = 0.0f;
    maixcam_gyro_y_dps = 0.0f;
    maixcam_gyro_z_dps = 0.0f;
    maixcam_imu_seq = 0U;
    maixcam_imu_frame_count = 0U;
    maixcam_last_imu_tick = 0U;
    maixcam_crc_error_count = 0U;
    maixcam_length_error_count = 0U;
    maixcam_rx_overflow_count = 0U;
    maixcam_format_error_count = 0U;
    maixcam_laser_enabled = 0U;
#if MAIXCAM_CONTROL_TX_ENABLE
    maixcam_control_seq = 0U;
    maixcam_last_control_tick = 0U;
#endif

    if (maixcam_huart != NULL)
        (void)HAL_UART_Receive_IT(maixcam_huart, &maix_rx_byte, 1U);
}

void maixcam_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t next_write;

    if ((maixcam_huart == NULL) || (huart->Instance != maixcam_huart->Instance)) return;

    next_write = (uint8_t)(maixcam_rx_write + 1U);
    if (next_write == maixcam_rx_read)
    {
        maixcam_rx_overflow_count++;
    }
    else
    {
        maixcam_rx_ring[maixcam_rx_write] = maix_rx_byte;
        maixcam_rx_write = next_write;
    }
    (void)HAL_UART_Receive_IT(maixcam_huart, &maix_rx_byte, 1U);
}

void maixcam_RxErrorCallback(UART_HandleTypeDef *huart)
{
    if ((maixcam_huart != NULL) && (huart->Instance == maixcam_huart->Instance))
    {
        maixcam_rx_read = maixcam_rx_write;
        (void)HAL_UART_Receive_IT(maixcam_huart, &maix_rx_byte, 1U);
    }
}

void maixcam_SendControlFrame(uint8_t mode, uint16_t circle_progress,
                              uint8_t laser_enabled)
{
    uint8_t payload[MAIXCAM_CONTROL_PAYLOAD_SIZE];
    uint8_t flags = (laser_enabled != 0U) ? MAIXCAM_CONTROL_FLAG_LASER : 0U;

    maixcam_laser_enabled = (laser_enabled != 0U) ? 1U : 0U;
    payload[0] = mode;
    payload[1] = (uint8_t)(circle_progress >> 8);
    payload[2] = (uint8_t)circle_progress;
    maixcam_send_frame(MAIXCAM_TYPE_CONTROL, flags, payload, sizeof(payload));
}

void MaixCam_SetLaserEnabled(uint8_t enabled)
{
    maixcam_SendControlFrame(MAIXCAM_MODE_IDLE, 0U, enabled);
#if MAIXCAM_CONTROL_TX_ENABLE
    maixcam_last_control_tick = HAL_GetTick();
#endif
}

uint8_t MaixCam_GetLaserEnabled(void)
{
    return maixcam_laser_enabled;
}

void MaixCam_LaserOn(void)
{
    MaixCam_SetLaserEnabled(1U);
}

void MaixCam_LaserOff(void)
{
    MaixCam_SetLaserEnabled(0U);
}

void maixcam_Update(void)
{
    uint8_t data;
#if MAIXCAM_CONTROL_TX_ENABLE
    uint32_t now;
#endif

    while (maixcam_rx_read != maixcam_rx_write)
    {
        data = maixcam_rx_ring[maixcam_rx_read];
        maixcam_rx_read++;
        maixcam_parse_byte(data);
    }

#if MAIXCAM_CONTROL_TX_ENABLE
    now = HAL_GetTick();
    if ((now - maixcam_last_control_tick) >= MAIXCAM_CONTROL_PERIOD_MS)
    {
        maixcam_SendControlFrame(MAIXCAM_MODE_IDLE, 0U, maixcam_laser_enabled);
        maixcam_last_control_tick = now;
    }
#endif
}
