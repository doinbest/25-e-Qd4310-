/**
 * @file        QD4310.c
 * @brief       基于HAL库的QD4310电机UART控制库
 * @details
 * @author      Liu-Curiousity (2675794963@qq.com)
 * @date        2026-6-15
 * @version     V1.2.0
 * @note
 * @warning
 * @par         历史版本:
 *		        V1.1.0创建于2026-1-25
 *		        V1.2.0创建于2026-6-15, 添加零点设置和重启设备
 * @copyright   (c) 2026 QDrive
 */

#include "QD4310.h"

uint8_t CRC8(const uint8_t *data, uint32_t len, uint8_t polynomial, uint8_t init,
             uint8_t xor_out, bool input_invert, bool output_invert);

static uint8_t TxBuffer[5] = {0};

#define QD4310_UART_TX_TIMEOUT_MS 5U

// 限制函数，用于替代C++的std::clamp
static float QD4310_Clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// 发送命令到电机
bool QD4310_SendCommand(QD4310_t *motor, QD4310_Command_t cmd, int16_t value) {
    uint32_t start_tick;
    bool ok = false;
    if ((motor == NULL) || (motor->huart == NULL)) return false;
    start_tick = HAL_GetTick();
    TxBuffer[0] = motor->id;
    TxBuffer[1] = (uint8_t)cmd;
    // 将int16_t值拆分为两个字节
    TxBuffer[2] = (uint8_t)(value & 0xFF);
    TxBuffer[3] = (uint8_t)((value >> 8) & 0xFF);
    TxBuffer[4] = CRC8(TxBuffer, 4, 0x07, 0x00, 0x00, false, false);

    /* One-way control: UART TX success is the only command success criterion. */
    ok = (HAL_UART_Transmit(motor->huart, TxBuffer, sizeof(TxBuffer),
                            QD4310_UART_TX_TIMEOUT_MS) == HAL_OK);
    motor->command_count++;
    motor->last_command_duration_ms = HAL_GetTick() - start_tick;
    if (ok) motor->consecutive_failures = 0U;
    else
    {
        motor->error_count++;
        if (motor->consecutive_failures < 255U) motor->consecutive_failures++;
    }
    return ok;
}


// 更新电机状态
void QD4310_Update(QD4310_t *motor, const uint8_t feedback[8]) {
    motor->enabled = feedback[0] & 0x01;

    // 重构int16_t值从字节数组
    int16_t current_raw = (int16_t)((feedback[3] << 8) | feedback[2]);
    motor->current = (float)current_raw * 10.0f / INT16_MAX;

    int16_t speed_raw = (int16_t)((feedback[5] << 8) | feedback[4]);
    motor->speed = (float)speed_raw * 1000.0f / 32767.0f;

    uint16_t angle_raw = (uint16_t)((feedback[7] << 8) | feedback[6]);
    motor->angle = (float)angle_raw * QD4310_TWO_PI / UINT16_MAX;
}


// 使能电机
bool QD4310_Enable(QD4310_t *motor) {
    return QD4310_SendCommand(motor, QD4310_CMD_ENABLE, 0x0000);
}

// 失能电机
bool QD4310_Disable(QD4310_t *motor) {
    return QD4310_SendCommand(motor, QD4310_CMD_DISABLE, 0x0000);
}

// 重启电机
bool QD4310_Reboot(QD4310_t *motor) {
    return QD4310_SendCommand(motor, QD4310_CMD_REBOOT, 0x0000);
}

// 设置电机零点
bool QD4310_SetZeroPos(QD4310_t *motor) {
    return QD4310_SendCommand(motor, QD4310_CMD_SET_ZERO_POS, 0x0000);
}

// 设置电机角度
bool QD4310_SetAngle(QD4310_t *motor, float angle) {
    angle = QD4310_Clamp(angle, 0.0f, QD4310_TWO_PI);
    int16_t angle_value = (int16_t)(angle / QD4310_TWO_PI * UINT16_MAX);
    return QD4310_SendCommand(motor, QD4310_CMD_ANGLE, angle_value);
}

// 设置电机步进角度
bool QD4310_SetStepAngle(QD4310_t *motor, float step_angle) {
    step_angle = QD4310_Clamp(step_angle, QD4310_MIN_STEPANGLE, QD4310_MAX_STEPANGLE);
    int16_t step_angle_value = (int16_t)(step_angle / QD4310_MAX_STEPANGLE * INT16_MAX);
    return QD4310_SendCommand(motor, QD4310_CMD_STEP_ANGLE, step_angle_value);
}

// 设置电机转速
bool QD4310_SetSpeed(QD4310_t *motor, float speed) {
    speed = QD4310_Clamp(speed, QD4310_MIN_SPEED, QD4310_MAX_SPEED);
    int16_t speed_value = (int16_t)(speed / QD4310_MAX_SPEED * INT16_MAX);
    return QD4310_SendCommand(motor, QD4310_CMD_SPEED, speed_value);
}

// 设置电机低速
bool QD4310_SetLowSpeed(QD4310_t *motor, float speed) {
    speed = QD4310_Clamp(speed, QD4310_MIN_SPEED, QD4310_MAX_SPEED);
    int16_t speed_value = (int16_t)(speed / QD4310_MAX_SPEED * INT16_MAX);
    return QD4310_SendCommand(motor, QD4310_CMD_LOW_SPEED, speed_value);
}

// 设置电机电流
bool QD4310_SetCurrent(QD4310_t *motor, float current) {
    current = QD4310_Clamp(current, QD4310_MIN_CURRENT, QD4310_MAX_CURRENT);
    int16_t current_value = (int16_t)(current / QD4310_MAX_CURRENT * INT16_MAX);
    return QD4310_SendCommand(motor, QD4310_CMD_CURRENT, current_value);
}


/**
 * @brief 反转字节(按bit反转)
 * */
static uint8_t ReverseBits(uint8_t data) {
    data = ((data & 0x55) << 1) | ((data & 0xAA) >> 1);
    data = ((data & 0x33) << 2) | ((data & 0xCC) >> 2);
    data = ((data & 0x0F) << 4) | ((data & 0xF0) >> 4);
    return data;
}

uint8_t CRC8(const uint8_t *data, uint32_t len, uint8_t polynomial, uint8_t init,
             uint8_t xor_out, bool input_invert, bool output_invert) {
    /**1.校验参数**/
    assert_param(data != NULL);
    assert_param(len > 0);
    /**2.变量定义**/
    uint8_t crc = init;
    /**3.计算**/
    do {
        crc ^= input_invert ? ReverseBits(*(data++)) : *(data++);
        for (uint8_t i = 0; i < 8; ++i) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ polynomial;
            } else {
                crc <<= 1;
            }
        }
    } while (--len);
    return output_invert ? ReverseBits(crc ^ xor_out) : (crc ^ xor_out);
}
