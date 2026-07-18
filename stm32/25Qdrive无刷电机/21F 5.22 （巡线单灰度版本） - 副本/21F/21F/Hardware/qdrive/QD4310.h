/**
 * @file        QD4310.h
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

#ifndef QD4310_H
#define QD4310_H


#include "headfile.h"

// 命令枚举
typedef enum
{
    QD4310_CMD_NOP = 0x00,        // NOP(获取反馈报文)
    QD4310_CMD_ENABLE = 0x01,     // 使能
    QD4310_CMD_DISABLE = 0x02,    // 失能
    QD4310_CMD_CURRENT = 0x03,    // 电流（力矩）控制
    QD4310_CMD_SPEED = 0x04,      // 速度控制
    QD4310_CMD_ANGLE = 0x05,      // 角度控制
    QD4310_CMD_LOW_SPEED = 0x06,  // 低速控制
    QD4310_CMD_STEP_ANGLE = 0x07, // 角度步进控制

    QD4310_CMD_REBOOT = 0xFF,       // 重启设备
    QD4310_CMD_SET_ZERO_POS = 0xFE, // 设置零点
    QD4310_CMD_CLEAR_ERROR = 0xFB,  // 清除错误
} QD4310_Command_t;

// QD4310电机结构体
typedef struct {
    bool enabled;
    uint8_t id;
    float speed;   // 转速，单位rpm
    float angle;   // 角度，单位弧度
    float current; // 电流，单位A
    UART_HandleTypeDef *huart;
    uint32_t command_count;
    uint32_t error_count;
    uint8_t consecutive_failures;
    uint32_t last_command_duration_ms;
} QD4310_t;

// 函数声明
/**
 * @brief 使能电机
 * @param motor 电机结构体指针
 */
bool QD4310_Enable(QD4310_t *motor);
/**
 * @brief 失能电机
 * @param motor 电机结构体指针
 */
bool QD4310_Disable(QD4310_t *motor);
/**
 * @brief 重启电机
 * @param motor 电机结构体指针
 */
bool QD4310_Reboot(QD4310_t *motor);
/**
 * @brief 设置电机零点
 * @param motor 电机结构体指针
 */
bool QD4310_SetZeroPos(QD4310_t *motor);
/**
 * @brief 更新电机状态
 * @param motor 电机结构体指针
 * @param feedback 来自电机的反馈数据数组
 */
void QD4310_Update(QD4310_t *motor, const uint8_t feedback[8]);
/**
 * @brief 设置电机角度
 * @param motor 电机结构体指针
 * @param angle 设置的角度,[0,2pi]
 */
bool QD4310_SetAngle(QD4310_t *motor, float angle);
/**
 * @brief 设置电机步进角度
 * @param motor 电机结构体指针
 * @param step_angle 设置的角度,[-2pi,2pi]
 */
bool QD4310_SetStepAngle(QD4310_t *motor, float step_angle);
/**
 * @brief 设置电机转速
 * @param motor 电机结构体指针
 * @param speed 设置的转速,[-1000,1000]
 */
bool QD4310_SetSpeed(QD4310_t *motor, float speed);
/**
 * @brief 设置电机转速
 * @param motor 电机结构体指针
 * @param speed 设置的转速,[-1000,1000]
 */
bool QD4310_SetLowSpeed(QD4310_t *motor, float speed);
/**
 * @brief 设置电机电流
 * @param motor 电机结构体指针
 * @param current 设置的Q轴电流，单位A，协议范围[-10,10]
 */
bool QD4310_SetCurrent(QD4310_t *motor, float current);

bool QD4310_SendCommand(QD4310_t *motor, QD4310_Command_t cmd, int16_t value);

// 数学常数定义
#define QD4310_PI (3.14159265358979323846f)
#define QD4310_TWO_PI (2.0f * QD4310_PI)

// 限制值宏定义
#define QD4310_MAX_SPEED (1000.0f)
#define QD4310_MIN_SPEED (-1000.0f)
#define QD4310_MAX_CURRENT (10.0f)
#define QD4310_MIN_CURRENT (-10.0f)
#define QD4310_MAX_STEPANGLE (QD4310_TWO_PI)
#define QD4310_MIN_STEPANGLE (-QD4310_TWO_PI)


#endif //QD4310_H
