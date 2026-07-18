#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

/** @brief 启动底盘巡线任务并清零本次运行状态。 */
void control_start(void);
/** @brief 停止底盘巡线并清零底盘 PID 输出。 */
void control_stop(void);
/** @brief 复位巡线任务状态和编码器里程。 */
void control_reset(void);
/** @brief 恢复比赛默认控制状态，不修改灰度校准值。 */
void control_competition_reset(void);

/** @brief 切换巡线速度档位。 */
void control_next_speed_gear(void);
/** @brief 获取当前巡线速度档位；返回值范围为 0 至 2。 */
uint8_t control_get_speed_gear(void);
/** @brief 获取当前档位基础速度；返回单位为 mm/s。 */
float control_get_line_speed_mm_s(void);
/** @brief 更新灰度传感器结果；应在主循环中周期调用。 */
void control_line_sensor_update(void);
/** @brief 循环设置目标圈数；返回值范围为 1 至 5。 */
uint8_t control_next_lap_target(void);
/** @brief 获取设置的目标圈数。 */
uint8_t control_get_lap_target(void);
/** @brief 获取已完成圈数。 */
uint8_t control_get_lap_done(void);
/** @brief 获取已完成的转弯节点数。 */
uint8_t control_get_turn_done(void);
/** @brief 获取扣除原地转向后的巡线里程；返回单位为 mm。 */
float control_get_run_distance_mm(void);
/** @brief 获取巡线状态文本；返回只读字符串。 */
const char *control_get_line_state_text(void);

/** @brief 执行一次 PID 调试模式控制；由 TIM7 的 10 ms 节拍调用。 */
void control_debug_update_10ms(void);
/** @brief 安全停止 PID 调试输出并清零相关 PID。 */
void control_debug_stop(void);

/** @brief 启动默认云台模式；仅在姿态和视觉数据有效时使能。 */
void gimbal_start(void);
/** @brief 启动双轴云台输出。 */
void gimbal_start_both(void);
/** @brief 启动指定轴像素跟踪；@param axis 轴索引：0 为 X，1 为 Y。 */
void gimbal_start_pixel_debug(uint8_t axis);
/** @brief 启动指定轴视觉调试；@param axis 轴索引：0 为 X，1 为 Y。 */
void gimbal_start_axis(uint8_t axis);
/** @brief 停止云台输出并清零 PID 状态。 */
void gimbal_stop(void);
/** @brief 切换云台启停状态。 */
void gimbal_toggle(void);
/** @brief 切换 X 轴电机使能状态。 */
void gimbal_toggle_x_motor(void);
/** @brief 切换 Y 轴电机使能状态。 */
void gimbal_toggle_y_motor(void);
/** @brief 启动指定轴 IMU 角速度调试；@param axis 轴索引。 */
void gimbal_start_rate_debug(uint8_t axis);
/** @brief 启动指定轴电流测试；@param current_a 测试电流，单位 A。 */
void gimbal_start_current_debug(uint8_t axis, float current_a);
/** @brief 立即停止并失能两台云台电机。 */
void gimbal_disable_all(void);
/** @brief 请求切换云台模式；非阻塞，内部会等待姿态零位过渡。 */
void gimbal_request_mode(uint8_t mode);
/** @brief 请求云台安全停止；非阻塞。 */
void gimbal_request_stop(void);
/** @brief 设置角速度调试目标；@param target_rpm 机械角速度，单位 rpm。 */
void gimbal_set_rate_target(uint8_t axis, float target_rpm);
/** @brief 设置像素调试目标；@param target_px 图像坐标，单位 px。 */
void gimbal_set_pixel_target(uint8_t axis, float target_px);
/** @brief 设置按键选择的云台任务模式。 */
void gimbal_set_task_mode(uint8_t mode);
/** @brief 切换到下一云台任务模式。 */
void gimbal_next_task_mode(void);
/** @brief 获取当前选择的云台任务模式。 */
uint8_t gimbal_get_task_mode(void);
/** @brief 执行云台周期任务；应在主循环中持续调用。 */
void gimbal_task(void);
/** @brief 查询云台控制是否处于激活状态；返回 0 或 1。 */
uint8_t gimbal_is_active(void);
/** @brief 查询最新视觉目标是否在超时范围内有效；返回 0 或 1。 */
uint8_t gimbal_target_valid(void);
/** @brief 查询 IMU 角速度调试是否激活；返回 0 或 1。 */
uint8_t gimbal_rate_debug_active(void);
/** @brief 查询 X 轴电机是否已使能；返回 0 或 1。 */
uint8_t gimbal_x_motor_enabled(void);
/** @brief 查询 Y 轴电机是否已使能；返回 0 或 1。 */
uint8_t gimbal_y_motor_enabled(void);
/** @brief 获取云台状态文本；返回只读字符串。 */
const char *gimbal_get_state_text(void);
/** @brief 获取 X 轴实际角速度；返回单位为 rpm。 */
float gimbal_get_x_rate_rpm(void);
/** @brief 获取 Y 轴实际角速度；返回单位为 rpm。 */
float gimbal_get_y_rate_rpm(void);
/** @brief 获取 X 轴下发给 QD4310 的速度命令；返回单位为 rpm。 */
float gimbal_get_x_cmd_rpm(void);
/** @brief 获取 Y 轴下发给 QD4310 的速度命令；返回单位为 rpm。 */
float gimbal_get_y_cmd_rpm(void);
/** @brief 获取 X 轴 IMU 速度环目标；返回单位为 rpm。 */
float gimbal_get_x_rate_target_rpm(void);
/** @brief 获取 Y 轴 IMU 速度环目标；返回单位为 rpm。 */
float gimbal_get_y_rate_target_rpm(void);
/** @brief 获取 X 轴实际像素坐标；返回单位为 px。 */
float gimbal_get_x_pixel_px(void);
/** @brief 获取 Y 轴实际像素坐标；返回单位为 px。 */
float gimbal_get_y_pixel_px(void);
/** @brief 获取 X 轴像素目标；返回单位为 px。 */
float gimbal_get_x_pixel_target_px(void);
/** @brief 获取 Y 轴像素目标；返回单位为 px。 */
float gimbal_get_y_pixel_target_px(void);

/** @brief 巡线运行标志；0 为停止，非零为运行。 */
extern uint8_t control_running;
/** @brief 巡线基础速度调试值；单位 mm/s。 */
extern int16_t speed_target_mm_s;
/** @brief 左轮最终 PWM 调试值；范围沿用原电机接口。 */
extern int16_t left_pwm;
/** @brief 右轮最终 PWM 调试值；范围沿用原电机接口。 */
extern int16_t right_pwm;

#endif /* CONTROL_H */
