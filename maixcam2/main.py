from maix import app, camera, display, image, nn, time, pinmap, pwm, uart
from maix.ext_dev import imu
import cv2
import numpy as np
import os
import math
import struct
import threading


# ------------------------- YOLO与靶标几何参数 -------------------------
MODEL_PATH = "best.mud"
CONF_TH = 0.45
IOU_TH = 0.45
CROP_PADDING = 12

# 下方轮廓对应贴有18 mm黑边后的A4纸内部白色区域。
# 该区域实际物理宽高比为261:174=1.5，而不是113:80。
INNER_W_MM = 261.0
INNER_H_MM = 174.0
CENTER_MM = (INNER_W_MM / 2.0, INNER_H_MM / 2.0)
CIRCLE_RADIUS_MM = 60.0
CIRCLE_START_DEG = -90.0
CIRCLE_SEGMENTS = 72

# ---------------------- 激光标定后的虚拟画面中心 ----------------------
# 8组手动标定结果表明，当激光实际对准靶心时，检测到的靶心坐标约为(308,200)。
# 按照V1.0.py相同的处理思路，直接把该坐标作为瞄准中心/画面中心，
# 不再把它平移到(320,240)。
AIM_CENTER_X = 310
AIM_CENTER_Y = 202
AIM_CENTER = (AIM_CENTER_X, AIM_CENTER_Y)

PLANE_CORNERS = np.float32(((0, 0), (INNER_W_MM, 0),
                            (INNER_W_MM, INNER_H_MM), (0, INNER_H_MM)))

DRAW_DEBUG = True
LOCK_DEAD_ZONE_PX = 5
LOCK_HOLD_MS = 300
VALID_CONFIRM_FRAMES = 3
TARGET_HOLD_MS = 120


# ---------------------------- 通信协议 V1 ----------------------------
UART_DEVICE = "/dev/ttyS4"
UART_BAUD_RATE = 115200
IMU_SEND_PERIOD_MS = 5
VISION_SEND_PERIOD_MS = 10
IMU_CALIBRATION_MS = 10000
IMU_CALIBRATION_ID = "gimbal_lsm6dsowtr_v1"

FRAME_SOF = b"\x55\xAA"
FRAME_VERSION = 0x01
FRAME_TYPE_VISION = 0x01
FRAME_TYPE_IMU = 0x02
FRAME_TYPE_CONTROL = 0x81

VISION_FLAG_TARGET = 0x01
VISION_FLAG_MAPPING = 0x02
VISION_FLAG_LOCKED = 0x04
VISION_FLAG_CONTROL_FRESH = 0x08
IMU_FLAG_VALID = 0x01
IMU_FLAG_CALIBRATED = 0x02
CONTROL_FLAG_LASER = 0x01


# ---------------------------- 激光PWM控制 ----------------------------
# 激光只能在STM32控制帧授权后打开；上电默认关闭。
laser_authorized = False

# B3通过PWM控制外部激光驱动，程序启动时默认关闭激光。
# 如果你的激光驱动为低电平有效，请交换LASER_ON_DUTY和LASER_OFF_DUTY。
LASER_PWM_PIN = "B3"
LASER_PWM_FREQ = 1000
LASER_ON_DUTY = 100
LASER_OFF_DUTY = 0


def init_laser_pwm():
    """自动查找B3对应的PWM通道，并以关闭激光的状态启动。"""
    pwm_id = None
    for function in pinmap.get_pin_functions(LASER_PWM_PIN):
        if function.startswith("PWM"):
            pwm_id = int(function[3:])
            pinmap.set_pin_function(LASER_PWM_PIN, function)
            break
    if pwm_id is None:
        raise RuntimeError("{} has no PWM function".format(LASER_PWM_PIN))
    return pwm.PWM(pwm_id, freq=LASER_PWM_FREQ,
                   duty=LASER_OFF_DUTY, enable=True)


laser_pwm = init_laser_pwm()
laser_output_state = False


def set_laser(enabled):
    """仅在激光状态发生变化时更新PWM，减少主循环中的无效调用。"""
    global laser_output_state
    enabled = bool(enabled)
    if enabled == laser_output_state:
        return
    laser_pwm.duty(LASER_ON_DUTY if enabled else LASER_OFF_DUTY)
    laser_output_state = enabled


def crc16_ccitt_false(data):
    """CRC-16/CCITT-FALSE: poly=0x1021, init=0xFFFF, no reflection."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


uart_dev = None
uart_tx_lock = threading.Lock()
camera_tx_seq = 0
control_rx_buffer = bytearray()
applied_control_seq = 0
control_fresh_since_visual = False
control_frame_received = False
control_mode = 0
circle_progress = 0
latest_gyro_dps = (0.0, 0.0, 0.0)
imu_data_valid = False


def send_frame(frame_type, flags, payload):
    """Build and send one V1 frame; the lock prevents cross-frame writes."""
    global camera_tx_seq
    with uart_tx_lock:
        body = struct.pack(">BBBBB", FRAME_VERSION, frame_type, flags,
                           camera_tx_seq, len(payload)) + payload
        camera_tx_seq = (camera_tx_seq + 1) & 0xFF
        packet = FRAME_SOF + body + struct.pack(">H", crc16_ccitt_false(body))
        uart_dev.write(packet)


def send_vision_frame(target_valid, mapping_valid, locked, error_x_px, error_y_px):
    global control_fresh_since_visual
    flags = 0
    if target_valid:
        flags |= VISION_FLAG_TARGET
    if mapping_valid:
        flags |= VISION_FLAG_MAPPING
    if locked:
        flags |= VISION_FLAG_LOCKED
    if control_fresh_since_visual:
        flags |= VISION_FLAG_CONTROL_FRESH

    payload = struct.pack(">ffB", float(error_x_px), float(error_y_px),
                          applied_control_seq)
    send_frame(FRAME_TYPE_VISION, flags, payload)
    control_fresh_since_visual = False


def send_imu_frame(valid, calibrated, gyro_x_dps=0.0, gyro_y_dps=0.0,
                   gyro_z_dps=0.0):
    flags = 0
    if valid:
        flags |= IMU_FLAG_VALID
    if calibrated:
        flags |= IMU_FLAG_CALIBRATED
    payload = struct.pack(">fff", float(gyro_x_dps), float(gyro_y_dps),
                          float(gyro_z_dps))
    send_frame(FRAME_TYPE_IMU, flags, payload)


def apply_control_frame(flags, seq, payload):
    """Only CRC-verified STM32 control frames may change the laser output."""
    global laser_authorized, applied_control_seq, control_fresh_since_visual
    global control_frame_received
    global control_mode, circle_progress

    control_mode = payload[0]
    circle_progress = (payload[1] << 8) | payload[2]
    laser_authorized = bool(flags & CONTROL_FLAG_LASER)
    applied_control_seq = seq
    control_fresh_since_visual = True
    control_frame_received = True
    set_laser(laser_authorized)


def poll_control_uart():
    """Parse a stream, tolerating half frames, concatenated frames and noise."""
    global control_rx_buffer
    received = uart_dev.read()
    if received:
        control_rx_buffer.extend(received)

    while True:
        start = control_rx_buffer.find(FRAME_SOF)
        if start < 0:
            control_rx_buffer = control_rx_buffer[-1:] if control_rx_buffer.endswith(b"\x55") else bytearray()
            return
        if start > 0:
            del control_rx_buffer[:start]
        if len(control_rx_buffer) < 7:
            return

        version = control_rx_buffer[2]
        frame_type = control_rx_buffer[3]
        flags = control_rx_buffer[4]
        seq = control_rx_buffer[5]
        payload_len = control_rx_buffer[6]
        frame_len = 9 + payload_len
        if version != FRAME_VERSION or payload_len > 16:
            del control_rx_buffer[0]
            continue
        if len(control_rx_buffer) < frame_len:
            return

        body = control_rx_buffer[2:7 + payload_len]
        received_crc = (control_rx_buffer[7 + payload_len] << 8) | control_rx_buffer[8 + payload_len]
        payload = bytes(control_rx_buffer[7:7 + payload_len])
        del control_rx_buffer[:frame_len]
        if received_crc != crc16_ccitt_false(body):
            continue
        if frame_type == FRAME_TYPE_CONTROL and payload_len == 3:
            apply_control_frame(flags, seq, payload)


def init_uart_and_imu():
    """Map A21/A22 once, then prepare calibrated board gyro output in dps."""
    global uart_dev
    pinmap.set_pin_function("A21", "UART4_TX")
    pinmap.set_pin_function("A22", "UART4_RX")
    uart_dev = uart.UART(UART_DEVICE, UART_BAUD_RATE)

    sensor = imu.IMU(
        "default",
        mode=imu.Mode.GYRO_ONLY,
        gyro_scale=imu.GyroScale.GYRO_SCALE_1000DPS,
        gyro_odr=imu.GyroOdr.GYRO_ODR_416,
    )
    if sensor.calib_gyro_exists(IMU_CALIBRATION_ID):
        sensor.load_calib_gyro(IMU_CALIBRATION_ID)
    else:
        print("[IMU] Keep gimbal still: calibrating gyro for 10 seconds")
        sensor.calib_gyro(IMU_CALIBRATION_MS, interval_ms=5,
                          save_id=IMU_CALIBRATION_ID)
    return sensor


imu_task_running = True
imu_calibrated = False


def imu_send_task(sensor):
    """Independent 200 Hz task: vision inference never schedules IMU packets."""
    global latest_gyro_dps, imu_data_valid
    while imu_task_running and not app.need_exit():
        try:
            data = sensor.read_all(calib_gryo=True, radian=False)
            gyro = data.gyro
            latest_gyro_dps = (float(gyro.x), float(gyro.y), float(gyro.z))
            imu_data_valid = True
            send_imu_frame(True, imu_calibrated, *latest_gyro_dps)
        except Exception as error:
            print("[IMU] read failed:", error)
            imu_data_valid = False
            send_imu_frame(False, imu_calibrated)
        time.sleep_ms(IMU_SEND_PERIOD_MS)


# ---------------------------- 视觉处理函数 ----------------------------
def clamp_rect(x, y, w, h, image_w, image_h):
    x = max(0, int(x))
    y = max(0, int(y))
    w = min(int(w), image_w - x)
    h = min(int(h), image_h - y)
    return x, y, w, h


def sort_corners(corners):
    """按左上、右上、右下、左下的顺序返回四个角点。"""
    corners = np.asarray(corners, dtype=np.float32).reshape(4, 2)
    sums = corners.sum(axis=1)
    diffs = np.diff(corners, axis=1).reshape(-1)
    return np.float32((corners[np.argmin(sums)], corners[np.argmin(diffs)],
                       corners[np.argmax(sums)], corners[np.argmax(diffs)]))


def find_inner_paper_quad(bgr_crop):
    """只在YOLO感兴趣区域内寻找A4纸内部白色四边形。"""
    gray = cv2.cvtColor(bgr_crop, cv2.COLOR_BGR2GRAY)
    binary = cv2.adaptiveThreshold(
        gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 27, 31
    )
    height, width = binary.shape[:2]
    if width < 8 or height < 8:
        return None

    # 去除与外部背景连通的区域，仅保留纸张边缘。
    mask = np.zeros((height + 2, width + 2), np.uint8)
    cv2.floodFill(binary, mask, (2, 2), 255, loDiff=5, upDiff=5, flags=4)
    cv2.floodFill(binary, mask, (width - 3, height - 3), 255,
                  loDiff=5, upDiff=5, flags=4)
    binary = cv2.bitwise_not(binary)

    result = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = result[0] if len(result) == 2 else result[1]
    if not contours:
        return None

    contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(contour) < 100:
        return None
    quad = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
    if len(quad) != 4:
        return None

    quad = sort_corners(quad)
    top = np.linalg.norm(quad[1] - quad[0])
    bottom = np.linalg.norm(quad[2] - quad[3])
    left = np.linalg.norm(quad[3] - quad[0])
    right = np.linalg.norm(quad[2] - quad[1])
    if min(top, bottom, left, right) < 12:
        return None
    return quad


def circle_target_mm(progress):
    phase = max(0.0, min(1.0, float(progress) / 10000.0))
    angle = math.radians(CIRCLE_START_DEG) + 2.0 * math.pi * phase
    return (CENTER_MM[0] + CIRCLE_RADIUS_MM * math.cos(angle),
            CENTER_MM[1] + CIRCLE_RADIUS_MM * math.sin(angle))


def project_plane_points(h_plane_to_crop, offset, points):
    points = np.asarray(points, dtype=np.float32).reshape(-1, 1, 2)
    projected = cv2.perspectiveTransform(points, h_plane_to_crop).reshape(-1, 2)
    projected += np.float32(offset)
    return projected


def map_target(bgr_image, target_box):
    """返回检测到的印刷靶心坐标。"""
    img_h, img_w = bgr_image.shape[:2]
    x, y, w, h = clamp_rect(*target_box, img_w, img_h)
    if w < 20 or h < 20:
        return None, None, None

    quad = find_inner_paper_quad(bgr_image[y:y + h, x:x + w])
    if quad is None:
        return None, None, None

    h_plane_to_crop = cv2.getPerspectiveTransform(PLANE_CORNERS, quad)
    raw_target_centre = project_plane_points(
        h_plane_to_crop, (x, y), [CENTER_MM]
    )[0]
    quad += np.float32((x, y))
    raw_point = tuple(np.round(raw_target_centre).astype(np.int32))
    return raw_point, quad, raw_point


def box_iou(a, b):
    if a is None or b is None:
        return 0.0
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    ix1, iy1 = max(ax, bx), max(ay, by)
    ix2, iy2 = min(ax + aw, bx + bw), min(ay + ah, by + bh)
    inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
    union = aw * ah + bw * bh - inter
    return inter / union if union > 0 else 0.0


def select_target(objs, last_box):
    """优先选择与上一帧匹配的目标，避免瞬时较大误检框抢占目标。"""
    if not objs:
        return None
    return max(objs, key=lambda obj: obj.score + 0.65 * box_iou(
        (obj.x, obj.y, obj.w, obj.h), last_box
    ) + 0.00001 * obj.w * obj.h)


# ---------------------------- 初始化 ----------------------------
imu_sensor = init_uart_and_imu()
imu_calibrated = True
imu_thread = threading.Thread(target=imu_send_task, args=(imu_sensor,), daemon=True)
imu_thread.start()

model_path = MODEL_PATH if os.path.exists(MODEL_PATH) else "/root/models/best.mud"
detector = nn.YOLO11(model=model_path, dual_buff=True)
cam = camera.Camera(640, 480, detector.input_format())
disp = display.Display()
cam.skip_frames(30)

last_valid_time = 0
last_box = None
valid_frame_count = 0
last_output_center = None
last_vision_send_time = 0


# ---------------------------- 主循环 ----------------------------
try:
    while not app.need_exit():
        poll_control_uart()
        img = cam.read()
        bgr = image.image2cv(img, copy=False)
        objs = detector.detect(img, conf_th=CONF_TH, iou_th=IOU_TH)
        target = select_target(objs, last_box)

        # 使用激光标定后的虚拟画面中心，与V1.0中的瞄准点处理一致。
        frame_center = AIM_CENTER
        target_center = None
        raw_target_center = None
        quad = None
        locked = False
        holding_target = False

        if target is not None:
            box = (target.x - CROP_PADDING, target.y - CROP_PADDING,
                   target.w + 2 * CROP_PADDING, target.h + 2 * CROP_PADDING)
            raw_target_center, quad, target_center = map_target(bgr, box)
            cv2.rectangle(bgr, (target.x, target.y),
                          (target.x + target.w, target.y + target.h),
                          (0, 0, 255), 2)
            cv2.putText(bgr, "Kuang:{:.2f}".format(target.score),
                        (target.x, max(16, target.y - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (0, 0, 255), 1, cv2.LINE_AA)

        if target_center is not None:
            last_valid_time = time.ticks_ms()
            last_box = (target.x, target.y, target.w, target.h)
            valid_frame_count = min(VALID_CONFIRM_FRAMES, valid_frame_count + 1)

            # 与V1.0.py保持相同符号约定：瞄准中心减去检测中心。
            err_x = frame_center[0] - raw_target_center[0]
            err_y = frame_center[1] - raw_target_center[1]
            locked = (abs(err_x) <= LOCK_DEAD_ZONE_PX and
                      abs(err_y) <= LOCK_DEAD_ZONE_PX)

            cv2.polylines(bgr, [np.round(quad).astype(np.int32)],
                          True, (0, 255, 0), 1)
            cv2.line(bgr, frame_center, raw_target_center, (255, 255, 0), 1)
            cv2.drawMarker(bgr, raw_target_center, (0, 255, 0),
                           cv2.MARKER_CROSS, 11, 2)

            if DRAW_DEBUG:
                colour = (0, 255, 0) if locked else (0, 255, 255)
                cv2.putText(
                    bgr,
                    "CAM ERR X:{} Y:{} {}".format(
                        err_x, err_y, "LOCK" if locked else "TRACK"
                    ),
                    (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    colour, 1, cv2.LINE_AA
                )

            if valid_frame_count >= VALID_CONFIRM_FRAMES:
                last_output_center = raw_target_center
            elif DRAW_DEBUG:
                cv2.putText(
                    bgr,
                    "ACQUIRE {}/{}".format(valid_frame_count, VALID_CONFIRM_FRAMES),
                    (4, 124), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (0, 165, 255), 1, cv2.LINE_AA
                )
        else:
            elapsed_lost = time.ticks_diff(time.ticks_ms(), last_valid_time)
            if elapsed_lost <= TARGET_HOLD_MS and last_output_center is not None:
                holding_target = True
            else:
                valid_frame_count = 0

            if elapsed_lost > LOCK_HOLD_MS:
                last_box = None

            if DRAW_DEBUG:
                cv2.putText(
                    bgr,
                    "TARGET HOLD" if holding_target else "TARGET LOST",
                    (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (0, 0, 255), 1, cv2.LINE_AA
                )

        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, last_vision_send_time) >= VISION_SEND_PERIOD_MS:
            last_vision_send_time = now_ms
            target_valid = target_center is not None and valid_frame_count >= VALID_CONFIRM_FRAMES
            mapping_valid = target_center is not None
            if target_valid:
                send_vision_frame(True, mapping_valid, locked, err_x, err_y)
            else:
                # 目标丢失时明确发送无效帧，STM32不能继续使用旧坐标。
                send_vision_frame(False, mapping_valid, False, 0.0, 0.0)

        # 白色十字：激光标定后的虚拟画面中心，而不是默认图像中心(320,240)。
        cv2.drawMarker(bgr, frame_center, (255, 255, 255),
                       cv2.MARKER_CROSS, 13, 2)
        cv2.putText(
            bgr,
            "SCREEN CENTER X:{} Y:{}".format(frame_center[0], frame_center[1]),
            (4, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
            (255, 255, 255), 1, cv2.LINE_AA
        )

        if raw_target_center is not None:
            cv2.putText(
                bgr,
                "TARGET CENTER X:{} Y:{}".format(
                    raw_target_center[0], raw_target_center[1]
                ),
                (4, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                (0, 255, 0), 1, cv2.LINE_AA
            )
        else:
            cv2.putText(
                bgr, "TARGET CENTER X:--- Y:---", (4, 52),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                (0, 0, 255), 1, cv2.LINE_AA
            )

        cv2.putText(
            bgr, "FPS:{:.1f}".format(time.fps()),
            (4, bgr.shape[0] - 6), cv2.FONT_HERSHEY_SIMPLEX,
            0.45, (255, 255, 255), 1, cv2.LINE_AA
        )

        if DRAW_DEBUG:
            cv2.putText(
                bgr, "LASER {}".format("ON" if laser_authorized else "OFF"),
                (4, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                (0, 255, 0) if laser_authorized else (0, 0, 255),
                1, cv2.LINE_AA
            )
            cv2.putText(
                bgr, "GYR {} {:+.1f},{:+.1f},{:+.1f}".format(
                    "OK" if imu_data_valid else "--", *latest_gyro_dps
                ),
                (4, 88), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                (0, 255, 0) if imu_data_valid else (0, 0, 255),
                1, cv2.LINE_AA
            )
            cv2.putText(
                bgr, "CTRL {} S:{:03d} M:{} P:{:05d}".format(
                    "RX" if control_frame_received else "--",
                    applied_control_seq, control_mode, circle_progress
                ),
                (4, 106), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                (0, 255, 0) if control_frame_received else (0, 0, 255),
                1, cv2.LINE_AA
            )

        disp.show(image.cv2image(bgr, bgr=True, copy=False))
finally:
    # 无论正常退出还是运行异常，都强制关闭激光并释放PWM。
    imu_task_running = False
    set_laser(False)
    laser_pwm.disable()
