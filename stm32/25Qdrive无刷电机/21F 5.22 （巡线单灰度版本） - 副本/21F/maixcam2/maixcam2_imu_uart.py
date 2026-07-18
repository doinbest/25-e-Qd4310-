"""MaixCAM2 IMU-to-STM32 UART adapter for the 2025 E-topic gimbal.

Call publisher.send(target_x, target_y, laser_x, laser_y) from the existing
vision loop after target and laser centers have been detected.  The STM32
receiver accepts the following 18-byte binary frame:

    A5 5A 01 TxH TxL TyH TyL LxH LxL LyH LyL GxH GxL GyH GyL GzH GzL XOR

The gyro fields are signed int16 values in 0.01 deg/s.  At 100 Hz this frame
uses only about 18 kbit/s, well below a 115200-baud 8N1 UART link.
"""

from maix.ext_dev import imu
from maix import err, pinmap, uart
import time


UART_DEVICE = "/dev/ttyS4"
UART_BAUD = 115200
UART_TX_PIN = "A21"
UART_RX_PIN = "A22"

GYRO_ODR_HZ = 208
SEND_PERIOD_S = 0.010
FILTER_ALPHA = 0.20
CALIBRATION_ID = "gimbal_gyro"
FACTORY_CALIBRATION_MS = 10000
RUNTIME_BIAS_SECONDS = 3.0

FRAME_HEAD0 = 0xA5
FRAME_HEAD1 = 0x5A
FRAME_VERSION = 0x01
GYRO_SCALE = 100.0


def _clamp_int16(value):
    return max(-32768, min(32767, int(round(value))))


def _u16_be(value):
    value = max(0, min(65535, int(value)))
    return bytes((value >> 8, value & 0xFF))


def _s16_be(value):
    value = _clamp_int16(value)
    if value < 0:
        value += 65536
    return bytes((value >> 8, value & 0xFF))


def create_uart():
    err.check_raise(pinmap.set_pin_function(UART_TX_PIN, "UART4_TX"), "UART4 TX mapping failed")
    err.check_raise(pinmap.set_pin_function(UART_RX_PIN, "UART4_RX"), "UART4 RX mapping failed")
    return uart.UART(UART_DEVICE, UART_BAUD)


def create_sensor():
    return imu.IMU(
        "default",
        mode=imu.Mode.DUAL,
        acc_scale=imu.AccScale.ACC_SCALE_2G,
        acc_odr=imu.AccOdr.ACC_ODR_208,
        gyro_scale=imu.GyroScale.GYRO_SCALE_250DPS,
        gyro_odr=imu.GyroOdr.GYRO_ODR_208,
        block=True,
    )


def calibrate_and_estimate_bias(sensor):
    """Keep the camera completely still whenever this function runs."""
    if not sensor.calib_gyro_exists(CALIBRATION_ID):
        sensor.calib_gyro(FACTORY_CALIBRATION_MS, interval_ms=5, save_id=CALIBRATION_ID)
    sensor.load_calib_gyro(CALIBRATION_ID)

    samples = [[], [], []]
    end_time = time.monotonic() + RUNTIME_BIAS_SECONDS
    while time.monotonic() < end_time:
        gyro = sensor.read_all(calib_gryo=True, radian=False).gyro
        samples[0].append(float(gyro.x))
        samples[1].append(float(gyro.y))
        samples[2].append(float(gyro.z))
        time.sleep_ms(5)
    return [sum(values) / len(values) for values in samples]


class VisionImuPublisher:
    def __init__(self):
        self.serial = create_uart()
        self.sensor = create_sensor()
        self.bias = calibrate_and_estimate_bias(self.sensor)
        self.filtered = [0.0, 0.0, 0.0]
        self.filter_ready = False

    def read_gyro_dps(self):
        gyro = self.sensor.read_all(calib_gryo=True, radian=False).gyro
        corrected = [
            float(gyro.x) - self.bias[0],
            float(gyro.y) - self.bias[1],
            float(gyro.z) - self.bias[2],
        ]
        if not self.filter_ready:
            self.filtered = corrected[:]
            self.filter_ready = True
        else:
            for index in range(3):
                self.filtered[index] += FILTER_ALPHA * (corrected[index] - self.filtered[index])
        return self.filtered

    def send(self, target_x, target_y, laser_x, laser_y):
        gyro_x, gyro_y, gyro_z = self.read_gyro_dps()
        frame = bytearray((FRAME_HEAD0, FRAME_HEAD1, FRAME_VERSION))
        frame += _u16_be(target_x) + _u16_be(target_y)
        frame += _u16_be(laser_x) + _u16_be(laser_y)
        frame += _s16_be(gyro_x * GYRO_SCALE)
        frame += _s16_be(gyro_y * GYRO_SCALE)
        frame += _s16_be(gyro_z * GYRO_SCALE)
        checksum = 0
        for byte in frame:
            checksum ^= byte
        frame.append(checksum)
        self.serial.write(frame)


def example_loop(get_centers):
    """Example only. get_centers() must return Tx, Ty, Lx, Ly every cycle."""
    publisher = VisionImuPublisher()
    next_time = time.monotonic()
    while True:
        target_x, target_y, laser_x, laser_y = get_centers()
        publisher.send(target_x, target_y, laser_x, laser_y)
        next_time += SEND_PERIOD_S
        remain = next_time - time.monotonic()
        if remain > 0:
            time.sleep(remain)
        else:
            next_time = time.monotonic()
