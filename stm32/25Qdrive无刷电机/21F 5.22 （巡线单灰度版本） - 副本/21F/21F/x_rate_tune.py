#!/usr/bin/env python3
"""X 轴角速度环安全阶跃调参工具。

使用前：
1. 在板端用 K1 切到 ``X Rate PID``；电机处于已使能、未启动也可以。
2. 关闭野火 PID 调试助手（同一 COM 口不能被两个程序同时打开）。
3. 确认机械活动范围安全，并从较小目标速度开始。

该脚本使用工程中 ``protocol.c`` 的 SZHY 协议，依次发送：PID 参数、
目标角速度、启动命令。每个 P 测试结束均会先发送 0 °/s，再发送停止命令。
接收到的 CH1 实际值会保存为 CSV，方便用 Excel 或 Python 画曲线。

依赖：pip install pyserial
示例：
    python x_rate_tune.py --port COM12
    python x_rate_tune.py --port COM12 --p-list 0.0005,0.0008,0.0012 --target 20
"""

from __future__ import annotations

import argparse
import csv
import struct
import sys
import time
from pathlib import Path

try:
    import serial
except ImportError as exc:
    raise SystemExit("缺少 pyserial，请先执行：pip install pyserial") from exc


FRAME_HEAD = b"SZHY"          # 0x59485A53 在 STM32 小端内存中的字节序
CHANNEL_1 = 0x01
SET_PID = 0x10
SET_TARGET = 0x11
START = 0x12
STOP = 0x13
SEND_FACT = 0x02


def make_frame(command: int, payload: bytes = b"", channel: int = CHANNEL_1) -> bytes:
    """Build one Wildfire PID-assistant frame.

    Firmware parses received numeric payloads as big endian. The frame length is
    the complete frame size (header + payload + checksum).
    """
    length = 11 + len(payload)
    header = struct.pack("<4sBIB", FRAME_HEAD, channel, length, command)
    checksum = sum(header + payload) & 0xFF
    return header + payload + bytes((checksum,))


def send_pid(port: serial.Serial, kp: float, ki: float, kd: float) -> None:
    port.write(make_frame(SET_PID, struct.pack(">fff", kp, ki, kd)))


def send_target(port: serial.Serial, target_dps: int) -> None:
    port.write(make_frame(SET_TARGET, struct.pack(">i", target_dps)))


def send_command(port: serial.Serial, command: int) -> None:
    port.write(make_frame(command))


def safe_stop(port: serial.Serial) -> None:
    """Ask the active rate loop to decelerate, then disable it."""
    try:
        send_target(port, 0)
        port.flush()
        time.sleep(0.35)
        send_command(port, STOP)
        port.flush()
    except serial.SerialException:
        pass


def read_frames(port: serial.Serial, rx: bytearray):
    """Yield (command, channel, payload) frames while retaining incomplete data."""
    rx.extend(port.read(port.in_waiting or 1))
    while True:
        start = rx.find(FRAME_HEAD)
        if start < 0:
            del rx[:-3]  # retain possible partial header
            return
        if start:
            del rx[:start]
        if len(rx) < 10:
            return
        length = struct.unpack_from("<I", rx, 5)[0]
        if length < 11 or length > 128:
            del rx[0]
            continue
        if len(rx) < length:
            return
        frame = bytes(rx[:length])
        del rx[:length]
        if (sum(frame[:-1]) & 0xFF) != frame[-1]:
            continue
        yield frame[9], frame[4], frame[10:-1]


def collect(port: serial.Serial, seconds: float, writer: csv.writer, label: str) -> None:
    deadline = time.monotonic() + seconds
    rx = bytearray()
    while time.monotonic() < deadline:
        for command, channel, payload in read_frames(port, rx):
            if command == SEND_FACT and channel == CHANNEL_1 and len(payload) >= 4:
                # MCU telemetry sends int32 directly from little-endian STM32 memory.
                actual = struct.unpack_from("<i", payload)[0]
                writer.writerow((f"{time.time():.3f}", label, actual))
        time.sleep(0.002)


def main() -> int:
    parser = argparse.ArgumentParser(description="X 轴角速度环安全阶跃调参")
    parser.add_argument("--port", required=True, help="例如 COM12")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--p-list", default="0.0005,0.0008,0.0012,0.0016",
                        help="从小到大测试的 Kp 列表")
    parser.add_argument("--target", type=int, default=20, help="阶跃目标，单位 °/s，范围 ±30")
    parser.add_argument("--seconds", type=float, default=3.0, help="每个 P 的保持时间")
    parser.add_argument("--csv", default="x_rate_tune.csv", help="输出记录文件")
    args = parser.parse_args()

    try:
        p_values = [float(x.strip()) for x in args.p_list.split(",")]
    except ValueError:
        raise SystemExit("--p-list 必须是逗号分隔的小数，例如 0.0005,0.001")
    if not p_values or any(p <= 0.0 or p > 0.005 for p in p_values):
        raise SystemExit("为安全起见，每个 Kp 必须在 (0, 0.005] 内")
    if not -30 <= args.target <= 30 or args.target == 0:
        raise SystemExit("为安全起见，--target 必须是非零的 -30~30 °/s")
    if args.seconds < 1.0:
        raise SystemExit("--seconds 至少为 1 秒")

    path = Path(args.csv).resolve()
    print("确认：板端已切至 X Rate PID，野火助手已关闭。Ctrl+C 可立即停机。")
    with serial.Serial(args.port, args.baud, timeout=0.02) as port, \
         path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(("unix_time", "test", "actual_rate_dps"))
        try:
            for kp in p_values:
                label = f"kp={kp:g},target={args.target}"
                print(f"测试 {label}")
                send_pid(port, kp, 0.0, 0.0)
                send_target(port, args.target)
                send_command(port, START)
                port.flush()
                collect(port, args.seconds, writer, label)
                safe_stop(port)
                time.sleep(0.8)
        except KeyboardInterrupt:
            print("\n用户中断，正在归零停机。")
        finally:
            safe_stop(port)
    print(f"完成，曲线数据已保存：{path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
