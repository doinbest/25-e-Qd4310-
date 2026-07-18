"""Host-side deterministic vectors for MaixCAM2 <-> STM32 UART protocol V1."""

import struct


SOF = b"\x55\xAA"
VERSION = 0x01


def crc16_ccitt_false(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def make_frame(frame_type, flags, seq, payload):
    body = struct.pack(">BBBBB", VERSION, frame_type, flags, seq, len(payload)) + payload
    return SOF + body + struct.pack(">H", crc16_ccitt_false(body))


class FrameParser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while True:
            start = self.buffer.find(SOF)
            if start < 0:
                self.buffer = self.buffer[-1:] if self.buffer.endswith(b"\x55") else bytearray()
                return frames
            del self.buffer[:start]
            if len(self.buffer) < 7:
                return frames
            if self.buffer[2] != VERSION or self.buffer[6] > 16:
                del self.buffer[0]
                continue
            size = 9 + self.buffer[6]
            if len(self.buffer) < size:
                return frames
            frame = bytes(self.buffer[:size])
            del self.buffer[:size]
            body = frame[2:-2]
            if struct.unpack(">H", frame[-2:])[0] == crc16_ccitt_false(body):
                frames.append(frame)


def main():
    assert crc16_ccitt_false(b"123456789") == 0x29B1

    vision = make_frame(0x01, 0x0B, 0x7F, struct.pack(">ffB", -12.5, 3.25, 0x42))
    imu = make_frame(0x02, 0x03, 0x80, struct.pack(">fff", 1.5, -2.0, 360.0))
    control = make_frame(0x81, 0x01, 0xFE, bytes((0x02, 0x12, 0x34)))

    assert vision.hex(" ") == "55 aa 01 01 0b 7f 09 c1 48 00 00 40 50 00 00 42 73 67"
    assert imu.hex(" ") == "55 aa 01 02 03 80 0c 3f c0 00 00 c0 00 00 00 43 b4 00 00 76 89"
    assert control.hex(" ") == "55 aa 01 81 01 fe 03 02 12 34 36 51"

    parser = FrameParser()
    assert parser.feed(b"noise" + vision[:8]) == []
    assert parser.feed(vision[8:] + imu + control) == [vision, imu, control]

    damaged = bytearray(imu)
    damaged[-1] ^= 0x01
    assert parser.feed(damaged) == []
    assert parser.feed(b"\x55\xAA\x01\x02\x00\x00\x11" + control) == [control]
    assert parser.feed(b"\x55\x00garbage" + control) == [control]

    seq_255 = make_frame(0x02, 0x03, 0xFF, struct.pack(">fff", 0.0, 0.0, 0.0))
    seq_000 = make_frame(0x02, 0x03, 0x00, struct.pack(">fff", 0.0, 0.0, 0.0))
    wrapped = parser.feed(seq_255 + seq_000)
    assert [frame[5] for frame in wrapped] == [0xFF, 0x00]
    print("protocol_v1_test: PASS")


if __name__ == "__main__":
    main()
