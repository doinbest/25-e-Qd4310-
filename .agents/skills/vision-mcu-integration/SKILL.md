---
name: vision-mcu-integration
description: Design, review, or modify MaixCAM2 vision feedback and its UART or other communication link to an STM32 or MSPM0 controller. Use for target detection results, packet formats, coordinate mapping, validity flags, serial parsing, timeouts, and vision-to-motor control integration; do not use for camera-only model tuning without MCU communication.
---

# Vision MCU Integration

Treat the camera as a perception node and the MCU as the final controller and safety authority.

## Integration workflow

1. Read both endpoints before changing either one: camera output generation and send code, MCU receive buffer and parser, control use site, and existing timeout or stop logic.
2. Establish and document the packet contract from code or project documentation: framing, field order, payload length, signedness, endianness, scale, unit, coordinate origin, axes, positive directions, checksum, and send rate. Do not infer missing fields.
3. Make compatible changes to both endpoints only when a protocol change is explicitly required. Otherwise preserve the deployed protocol.
4. Verify failure behavior: target not found, invalid frame, partial frame, concatenated frames, lost bytes, repeated data, stopped camera transmission, and MCU communication timeout.

## Vision-side rules

- Initialize camera, display, model, and communication objects once; do not recreate them in the frame loop.
- Send simple observations such as `valid`, `target_x`, `target_y`, `error_x`, `error_y`, `angle_deg`, or confidence. Keep coordinate unit and direction explicit.
- When multiple detections exist, apply a stated selection rule. Do not assume the first result is the correct target without an API guarantee.
- On target loss, send an invalid result when the existing protocol supports it. Debug display and prints must not change the transmitted data or create avoidable frame-loop delays.

## MCU-side rules

- The MCU owns mode management, data validity, timeout decisions, PID calculation, output limiting, and motor stop decisions. The camera must not directly command PWM or motor current.
- Parse framing robustly and bound every receive-buffer access. Update the last-valid-data timestamp only after a complete, validated packet.
- Reject stale, malformed, or out-of-range data and enter the existing safe behavior. Do not continue controlling with old vision data after timeout.
- Confirm the camera-image coordinate direction against the MCU control and motor directions before applying a sign change. Keep the reason for any inversion in a concise comment.

## Handoff checklist

Report the confirmed protocol fields and directions, both endpoints changed, simulated or static checks completed, and hardware checks still required: byte stream behavior, timeout stop, camera-to-actuator direction, and target-loss response.
