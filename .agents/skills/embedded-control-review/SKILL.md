---
name: embedded-control-review
description: Review or change STM32 or MSPM0 bare-metal C11 control firmware, including motor, encoder, IMU, PWM, PID, interrupts, timers, and safety logic. Use for embedded control code reviews, refactors, feature changes, build checks, and hardware-facing debugging; do not use for pure MaixCAM2 vision work unless MCU control code is also in scope.
---

# Embedded Control Review

Keep embedded control changes small, verifiable, and safe for on-board testing.

## Review workflow

1. Read the relevant project configuration, entry points, drivers, and existing control path before editing. Confirm the MCU, SDK, peripheral instance, pins, units, directions, and control period from project files; label any missing fact as unconfirmed.
2. Trace the complete path: input or feedback -> validity checks -> control calculation -> output limit -> motor command -> fault or timeout stop.
3. Make the minimum change that meets the request. Preserve CubeMX user-code boundaries and do not edit vendor libraries, startup files, clock configuration, or pin configuration unless explicitly requested.
4. Build with the project's existing configuration when available. Report static-only verification and the need for hardware testing when a toolchain or board is unavailable.

## Bare-metal C11 rules

- Use fixed-width integer types and clear names that include units where relevant, such as `period_ms`, `angle_deg`, and `speed_rpm`.
- Keep the main loop or periodic scheduler visible. Do not add an RTOS, dynamic allocation, callback framework, or generic device layer without a clear project need.
- Put only short data capture, flag updates, and timestamp updates in ISRs. Keep parsing, floating-point control, printing, blocking waits, and motor-command logic outside interrupts.
- Use `volatile` only for data shared with an ISR or hardware; consider atomicity and update ordering for multi-byte shared values.
- Keep parameters that require field tuning together and document their unit and effect.

## PID and motor safety checks

- Confirm error sign, measurement unit, output unit, motor positive direction, and control period before changing PID logic or gains.
- Keep output, integral, and actuator limits. Reset or freeze integral when output is disabled, a mode changes, feedback is invalid, or a fault occurs as appropriate to existing behavior.
- Retain startup protection, target or feedback timeout handling, invalid-data handling, and a safe stop path. Never enable an actuator automatically at power-up unless the request explicitly requires it.
- Do not alter protocol, coordinate direction, PID gains, filter behavior, and output range together unless the user explicitly requests the combined change.

## Review output

State verified facts separately from unconfirmed hardware details. For code changes, summarize affected control and safety paths, validation performed, and any on-board checks still required.
