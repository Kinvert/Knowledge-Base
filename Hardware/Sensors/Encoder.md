---
title: Encoder
aliases:
  - Encoders
  - Rotary Encoder
  - Position Encoder
tags:
  - robotics
  - sensors
  - control
  - actuation
---

# Encoder

An **encoder** converts mechanical motion into electrical signals so control software can estimate position, velocity, and sometimes direction.

They are critical where precise state feedback is needed, such as motor drives, robotics joints, and motion platforms.

---

## Types

### Incremental encoder
- Reports relative motion through channel pulses.
- Requires a reference/home event for absolute position recovery after reset.
- Strong fit for high-speed closed loops and cost-sensitive systems.

### Absolute encoder
- Outputs absolute position directly at power-on.
- Prevents position ambiguity after power cycles.
- Preferred for axes where homing is expensive or impossible.

### Magnetic encoder
- Uses magnet+hall/sensor chain for compact implementation.
- Works well in sealed environments.
- Typically lower resolution than optical encoders.

### Optical encoder
- Common high-resolution solution for precise motion.
- Sensitive to dust and contamination.

---

## Key design dimensions

- **Resolution**: pulses per revolution (PPR) / counts per revolution (CPR).
- **Signal type**: quadrature (`A/B/Z`) vs SSI/CANopen/CIP variants.
- **Update rate**: affects control loop stability and derivative estimates.
- **Latency**: critical for high-bandwidth velocity control.
- **Failure behavior**: missing pulse detection and error flags.

---

## Common failure modes

- Line loss or jitter from EMI
- Missed ticks from low timer resolution
- Mechanical misalignment and shaft runout
- Wrong index wiring causing invalid home offset

---

## Comparison table

| Type | Position Persistence | Typical Cost | Resolution | Robustness | Best Use |
|---|---|---|---|---|---|
| Incremental | No | Low | Medium-High | Medium | High-speed control where homing is acceptable |
| Absolute | Yes | High | Medium | Medium | Servo axes, industrial arms |
| Magnetic | Yes | Low-Medium | Medium | High | Harsh environments |
| Optical | Yes | Medium | High | Low-Medium | Precision metrology and CNC |
| Hall-based | Yes | Low | Low | High | Simple low-cost brushless drives |

---

## In robotics

1. Use encoder + IMU in **sensor fusion** for odometry and attitude loops.
2. Use velocity derivatives for feed-forward and damping.
3. Filter raw counts before differentiation to avoid noise amplification.

---

## Related notes

- [[State Estimation]]
- [[Kalman Filter]]
- [[Motor Controllers]]
- [[PWM]]
- [[PID Controller]]

