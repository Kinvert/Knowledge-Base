# PWM (Pulse Width Modulation)

**PWM** is a technique for encoding analog-like values using digital signals by varying the duty cycle of a square wave. A microcontroller outputs a fixed-frequency signal that switches between high and low, and the ratio of on-time to total period determines the effective power or control value. PWM is fundamental to motor control, LED dimming, servo positioning, and power regulation.

---

## 📚 Overview

Instead of outputting a true analog voltage (which would require a [[DAC]]), PWM rapidly switches a digital pin on and off. The average voltage perceived by the load depends on the duty cycle—50% duty cycle gives roughly half the supply voltage on average.

Key highlights:
- Digital pins can simulate analog output
- Hardware timers generate precise, CPU-free PWM
- Frequency and duty cycle are independently configurable
- Used across nearly all embedded systems

---

## 🧠 Core Concepts

- **Duty Cycle**
  Percentage of time the signal is HIGH. 0% = always off, 100% = always on, 50% = half power

- **Frequency**
  How fast the signal cycles. Higher frequencies reduce audible noise in motors but increase switching losses

- **Resolution**
  Number of discrete duty cycle steps (8-bit = 256 levels, 16-bit = 65536 levels)

- **Hardware Timer**
  Dedicated peripheral that generates PWM without CPU intervention

- **Dead Time**
  Brief delay between switching in H-bridges to prevent shoot-through

- **Complementary PWM**
  Paired outputs for driving high-side and low-side switches (used in [[H-Bridge]], [[FOC]])

---

## 📊 Comparison Chart

| Application | Typical Frequency | Duty Cycle Use | Notes |
|-------------|-------------------|----------------|-------|
| **LED dimming** | 500 Hz - 25 kHz | Brightness control | Higher freq avoids flicker |
| **DC motor speed** | 1 - 20 kHz | Speed control via [[H-Bridge]] | Audible whine below ~18 kHz |
| **Servo control** | 50 Hz (fixed) | Position encoding | 1-2 ms pulse width |
| **[[BLDC]]/[[ESC]]** | 8 - 48 kHz | Commutation timing | Higher for [[FOC]] |
| **Switching power supply** | 100 kHz - 2 MHz | Voltage regulation | Efficiency critical |
| **Audio (class D amp)** | 300+ kHz | Analog signal encoding | Filtered to audio |

---

## 🔧 Servo Control (Special Case)

Hobby servos use a specific PWM protocol:
- **Frequency:** 50 Hz (20 ms period)
- **Pulse width:** 1 ms = full left, 1.5 ms = center, 2 ms = full right
- The duty cycle encodes position, not power

```
│◄──────── 20 ms ─────────►│
┌──┐                        ┌──┐
│  │                        │  │
┘  └────────────────────────┘  └──
│◄►│
1-2 ms pulse = position
```

See [[Servo Motor]], [[Continuous Rotation Servo]]

---

## 🔧 Motor Speed Control

For DC motors via [[H-Bridge]]:
- PWM controls average voltage to motor
- Higher duty cycle = more speed
- Direction controlled by H-bridge switching pattern
- Frequency chosen to minimize audible noise and maximize efficiency

For [[BLDC]] via [[ESC]]:
- PWM input (1000-2000 µs) sets throttle
- ESC internally generates 3-phase commutation
- Advanced ESCs use [[FOC]] for smoother control

---

## 🔧 Microcontroller Implementation

Most MCUs have dedicated timer peripherals for PWM:

| MCU | PWM Channels | Resolution | Notes |
|-----|--------------|------------|-------|
| [[ATmega328P]] | 6 | 8-bit | Arduino Uno |
| [[ESP32]] | 16 | Up to 16-bit | LEDC peripheral |
| [[STM32F103R8T6]] | 15+ | 16-bit | Advanced timers |
| [[Teensy 4.1]] | 31 | 15-bit | FlexPWM + QuadTimer |
| [[Raspberry Pi Pico]] | 16 | 16-bit | PIO can extend |
| [[Parallax Propeller 1]] | 32 | Flexible | Cog-based, via counters |
| [[Parallax Propeller 2]] | 64 | Flexible | [[Smart Pins]] with native PWM |

The Propeller chips are unique—each cog can generate PWM independently, and the P2's smart pins have dedicated PWM modes built into the I/O hardware itself.

---

## 🔧 Basic Arduino Example

```cpp
// PWM on pin 9 (Timer1)
void setup() {
  pinMode(9, OUTPUT);
}

void loop() {
  // 0-255 duty cycle (8-bit)
  analogWrite(9, 127);  // 50% duty cycle
  delay(1000);
  analogWrite(9, 255);  // 100% duty cycle
  delay(1000);
}
```

---

## ✅ Pros

- Simple digital output simulates analog behavior
- Hardware timers free up CPU
- Precise, repeatable timing
- Low power switching losses (at reasonable frequencies)
- Universal support across all microcontrollers

---

## ❌ Cons

- Not true analog—high-frequency noise present
- Audible whine in motors at certain frequencies
- Resolution limited by timer bit-width
- EMI concerns at high frequencies/currents
- Servo protocol is timing-sensitive

---

## 🔩 Compatible Items

- [[H-Bridge]] - Motor direction + PWM speed
- [[ESC]] - Accepts PWM throttle input
- [[Servo Motor]] - Position control via pulse width
- [[FOC]] - Advanced PWM commutation
- [[BLDC]] / [[PMSM]] - Brushless motor control
- [[Encoder]] - Feedback for closed-loop PWM control
- [[PID Controller]] - Adjusts duty cycle based on error
- [[Motor Controllers]] - Integrate PWM generation
- [[Parallax Propeller 1]] / [[Parallax Propeller 2]] - Multicore MCUs with flexible PWM

---

## 🔗 Related Concepts

- [[GPIO]] (PWM output pins)
- [[Timer]] (Hardware PWM generation)
- [[ADC]] (Feedback for closed-loop control)
- [[DAC]] (True analog alternative)
- [[Smart Pins]] (P2's hardware PWM/ADC/DAC)
- [[H-Bridge]] (DC motor control)
- [[ESC]] (Brushless motor control)
- [[FOC]] (Field-oriented control)

---

## 📚 External Resources

- [Arduino PWM Tutorial](https://www.arduino.cc/en/Tutorial/Foundations/PWM)
- [ESP32 LEDC Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/ledc.html)
- [STM32 Timer/PWM Guide](https://www.st.com/resource/en/application_note/an4013-stm32-crossseries-timer-overview-stmicroelectronics.pdf)
- [Propeller 2 Smart Pin Documentation](https://www.parallax.com/propeller-2/)
