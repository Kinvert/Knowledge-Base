# FreeRTOS

FreeRTOS is an open-source Real-Time Operating System (RTOS) kernel for embedded devices, widely used in robotics, IoT, and automotive systems. It's known for being lightweight, deterministic, and easy to integrate into bare-metal microcontroller environments. While FreeRTOS can run standalone, it's often used in conjunction with platform-specific SDKs and HALs. 

---

## ⚙️ Overview

FreeRTOS (Free Real-Time Operating System) provides preemptive, cooperative multitasking capabilities for microcontrollers and small embedded systems. Developed by Real Time Engineers Ltd., and now stewarded by Amazon Web Services (AWS), it supports a large array of architectures (ARM Cortex-M, RISC-V, etc.) and is available under the MIT license.

---

## 🧠 Core Concepts

- **Tasks**: Lightweight threads with optional priorities.
- **Scheduler**: Determines which task runs at any given time.
- **Queues**: Thread-safe data transfer between tasks.
- **Semaphores/Mutexes**: Synchronization primitives.
- **Timers**: Software timers for deferred execution.
- **Hooks**: Idle, tick, and malloc failure hooks for diagnostics.
- **Tickless Idle**: Low-power sleep mode integration.

---

## 📊 Comparison Chart

| Feature                     | FreeRTOS       | Zephyr RTOS    | ChibiOS/RT     | ThreadX        | NuttX          | RTEMS          |
|----------------------------|----------------|----------------|----------------|----------------|----------------|----------------|
| License                    | MIT            | Apache 2.0     | GPL            | Proprietary    | BSD            | GPL            |
| Memory Footprint           | Very Low       | Medium         | Low            | Low            | Medium         | High           |
| Supported Architectures    | Many (ARM, RISC-V, etc.) | Broad        | Many           | Many           | Broad          | Limited        |
| SMP Support                | No             | Yes            | No             | No             | Yes            | Yes            |
| POSIX Compatibility        | No             | Partial        | No             | No             | Yes            | Yes            |
| Ecosystem                  | Large          | Growing fast   | Moderate       | Azure focused  | Niche          | Academic/Space |
| AWS Integration            | Yes (FreeRTOS+) | No             | No             | Partial        | No             | No             |

---

## 🧰 Use Cases

- Small embedded systems
- Microcontroller firmware
- Low-power IoT devices
- Robotics task scheduling
- Automotive ECUs (non-safety-critical)
- Proof-of-concept RTOS experiments

---

## ✅ Strengths

- Tiny footprint (<10KB)
- Portable across many architectures
- Large user community
- Easy integration with HAL libraries
- Rich documentation and examples
- Simple and understandable kernel

---

## ❌ Weaknesses

- Lacks full POSIX support
- No native SMP (Symmetric Multiprocessing)
- Debugging tools less advanced than in larger OSes
- Not suitable for complex safety-critical systems without FreeRTOS+ components

---

## 🧩 Compatible Items

- [[STM32]] (Popular ARM Cortex-M microcontroller family)
- [[ESP32]]
- [[CMSIS]] (ARM Cortex Microcontroller Software Interface Standard)
- [[Segger J-Link]] (Debugger)
- [[PlatformIO]] (Embedded development environment)
- [[Amazon FreeRTOS]] (Cloud-connected version with OTA and security)

---

## 🔧 Developer Tools

- `make` or `cmake` toolchains
- `GDB` for debugging
- `OpenOCD` for JTAG/SWD
- `FreeRTOS Tracealyzer` for task profiling
- `Segger SystemView` for real-time analysis

---

## 📚 Documentation and Support

- Official site: https://www.freertos.org/
- Extensive examples per chip vendor (e.g. ST, Microchip, Espressif)
- AWS FreeRTOS documentation: https://docs.aws.amazon.com/freertos
- Community forums and GitHub discussions

---

## 🔄 Variants

- **Amazon FreeRTOS**: Adds TLS, MQTT, OTA, and AWS IoT Core integration.
- **SafeRTOS**: IEC 61508 SIL3-certified variant by Wittenstein High Integrity Systems.
- **FreeRTOS+TCP**: Lightweight TCP/IP stack.
- **FreeRTOS+FAT**: Embedded FAT filesystem support.

---

## 🧠 Related Notes

- [[RTOS]] (Real-Time Operating System)
- [[Zephyr RTOS]] (Lightweight RTOS with Linux-style features)
- [[ChibiOS]] (Compact RTOS with HAL)
- [[Embedded Systems]] (Low-level software development)
- [[Task Scheduling]] (Managing execution priorities)
- [[Memory Management]] (Static vs dynamic)
- [[Interrupts]] (Asynchronous event handling)
- [[Mutex]] (Mutual exclusion primitives)

---

## 🌐 External Resources

- https://www.freertos.org/
- https://github.com/FreeRTOS
- https://freertos.org/FreeRTOS_Support_Forum_Archive
- https://www.freertos.org/FreeRTOS-Plus/FreeRTOS_Plus_TCP/index.html
- https://www.freertos.org/FreeRTOS-Plus/FreeRTOS_Plus_FAT/index.html

---
