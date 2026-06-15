---
aliases:
  - Zynq 7000
  - Zynq-7000 Series
  - Zynq7000
  - Xilinx Zynq-7000
  - AMD Zynq-7000
  - "Zynq-7000 SoC"
title: Zynq-7000 SoC
tags:
  - FPGA
  - SoC
  - ARM
  - Xilinx
  - AMD
---

# Zynq-7000 SoC

The **Zynq-7000** line is a heterogeneous system-on-chip family that combines ARM processing and FPGA fabric in one chip. It was introduced by Xilinx as a way to blend software-defined logic with real-time hardware acceleration in the same device.

---

## 🧠 Core Architecture

- **PS (Processing System)**: dual-core ARM Cortex-A9 processor subsystem, with cache and memory subsystem for Linux or RTOS use.
- **PL (Programmable Logic)**: Xilinx FPGA fabric for custom peripherals, DSP blocks, and high-throughput pipelines.
- **Interconnect**: high-throughput ports between PS and PL (including AXI) for DMA and control paths.
- **Memory**: on-device support for DDR3/DDR3L in many parts, plus optional OCRAM and peripheral blocks.
- **Peripherals**: USB OTG, Gigabit Ethernet, I/O controllers, timers, UARTs, SPI/I2C, SD/MMC and more depending on device variant.
- **Device families**: part numbers in the `XC7Z0xx`/`XC7Z1xx` style and associated package/speed variants.

---

## ✅ Why it matters

Zynq-7000 devices are still useful when you need:

- Tight hardware/software partitioning in a single board/chip.
- Low-latency data path logic in PL beside a Linux-capable CPU.
- Deterministic real-time accelerators alongside standard application software.
- Reuse of a single codebase for both bare-metal and software-rich environments.

---

## 🧩 How it is typically used

1. Put deterministic/control-heavy logic into PL (e.g., protocol endpoints, filters, video preprocessing).
2. Keep application, networking, storage, and user logic in PS.
3. Use AXI DMA, interrupts, and coherent shared memory pathways to move data.
4. Iterate between hardware and software with tight integration in build and boot flow.

---

## 🧪 Tooling and workflow

- **FPGA build:** Xilinx Vivado for hardware design, bitstream generation, and timing.
- **Software flow:** Vitis / Xilinx SDK-era toolchain for application and drivers.
- **OS support:** often booted with Linux (including distro images), or smaller RTOS/bare-metal stacks when real-time determinism is primary.
- **Dev boards:** commonly deployed on evaluation and carrier boards for camera, robotics, industrial I/O, and embedded vision work.
- **Boot process:** bootloader and FSBL chain plus design bitstream load and application startup.

---

## ⚖️ Comparison Chart

| Platform | CPU type | FPGA/PL | Linux-friendly? | Real-time control strength | Typical sweet spot |
|---|---|---|---:|---|---|
| **Zynq-7000 SoC** | Dual Cortex-A9 | Yes (Xilinx FPGA fabric) | Yes | Strong | Mixed FPGA + Linux systems |
| **Raspberry Pi 5** | Multi-core ARM CPU only | No | Strong | Single-node SBC, general Linux apps |
| **Jetson Nano / Jetson Family** | ARM + NVIDIA CUDA GPU | No | Moderate | AI/vision acceleration with GPU stacks |
| **Intel/Altera Cyclone V SoC** | ARM + FPGA fabric | Yes | Strong | FPGA-heavy designs in Intel toolchains |
| **Raspberry Pi Pico** | Dual ARM Cortex-M0+ | No | Excellent for deterministic I/O only | Microcontroller-focused edge tasks |
| **Raspberry Pi** | ARM SBC | No | Good for non-hard-real-time use | General prototyping and Linux apps |

---

## 🗂️ Strengths

- Integrated compute + reconfigurable hardware path without adding extra boards.
- Mature ecosystem in industrial and legacy embedded pipelines.
- Good fit for video pipelines, custom interfaces, robotics, and deterministic signal paths.
- Strong vendor documentation history and established reference designs.

## ⚠️ Weaknesses / tradeoffs

- Development can be toolchain-heavy versus purely software boards.
- Power/timing closure and resource tuning take real FPGA design discipline.
- Older microarchitecture compared with newer AMD/FPGA SoCs with more modern cores.
- Supply-chain variance and board choice strongly impact your achievable performance.

---

## 🧱 Design patterns

- Put only timing-sensitive or bandwidth-bound functions in PL.
- Keep Linux services and application logic in PS for maintainability.
- Use hardware abstraction around AXI/interrupt paths to reduce cross-domain bugs.
- Validate timing and DMA throughput early; PL can become bottleneck if interface sizing is underestimated.

---

## 🔍 Common pitfalls

- Over-mixing PS/PL responsibilities late in design causes expensive re-layout and debug loops.
- Assuming PL-generated logic is always faster than software; memory and bus bandwidth often dominate.
- Underestimating boot integration complexity when moving from board-specific BSP defaults to custom rootfs.
- Ignoring power/timing budgets in fanless or compact industrial layouts.

---

## Related notes

- [[FPGA]]
- [[Jetson Family]]
- [[Jetson Nano]]
- [[Raspberry Pi]]
- [[Raspberry Pi 5]]
- [[Raspberry Pi Pico]]
- [[Raspberry Pi Pico W]]

---

## External links

- [AMD Xilinx product pages (Zynq-7000 overview)](https://www.xilinx.com/products/silicon-devices/soc/zynq-7000.html)
- [AMD Adaptive SoC and SoC product docs portal](https://www.amd.com/en/products/processors-and-technologies/embedded-and-automotive-processors/)
- [AMD/Vivado Design Suite](https://www.xilinx.com/products/design-tools/vivado.html)
- [Vitis unified software platform](https://www.xilinx.com/products/design-tools/vitis.html)
- [Zynq-7000 technical reference manual (Zynq-7000 All Programmable SoC)](https://docs.amd.com/r/en-US/ug585-zynq-7000-technical-reference-manual)
