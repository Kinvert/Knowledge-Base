# 🟨 FPGA (Field-Programmable Gate Array)

**FPGA** stands for **Field-Programmable Gate Array**, a type of integrated circuit that can be **programmed after manufacturing** to implement custom digital logic. Unlike microcontrollers or CPUs, FPGAs do not execute instructions sequentially—they instead perform computation through **configurable hardware blocks** wired together to form custom circuits.

---

## 🧠 Summary

- Hardware description is typically written in **HDL** (Hardware Description Language) like **Verilog** or **VHDL**.
- Enables parallelism and determinism at the hardware level.
- Used for applications requiring **high performance**, **low latency**, or **precise timing control**.

---

## 🧰 Key Features

| Feature               | Description                                                             |
|-----------------------|-------------------------------------------------------------------------|
| Reconfigurability     | Can be reprogrammed with new logic even after deployment                |
| Parallel Processing    | Executes many operations simultaneously, unlike serial CPUs            |
| Deterministic Timing  | Ideal for real-time systems and tight timing constraints                |
| Custom Peripherals    | Create your own UARTs, SPI, DMA, etc. directly in hardware              |
| Hardware Acceleration | Speed up compute-intensive tasks (e.g., encryption, ML inference)       |

---

## ⚙️ Common Use Cases

- High-speed data acquisition and DSP
- Signal processing in radar, sonar, SDR
- Industrial and automotive control systems
- High-frequency trading (HFT)
- Custom video/image processing pipelines
- Emulation of legacy hardware (e.g., retro gaming)

---

## 🛠️ Tools & Languages

| Tool / Lang   | Purpose                                |
|---------------|----------------------------------------|
| Verilog       | RTL hardware design (HDL)              |
| VHDL          | Alternative HDL, often in academia     |
| SystemVerilog| Superset of Verilog for verification    |
| Vivado        | Xilinx FPGA IDE                        |
| Quartus       | Intel (Altera) FPGA IDE                |
| IceStorm      | Open-source toolchain for Lattice FPGAs|
| nmigen        | Python-based HDL                       |

---

## 🧩 Popular FPGA Families

| Vendor    | Series / Models                             | Notes                                     |
|-----------|---------------------------------------------|-------------------------------------------|
| Xilinx    | Spartan, Artix, Kintex, Virtex, Zynq        | Broad use in industry; Vivado software    |
| Intel     | Cyclone, Arria, Stratix                     | Previously Altera                         |
| Lattice   | iCE40, ECP5                                 | Small, power-efficient, open-source tools |
| Microchip | SmartFusion, IGLOO                          | Mixed-signal and ultra-low power          |

---

## 🔌 Development Boards

| Board              | FPGA Chip      | Notes                                              |
|--------------------|----------------|----------------------------------------------------|
| DE10-Nano          | Intel Cyclone V| Popular for education and hobbyist use (Terasic)   |
| Digilent Arty A7   | Xilinx Artix-7 | Good for Xilinx Vivado-based projects              |
| TinyFPGA BX        | Lattice iCE40  | USB-powered, fits breadboards                      |
| ULX3S              | Lattice ECP5   | Open-source friendly, powerful and flexible        |
| Mojo V3            | Spartan-6      | Beginner-friendly, Arduino-like                   |

---

## ⚖️ FPGA vs MCU vs ASIC

| Feature         | FPGA                | Microcontroller (MCU)     | ASIC                         |
|-----------------|---------------------|----------------------------|-------------------------------|
| Reprogrammable  | ✅ (many times)      | ✅ (software)              | ❌ (fixed logic)              |
| Speed           | ✅ (parallel)        | ⚠️ (limited by CPU)         | ✅ (optimized)                |
| Power Usage     | ⚠️ (can be high)     | ✅ (low)                   | ✅ (optimized)                |
| Cost            | ⚠️ (moderate to high)| ✅ (low)                   | ❌ (very high development cost)|
| Time to Market  | ✅ (quick iterations)| ✅                          | ❌ (long lead times)          |
| Flexibility     | ✅                  | ⚠️ (via software)           | ❌                            |

---

## 🏆 Strengths

- Parallel processing and determinism
- Custom-tailored hardware for specific tasks
- Post-manufacturing reconfiguration
- Can emulate or simulate other digital systems

---

## ⚠️ Weaknesses

- Steeper learning curve than software development
- Toolchains are complex and often proprietary
- Power consumption higher than MCUs
- Not ideal for general-purpose computing

---

## 🔗 Related Notes

- [[Verilog]]
- [[VHDL]]
- [[JTAG]]
- [[Digital Logic]]
- [[Signal Processing]]
- [[Hardware Acceleration]]
- [[ASIC]]
- [[RTOS]]

---

## 🌐 External Links

- [Xilinx FPGAs](https://www.xilinx.com/)
- [Intel FPGAs](https://www.intel.com/content/www/us/en/products/programmable/fpga.html)
- [Lattice Semiconductor](https://www.latticesemi.com/)
- [Awesome FPGA List](https://github.com/hdl/awesome)

---
