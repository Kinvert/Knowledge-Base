# 🟧 OpenOCD

**OpenOCD (Open On-Chip Debugger)** is an open-source tool for debugging, flashing, and testing embedded systems via interfaces like **JTAG** and **SWD**. It acts as a bridge between a target device and a debugger frontend (e.g. GDB).

---

## 🧠 Summary

- Primarily used to interface with embedded microcontrollers over **JTAG/SWD**.
- Provides support for flash programming, CPU register access, and stepping through code.
- Works with many USB-based debug adapters (e.g. [[FT232H]], [[Bus Pirate]], [[ST-Link]], [[J-Link]]).
- Integrates with GDB for full source-level debugging of firmware.

---

## ⚙️ Key Features

| Feature               | Description                                              |
|-----------------------|----------------------------------------------------------|
| JTAG/SWD Support      | Standard protocols for embedded debugging                |
| GDB Server Mode       | Acts as a remote target for `gdb` to communicate with    |
| Flashing Firmware     | Write firmware directly to microcontroller memory        |
| Scripting             | TCL scripting support for automation                    |
| Broad Adapter Support | Works with FTDI, ST-Link, J-Link, CMSIS-DAP, and more    |

---

## 🧰 Supported Interfaces

- **FTDI**-based (e.g. FT2232H)
- **CMSIS-DAP**
- **ST-Link** (STMicroelectronics)
- **J-Link** (Segger)
- **Bus Pirate**
- **Raspberry Pi GPIO** (bitbang JTAG)

---

## 📦 Common Use Cases

- Debugging bare-metal embedded code.
- Flashing new firmware to MCUs (e.g. STM32, ESP32).
- Automating test scripts for embedded CI pipelines.
- Interfacing with custom hardware and bootloaders.

---

## 🏆 Strengths

- Widely supported and flexible.
- Free and open-source with community support.
- Supports hundreds of MCUs and adapters.

---

## ⚠️ Weaknesses

- Can be complex to configure (TCL-based config system).
- Performance may vary depending on adapter.
- GUI frontends are limited; mostly CLI-driven.

---

## 🔄 Related Comparison

| Tool         | Protocols     | GUI | Flashing | Debugging | Scriptable | Notes                                |
|--------------|---------------|-----|----------|------------|------------|--------------------------------------|
| OpenOCD      | JTAG, SWD     | ❌  | ✅       | ✅         | ✅ (TCL)    | Most versatile and open-source       |
| pyOCD        | SWD           | ❌  | ✅       | ✅         | ✅ (Python) | ARM Cortex-M only, simple interface  |
| Segger J-Link| JTAG, SWD     | ✅  | ✅       | ✅         | ❌         | Proprietary, but very fast           |
| Bus Pirate   | Bitbang/JTAG* | ❌  | ⚠️        | ⚠️          | ✅         | Slower, very low-level               |

---

## 🔗 Related Notes

- [[JTAG]]
- [[FT232H]]
- [[pyftdi]]
- [[Bus Pirate]]
- [[STM32]]
- [[ARM Cortex-M]]
- [[Debugging Tools]]

---

## 🌐 External Links

- [Official OpenOCD GitHub](https://github.com/openocd-org/openocd)
- [Documentation (OpenOCD Manual)](http://openocd.org/doc/html/)
- [Adapter Configuration Examples](https://github.com/ntfreak/openocd)

---
