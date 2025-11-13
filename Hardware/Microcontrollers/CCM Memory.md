# CCM Memory (Core-Coupled Memory) 🧠

Core-Coupled Memory (CCM) is a small, fast SRAM block on many microcontrollers that is connected directly to the CPU core (or a dedicated core port) rather than to the system bus matrix. CCM gives deterministic, low-latency access for the core but is often **not** reachable by DMA or some peripherals and — depending on the MCU variant — may be data-only or executable.

---

## ⚙️ Overview
CCM is a tight, low-latency RAM region intended for latency-sensitive data and sometimes code. STM32 families (notably F2/F3/F4 and many newer G4 parts) expose a CCM/CCMRAM region separate from general SRAM; behaviour varies (some parts allow execution, some are data-only). CCM is typically on a dedicated port to the core, bypasses bus congestion, and therefore often offers zero-wait or deterministic access under load.

---

## 📚 Summary (one paragraph)
Use CCM for worst-case deterministic data access (critical RTOS heaps, interrupt scratch buffers, lockless ring buffers for the core), but be aware that peripherals/DMA usually cannot access CCM — so anything needing DMA must live in normal SRAM or be staged through it. For code execution check the specific MCU datasheet: on some STM32 families CCM is data-only, on others parts of it can execute.

---

## 🧩 Core Concepts
- **Dedicated connection** — CCM connects directly to core (like TCM concept) for deterministic timing.
- **Non-DMA region** — most CCM regions are not on the system bus matrix and so are unreachable by DMA/peripherals.
- **Size & placement** — CCM is small (e.g., 32–128 KB typical on STM32 families) and must be explicitly assigned in the linker script to place symbols there.
- **Executable vs data-only** — behaviour depends on family: some allow code in CCM, some only data. Always confirm in the reference manual.
- **Deterministic latency** — useful for real-time tasks where cache unpredictability is unacceptable (TCM/CCM vs cache tradeoffs).

---

## 🔍 Comparison Chart (CCM vs other memory types)

| Feature / Metric | CCM (Core-Coupled) | On-chip SRAM (general) | TCM (ITCM/DTCM) | Cache (L1/L2) | Flash / EEPROM |
|---|---:|---:|---:|---:|---:|
| Typical size | small (tens KB) | medium to large (KBs→MBs) | small, dedicated (TCM) | invisible to programmer (cache) | large, non-volatile |
| Access latency | extremely low / deterministic | low but bus-dependent | deterministic (designed for core) | low average, but variable (cache miss) | very high |
| DMA accessible? | usually **no** (not on system bus). | yes (usually) | may be no (depends on design) :| not directly (cache is transparent) | not directly |
| Executable? | varies (family dependent). | yes | ITCM executable, DTCM for data | code lives in main memory, cached | code storage |
| Best for | deterministic data, core scratch buffers, ISR scratch | general data, DMA buffers | critical code/data with deterministic timing | throughput/bandwidth acceleration | code/data persistence |

---

## 🧾 Use Cases
- ISR scratch buffers and critical RTOS data structures (reducer worst-case latency).
- Small, latency-sensitive heaps (FreeRTOS heap placed in CCM).
- Lockless structures accessed only by CPU core (ring buffers, lookup tables).
- Temporary staging for CPU-only algorithms where DMA involvement is unnecessary.

---

## ✅ Strengths
- Deterministic, low access latency.
- Avoids bus contention when peripherals are busy (core gets dedicated access).
- Useful for meeting real-time deadlines without cache timing variability.

---

## ⚠️ Weaknesses
- Often **not** DMA/peripheral accessible — major limitation for data movement.
- Small capacity — cannot hold large buffers or bulk data.
- Family differences: confusing rules across STM32 variants (some allow execute, others don’t).

---

## 🧾 Pros / Cons (quick)
- Pros: deterministic timing, reduces contention, great for critical data.
- Cons: limited size, DMA inaccessible, toolchain/linker work required.

---

## 🛠️ How to Use / How It Works
- Check the MCU reference manual to find the CCM base address, size and whether it’s executable.
- Modify the linker script to add a `.ccmram` (or similarly named) section and assign variables or code to that section.
- Be explicit: mark DMA buffers to live in normal SRAM; mark core-only buffers to CCM.
- Consider cache/TCM tradeoffs: if the MCU offers TCM (ITCM/DTCM) that may be a cleaner architectural option for deterministic code/data.

---

## 🧑‍💻 ONE-LINERS — Handling CCM in different languages / environments
- **C (GCC)**: `__attribute__((section(".ccmram")))` on globals or add linker script entry for `.ccmram`.
- **C++**: Same `__attribute__((section(".ccmram")))` or `alignas(...)` + section placement for static objects.
- **Rust**: `#[link_section = ".ccmram"] static mut FOO: [u8;N] = [0;N];` and adjust `memory.x` linker script.
- **Assembly**: Place labels in a `.section .ccmram` block and ensure vector/branch targets allowed by architecture.
- **MicroPython / CircuitPython**: Usually unavailable at runtime — prefer writing C modules or allocating via linker; MicroPython abstraction rarely controls placement.
- **FreeRTOS**: Move the heap (`heap_4` etc.) to CCM by placing the heap array in `.ccmram` (linker change).
- **Linker scripts**: Add a `MEMORY` region for CCM and an output section mapping `*(.ccmram*)`. `ld` changes required.
- **Bare metal / bootloaders**: Initialize CCM data like any RAM region; ensure startup code clears `.bss` in CCM.
- **High-level languages (Go/Ada)**: Similar to Rust — use `link_section`/pragma or compile-time placement; not common on constrained MCUs.

---

## 🔁 Variants & Vendor Implementations
- **STM32**: Many F2/F3/F4/G4 parts have CCMRAMA / CCM; size and executable rules differ by family.
- **ARM TCM**: Semantically similar concept (ITCM/DTCM) offered on some M7/M33 designs — explicitly architected for deterministic code/data.
- **Other vendors**: Microchip/NXP/others provide TCM-like tightly-coupled RAMs for critical code on some M7 and high-end MCUs (Microchip AN5740).

---

## 🔗 Compatible Items (examples)
- STM32F4xx (64 KB CCM on many parts) — good for CPU-only heaps.
- STM32F3 / STM32G4 families — have CCM with slightly different capabilities; refer to AN4296.
- Cortex-M7 MCUs with ITCM/DTCM (Microchip, NXP) — use TCM for deterministic code/data.

---

## 🧭 Related Concepts / Notes
- [[STM32]]
- - [[TCM]] (Tightly Coupled Memory) — deterministic memory model.
- - [[SRAM]] (On-chip SRAM) — general use RAM, DMA accessible.
- - [[Cache]] — non-deterministic but high throughput.
- - [[Linker Scripts]] (memory placement basics) — how to map sections into CCM.
- - [[FreeRTOS]] (heap placement into CCM) — practical use case.

---

## 🧰 Developer Tools
- STM32CubeIDE (linker script editing, scatter file support).
- GNU ld / `arm-none-eabi-gcc` toolchain (custom `.ld` memory regions).
- IAR / Keil: use their scatter/placement pragmas and project memory map settings.

---

## 📖 Documentation & Support
- ST ANs and reference manuals (search `<device> reference manual` for CCM/CCMRAM section).
- Zephyr / RTOS examples show `__ccm_data_section` usage and caveats.
- Community threads (StackOverflow, ST community) are useful for device-specific gotchas.

---

## 📚 External Resources / Further Reading
- ST application note: *Use STM32F3/STM32G4 CCM SRAM...* (AN4296).
- Zephyr CCM sample README (practical placement examples).
- ARM docs on TCM and memory system (differences between cache and TCM).

---

## 🧭 Key Highlights (TL;DR)
1. CCM = dedicated small RAM for the CPU with deterministic low latency.
2. Usually **not** DMA or peripheral accessible — plan buffer placement accordingly.
3. Toolchain/linker script changes needed to place symbols into CCM.
4. Behavior (executable/data-only) depends on MCU family — always check the datasheet/reference manual.

---

## 🧭 Examples (note)
For code examples (linker snippets, GCC attributes, Rust `link_section`) I can provide short, single-line examples in a follow-up message if you want them formatted for your toolchain — they were omitted here per your `markdown` constraints.

