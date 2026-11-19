# Luckfox

This note compares Luckfox SBC-style Linux boards against the PJRC Teensy MCU family for "agentic" automation workflows, with emphasis on what can actually be automated from scripts and what is realistically suitable for control-loop style tasks.

---

## Purpose of this note

Luckfox is often useful as:

- a compact Linux edge node for running scripts, HTTP APIs, and light compute
- a dev board for camera/IoT demos, media ingest, and Python services
- a cheap headless coordinator for orchestrating sensor rigs and remote jobs

What it is not typically good for:

- deterministic motor-control loops under sub-ms timing constraints
- replacing desktop-level CFD/geometry preprocessing for complex meshing
- full-time replacement for simulation hosts

In contrast, Teensy boards are MCU-class deterministic controllers with extremely fast compile+flash cycles, low power, and strong real-time behavior.

---

## Luckfox families and what the specs page actually implies

Luckfox naming is confusing because there are multiple families sharing the same “Pico” naming tree. The official pages split boards around Rockchip SoC generation:

- RV1103 boards (single-core Cortex-A7 + RISC-V, 0.5 TOPS NPU class)
- RV1106 boards (single-core Cortex-A7 + RISC-V, 0.5 to 1 TOPS NPU class, more RAM/storage options)

Core references for these families:

- RV1103/RV1103G1: Cortex-A7 1.2GHz + 0.5 TOPS INT4/8/16 NPU, 4MP@30 fps ISP.
- RV1106G2/G3: Cortex-A7 1.2GHz + NPU path up to 1 TOPS, 5MP@30 fps ISP, more mature Linux options.
- Linux side is `buildroot`-centric on many variants, with `luckfox-config`, ADB, and board-specific SDK config files.

The board pages repeatedly list:

- RAM in DDR2 or DDR3L ranges (64MB on smaller RV1103 boards, up to 256MB on RV1106 max-class variants).
- SPI NAND or eMMC storage variants depending on model.
- USB2 host/device support on almost all production boards.
- MIPI CSI camera support on a large subset.
- 10/100 Ethernet on versions carrying PHY.
- Optional Wi-Fi/BLE depending on model (especially RV1106 RV1106G3-based class such as Zero/Ultra W/BW variants).

Export-control restriction warnings are present on Luckfox storefront pages for multiple destinations.

---

## Luckfox vs Teensy: high-level fit map

| Platform family | Compute model | OS | Real-time behavior | IO style | Tooling model | Best for |
|---|---|---|---|---|---|---|
| Teensy LC / 2.x / 3.x / 4.x | MCU (Cortex-M / AVR) | none (bare-metal / RT-ish) | strong deterministic timing, low jitter | GPIO/USART/SPI/I2C/CAN/audio | `arduino`, `platformio`, `Makefile` + `teensy_loader_cli` | tight loops, deterministic motion, signal I/O |
| Luckfox RV1103 | Cortex-A7 + RISC-V (Linux SoC) | Linux userspace | cooperative only (non-RT) | rich Linux peripherals + GPIO/UART/SPI/I2C/CSI | Git SDK + shell + `make/build.sh` | edge services, video/vision, coordination |
| Luckfox RV1106 | Cortex-A7 + RISC-V + higher NPU/ram class | Linux userspace | cooperative only (non-RT) | same + more bandwidth/features | same + kernel/image matrix tooling | vision + lightweight AI + edge data collection |

Short version:
- Use Teensy when you need “hard real-time enough for motor loops and sensor determinism.”
- Use Luckfox when you need Linux and heavy software stacks near an embedded deployment site.

---

## Representative Luckfox board matrix (from official product pages)

Use this as a practical starting comparison. Prices are storefront-dependent and can move; treat as snapshot-style.

| Board | SoC | RAM | NPU | ISP | Storage default | Wireless | Camera | Notes |
|---|---|---:|---:|---|---|---|---|
| Luckfox Pico (RV1103) | RV1103 (A7 1.2GHz) | 64MB DDR2 | 0.5 TOPS | 4MP@30 | SPI NAND 128MB or microSD expansion | no | MIPI CSI (2-lane) | minimal-cost RV1103 entry board |
| Luckfox Pico Mini A / Mini B | RV1103G1 | 64MB DDR2 | 0.5 TOPS | 4MP@30 | SPI NAND / TF-dependent variants | no | MIPI CSI (2-lane) | differs mainly in flash preset |
| Luckfox Pico Plus / Plus M | RV1103G1 | 64MB DDR2 | 0.5 TOPS | 4MP@30 | flash + Ethernet variants | no | MIPI CSI (2-lane) | includes network-capable variants |
| Luckfox WebBee | RV1103G1 | 64MB DDR2 | 0.5 TOPS | 5MP@30 | SPI NAND 128MB | no | 2-lane CSI | includes USB-A and RJ45 enclosure-friendly variant |
| Luckfox Pico Pro | RV1106G2 | 128MB DDR3L | 0.5 TOPS | 5MP@30 | SPI NAND 256MB | no | 2-lane CSI | includes RTC battery header, UART/USB ecosystem |
| Luckfox Pico Max | RV1106G3 | 256MB DDR3L | 1 TOPS | 5MP@30 | SPI NAND / eMMC class variants | no | 2-lane CSI | higher RAM and throughput variants |
| Luckfox Pico Zero | RV1106G3 | 256MB DDR3L | 1 TOPS | 8M@15 / 5M@30 mentions | eMMC 8GB + microSD | Wi-Fi6 + BT 5.2/BLE | MIPI CSI 4-lane | onboard wireless and audio headers |
| Luckfox Pico Ultra / Ultra W / Ultra BW | RV1106G3 / G2 mix | 128/256MB DDR3L | 0.5 or 1 TOPS | 5MP@30 | eMMC (often 8GB) + external | some W/BW variants Wi-Fi6+BT | MIPI CSI | PoE / DPI variants in RV1106 family |

The table above intentionally focuses on behavior, not exact SKU edge variants.

---

## Teensy stack comparison across all version families

The PJRC technical table is the source of truth for exact per-version specs.

| Teensy family | CPU / FPU | Clock | RAM | Flash | USB/IO shape | Discontinuation / status signals |
|---|---|---|---:|---:|---|---|
| Teensy 2.0 | ATMEGA32U4 @16MHz | 16MHz AVR | 2.5KB | ~31.5KB | classic USB HID style | legacy AVR branch |
| Teensy++ 2.0 | AT90USB1286 @16MHz | 16MHz AVR | 8KB | ~127KB | same class as 2.0 | legacy AVR branch |
| Teensy LC | MKL26Z64VFT4 (M0+) | 48MHz | 8KB | ~62KB-ish class | tiny, low power design | still sold in limited/legacy periods |
| Teensy 3.2 | MK20DX256VLH7 | 72MHz (96MHz OC in some contexts) | 64KB | ~256KB | classic 3.x feature set | officially marked as discontinued in final batch notices |
| Teensy 3.5 | MK64FX512VMD12 | 120MHz | 256KB | 512KB | improved peripheral set | officially in 3.x sunset context |
| Teensy 3.6 | MK66FX1M0VMD18 | 180/240MHz class | 256KB | 1MB | USB host + SDIO + audio options | officially discontinued; limited batches only |
| Teensy 4.0 | IMXRT1062DVL6 (Cortex-M7) | 600MHz | 1MB | 2MB | USB host/device, advanced DMA/audio | actively recommended over 3.x |
| Teensy 4.1 | IMXRT1062DVJ6 (Cortex-M7) | 600MHz | 1MB + optional external RAM | 2MB + expansion | USB host/device, better connectors, SD + Ethernet optional in practice | current premium MCU option |

Important historical note from PJRC announcements:
- 3.x legacy parts (LC, 3.5, 3.6) were moved into limited supply and phased.
- 3.2 had final-batch language and temporary inventory refreshes; migration was explicitly recommended toward 4.x.
- PJRC also published explicit command-line loader mappings for all teensy mcu IDs, including `TEENSY41`, `TEENSY40`, `TEENSY36`, `TEENSY35`, `TEENSY32`, `TEENSYLC`, `TEENSY2`, etc.

---

## CLI reality check: where automation is strongest

### Teensy command flow (deterministic and mature)

Official CLI route:

```bash
teensy_loader_cli --mcu=TEENSY41 -w firmware.hex -v
```

You can script this in Makefiles, which is exactly how PJRC advertises it.

Observed mapping examples from official docs:

- `--mcu=TEENSY41`, `--mcu=TEENSY40`
- `--mcu=TEENSY36`, `--mcu=TEENSY35`, `--mcu=TEENSY32`
- `--mcu=TEENSYLC`, `--mcu=TEENSY2PP`, `--mcu=TEENSY2`
- also `--mcu=imxrt1062`, `--mcu=mk66fx1m0`, `--mcu=mk64fx512`, `--mcu=mk20dx256`, etc.

In practice:
- Very fast edit/compile/flash loop.
- Great for deterministic CI jobs that need tiny flash times.
- Very difficult to run complex networked orchestration directly on-device beyond what you compile in.

### Luckfox SDK CLI flow (Linux image pipeline)

Luckfox flow is image-centric, not “single binary push” by default:

1. Clone SDK (`gitee.com/LuckfoxTECH/luckfox-pico` or GitHub mirror).
2. Select board via `./build.sh lunch`.
3. Configure env vars (for example `RK_BOOT_MEDIUM`, `RK_KERNEL_DTS`, `RK_BOOTARGS_CMA_SIZE`).
4. Build image with `./build.sh`.
5. Flash using board-specific scripts (`rkflash.sh` family where supported) or platform image flow.

Docs explicitly show generated output file layout with build artifacts such as:
- `output/image/download.bin`
- `uboot.img`
- `kernel`, `rootfs`, `userdata`, `env.img`

Luckfox CLI workflows are stronger at:
- batch image generation
- custom partition/table variants
- integrating peripheral/driver stacks and Python tools

But they are heavier and slower than Teensy when doing tiny iteration cycles.

---

## Decision matrix for CFD-oriented automation loops

If your goal is:

1. parametric geometry generation (bluff body, diffuser angle sweep)
2. CAD export to STEP
3. meshing + solver execution (OpenFOAM likely)
4. convergence + metric extraction + loop

Then:

### Use Teensy-like MCU only for:
- deterministic actuator/sensor edges
- hard real-time gating or trigger logic
- field I/O that must happen at deterministic rates

### Use Luckfox-like SBC only for:
- coordinating remote experiments
- logging, network transport, dashboarding
- non-real-time glue code, queue dispatch, file staging
- lightweight pre/post-processing in Python/C shell scripts

### Keep the heavy solver loop on a dedicated host:
- a desktop/server (x86 arm/hpc) should own meshing, CFD execution, and parametric loops.
- use Luckfox/Teensy as endpoints in a controller network.

This is usually the practical split for “agent teams”:

- Host/Controller: OpenFOAM + meshing + Python optimizer
- Edge board A (Teensy): deterministic actuation/safety IO
- Edge board B (Luckfox): telemetry + dataset ingestion + experiment state machine

---

## Suggested automation pattern with JSON-driven design space

At minimum, script your sweep as declarative data:

```json
[
  {"case_id":"b1", "diffuser_angle":15, "ground_clearance":48, "cfd_seed":101},
  {"case_id":"b2", "diffuser_angle":18, "ground_clearance":48, "cfd_seed":102}
]
```

Then:

1. Parse JSON on host (`python`, `jq`).
2. Generate CAD parameters for your CAD engine (OpenSCAD / FreeCAD / Salome APIs).
3. Export STEP from deterministic build step.
4. Push geometry to CFD host.
5. Launch mesh + solve.
6. Collect metrics to JSON.
7. Publish compact status packet to Luckfox/Teensy controllers via MQTT/HTTP/serial.

This pattern is how you get repeatable agent loops.

---

## When Luckfox is the wrong tool

- Hard 100% deterministic control with 1–10 kHz timing constraints.
- very tiny footprint requiring low-power always-on battery <100mA class loops.
- if you need a single-step USB-HID programmer workflow in minutes.

In those cases, Teensy remains the simpler fit.

---

## When Luckfox is the right tool

- You need Linux userspace to host protocol stacks, web, databases, or Python dependencies.
- You want to experiment with vision/AI/edge processing in the same form factor.
- You need Ethernet/Wi-Fi/BT and file transfer without adding extra modules.
- You are comfortable with image-based deployment and can tolerate slower reflash cycles.

---

## Practical comparison by workflow primitive

| Workflow | Teensy 4.x | Teensy older (3.x/LC/2.x) | Luckfox RV1103 | Luckfox RV1106 |
|---|---|---|---|---|
| Deterministic GPIO timing | ✅✅✅ | ✅✅ | ⚠️ non-RT Linux | ⚠️ non-RT Linux |
| Bare-metal compile/upload loop | very fast | fast | slower (image-centric) | slower (image-centric) |
| USB device stack for peripherals | robust | robust | Linux USB host/device; more stack overhead | same |
| Sensor fusion at edge | good | good | good (with user-space stack) | better with more RAM |
| Remote orchestration from host | possible via serial/UART | possible | native SSH/HTTP possible | native SSH/HTTP possible |
| Python automation runtime | limited | limited | strong (full Linux/Python) | strong |
| Integrated CI-friendly script interface | yes via teensy_loader_cli | yes | yes via shell scripts, board config, git flows | yes |

## Detailed Luckfox/Teensy tradeoff by release class

This matrix is practical for deciding what to standardize in a production loop.

| Criterion | Luckfox Pico / Mini / Plus / Pro / Max / Zero / Ultra | Luckfox WebBee | Teensy 2.0 / 2.0++ | Teensy LC | Teensy 3.2 | Teensy 3.5 | Teensy 3.6 | Teensy 4.0 | Teensy 4.1 |
|---|---|---|---|---|---|---|---|---|---|
| Best control target | Linux edge gateway, vision, logging | Linux + more comms-oriented variants | Bare-metal USB HID/actuator node | Low-power MCU timing | legacy USB/IO, low-mid speed control | mid-high speed control | high-speed deterministic host I/O | same as 4.0 + richer I/O + more pins | same as 4.0 + SD/eMMC expansion + extra UART/SPI/Sd |
| Typical shell/app stack | BusyBox/Ubuntu-ish custom userspace | same | Arduino/PlatformIO | Arduino/PlatformIO | Arduino/PlatformIO | Arduino/PlatformIO | Arduino/PlatformIO | Arduino/PlatformIO | Arduino/PlatformIO |
| Typical CAD/CFD loop role | dispatcher / telemetry / API bridge | dispatcher / telemetry / API bridge | actuator interface only | actuator interface | legacy actuator only | legacy actuator only | hard real-time actuators + high-speed control | hard real-time actuators + highest flexibility |
| Compile/flash pace | slowest (image build + flash) | slowest (image build + flash) | very fast | fast | fast | fast-medium | medium-fast | medium-fast |
| Determinism target | soft real-time only | soft real-time only | moderate jitter control | moderate | moderate | moderate | low jitter with good discipline | low jitter with best margins |
| Built-in wireless | no (model-dependent BLE/Wi-Fi variants are rare) | no (rare) | none built-in | none | none | none | none | none |
| Hardware debug path | UART + SSH + script logs | UART + SSH + script logs | USB/serial + `teensy_loader_cli` | USB/serial + `teensy_loader_cli` | USB/serial + `teensy_loader_cli` | USB/serial + `teensy_loader_cli` | USB/serial + `teensy_loader_cli` | USB/serial + `teensy_loader_cli` |
| Cost envelope (base units) | low | low-mid | low | lower | lower-mid | lower | higher | higher |
| Long-run campaign suitability | high as service node | high as service node | excellent for deterministic I/O layer only | excellent for deterministic I/O layer only | excellent for deterministic I/O layer only | excellent for deterministic I/O layer only | excellent for deterministic I/O layer only | excellent for deterministic I/O layer also |

## Why this distinction matters in a CFD pipeline

For an automated bluff-body campaign:

- Put parameterized geometry generation on desktop/server CAD + mesh host.
- Put solver orchestration in versioned scripts.
- Let microcontrollers only own deterministic boundaries (actuators/sensors).
- Use Luckfox-type boards as the “operations sidecar” for remote logging, state publishing, and low-latency scheduling tasks.

This avoids building all CAD/meshing dependencies into microcontroller firmware and keeps the loop deterministic.

## Typical automation architecture (recommendation)

1. Human edits `design_space.xlsx` or `design_space.json`.
2. Pipeline controller parses design row.
3. Parametric CAD/mesher produces `STEP` + `STL`.
4. OpenFOAM host runs mesh + solve + QA.
5. Luckfox receives run summary and publishes MQTT/HTTP status.
6. Teensy boards receive only final actuation commands or local triggers.
7. Controller stores each case manifest for replayability.

This split prevents timing assumptions from bleeding from Linux services into hard-RT loops.

## Caveats with the current notes and data

Several specs here are board-generation specific and vary by retail lot and SKU variant. Treat flash sizes, RAM bins, and exact peripheral revisions as **revision-sensitive** and verify against the exact product page before procurement or hardening an automated pipeline.

---

## Gotchas and operational reality

- Luckfox pages mix `RV1103`, `RV1106`, and naming modifiers (`Mini`, `Plus`, `Pro`, `Max`, `Ultra`, `Zero`) in the same catalog section; double-check SKU vs board variant before purchase.
- Several Luckfox SKUs depend on storage medium defaults (SPI NAND vs eMMC vs SD card boot), which changes first-boot flow.
- RK boot media and board config are important; a wrong `BoardConfig-*` choice can make a build boot-loop or fail to auto-boot.
- If you need repeated OTA updates in field devices, automate image build + checksum + versioning carefully.
- A lot of Luckfox docs point to Ubuntu-22.04 x86_64 as the blessed build environment.

---

## Related notes

- [[Teensy 4]]
- [[Teensy 4.1]]
- [[Raspberry Pi Pico]]
- [[Microcontrollers]]
- [[OpenFOAM]]
- [[OpenFOAM Projects]]

---

## External references

- PJRC Teensy technical table: https://www.pjrc.com/teensy/techspecs.html
- PJRC Teensy CLI: https://www.pjrc.com/teensy/loader_cli.html
- PJRC Teensy 3.2 EOL: https://www.pjrc.com/teensy-3-2-end-of-life/
- PJRC product page set:
  - https://www.pjrc.com/store/teensy40.html
  - https://www.pjrc.com/store/teensy41.html
  - https://www.pjrc.com/store/teensy36.html
  - https://www.pjrc.com/store/teensy35.html
  - https://www.pjrc.com/store/teensy32.html
  - https://www.pjrc.com/teensy/teensylc.html
- Luckfox product pages:
  - https://www.luckfox.com/Luckfox-Pico/EN-Luckfox-Pico
  - https://www.luckfox.com/EN-Luckfox-Pico-Pro
  - https://www.luckfox.com/luckfox-pico-zero
  - https://www.luckfox.com/Luckfox-Pico/Luckfox-WebBee
  - https://www.luckfox.com/EN-Luckfox-Pico-Plus?ci=531
- Luckfox wiki:
  - https://wiki.luckfox.com/Luckfox-Pico/Luckfox-Pico-RV1103/
  - https://wiki.luckfox.com/Luckfox-Pico-Zero/
  - https://wiki.luckfox.com/Luckfox-Pico-RV1106/
  - https://wiki.luckfox.com/Luckfox-Pico-Plus-Mini/SDK-Image-Compilation/
  - https://wiki.luckfox.com/Luckfox-Pico-WebBee/SDK/
  - https://wiki.luckfox.com/

---

## Summary

For a deterministic control-first stack, Teensy still wins.
For Linux automation-first stacks, Luckfox gives better software surface area and connectivity.
For CFD design-space loops, neither is a replacement for the solver host: treat both as distributed I/O and orchestration endpoints, not your CFD engine.
