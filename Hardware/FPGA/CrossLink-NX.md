# CrossLink-NX (Lattice CrossLink-NX FPGA)

Lattice **CrossLink-NX** is a low-power FPGA family optimized for embedded vision, camera/display bridging, and **sensor aggregation** (combining multiple MIPI [[CSI-2]] streams into one). Built on the Nexus platform (28 nm FD-SOI), it targets applications where you need hardened [[D-PHY]] blocks, tiny packages, fast I/O, and deterministic, low-latency data movement between sensors and processors. :contentReference[oaicite:0]{index=0}

---

## ⚙️ Overview

- **Domain**: Embedded vision, robotics, automotive, edge AI pre/post-processing  
- **Why it exists**: To bridge/aggregate/duplicate camera streams and offload light vision tasks before an SoC/ISP/CPU/GPU sees the data  
- **Signature features**: Up to **two hardened 4-lane MIPI D-PHY** (TX/RX), soft D-PHY on GPIO for extra lanes, small packages, low power, robust SER figures; 17 K–40 K logic-cell class devices and family variants (including CrossLinkU-NX with hard USB). :contentReference[oaicite:1]{index=1}

---

## 🧠 Core Concepts

- **Hardened MIPI**: Native D-PHY blocks reduce power and ease timing vs “soft D-PHY” on regular I/O. :contentReference[oaicite:2]{index=2}  
- **Sensor Aggregation**: Merge N camera streams into one CSI-2 output using **Virtual Channels (VCs)** or spatial compositing (side-by-side, tiling), optionally color-convert/resize on-FPGA. :contentReference[oaicite:3]{index=3}  
- **Bridging & Duplication**: CSI-2↔CSI-2, CSI-2→HDMI/DSI, display splitting/replication for redundancy. :contentReference[oaicite:4]{index=4}  
- **Deterministic Pipelines**: Line/frame-synchronous processing with minimal jitter—useful for robotics timestamping and fusion.

---

## 🔭 Common Uses (Robotics & Embedded)

- **Multi-camera rigs** (stereo, surround, 360): 2:1 or 4:1 aggregation to a single AP input using VCs or mosaics  
- **SoC interoperability**: Adapt sensors to APs with fewer CSI-2 ports (e.g., [[Jetson Family]], [[Raspberry Pi]])  
- **Pre-ISP conditioning**: Mux, crop, scale, simple color space ops before ISP/NN accelerator  
- **Automotive ADAS**: Camera fan-in/out, display split/duplication for redundancy and diagnostics :contentReference[oaicite:5]{index=5}

---

## 📊 Comparison Chart — Lattice Siblings (vision focus)

| Family | Hardened D-PHY | USB (Hard PHY) | Typical LCs | Notable Use |
|---|---|---:|---:|---|
| **CrossLink-NX** | **2× 4-lane (TX/RX), up to ~10 Gb/s per PHY** | No | 17K–40K | Camera aggregation/bridging |
| **CrossLink (orig.)** | 2× 4-lane (bridge-centric) | No | ~6K | Legacy camera/display bridges |
| **CrossLinkU-NX** | 2× 4-lane + soft D-PHY | **USB 2.0 + USB 3.2 Gen 1 (hard)** | 33K | USB cameras/bridges + MIPI |
| **CertusPro-NX** | No hard MIPI (use soft) | No | Up to 100K+ | General purpose, SERDES-heavy |

**Notes**: CrossLink-NX emphasizes embedded MIPI at low power; **CrossLinkU-NX** adds **hard USB 3.2**; “orig. CrossLink” is older bridge silicon. Specs summarized from Lattice product pages and datasheets. :contentReference[oaicite:6]{index=6}

---

## 🧩 Comparison Chart — Against common low/mid FPGAs

| Device | Hard MIPI D-PHY | Soft D-PHY feasible | Power (qualitative) | Camera Aggregation Fit |
|---|---|---|---|---|
| **CrossLink-NX (Lattice)** | **Yes (2× 4-lane)** | Yes (HP I/O) | Very low | **Excellent** (native) |
| **ECP5 (Lattice)** | No | Yes | Low | Good (soft IP, tighter margins) |
| **Spartan-7 / Artix-7 (AMD)** | No | Yes | Moderate | Good (needs soft IP/ext PHY) |
| **Cyclone 10 LP (Intel)** | No | Yes | Moderate | Good (soft IP) |
| **PolarFire (Microchip)** | No (usually ext PHY) | Yes | Low-mod | Good (heavier dev effort) |
| **GOWIN GW2A/GW1N** | No | Yes | Low | Fair/Good (soft IP maturity varies) |

**Takeaway**: CrossLink-NX’s **hardened** D-PHY reduces risk and power for MIPI-heavy designs vs families that rely on soft PHY or external transceivers. :contentReference[oaicite:7]{index=7}

---

## 🧯 Comparison Chart — Multi-camera strategies

| Strategy | How it works | Pros | Cons | When to choose |
|---|---|---|---|---|
| **CrossLink-NX VC Aggregation** | Merge multiple CSI-2 inputs to one output via **Virtual Channels** | Single cable/port, time-aligned frames, flexible | FPGA dev needed | When AP lanes/ports are scarce |
| **CrossLink-NX Mosaic/Tiling** | Stitch images (e.g., side-by-side) into one larger frame | Simple downstream processing | Pixel resampling/crop logic | Fixed layouts, bandwidth math |
| **MIPI Switch/Multiplexer** | Time-slice cameras onto one link | Simple hardware | Not simultaneous; pipeline artifacts | Non-concurrent capture tolerated |
| **Many-lane AP (e.g., big Jetson)** | Connect each camera directly | Simplifies design | Cost, power, connector count | If budget/power allow |

**Notes**: Lattice provides VC aggregation and 4:1 examples; time-sliced muxes can cause skewed capture timing (seen in some quad-cam boards). :contentReference[oaicite:8]{index=8}

---

## 🛠️ How It Works (Sensor Aggregation)

1. **CSI-2 RX** on hardened D-PHY blocks receives multiple streams  
2. **Align & Tag**: Frames buffered/retimed; assign **Virtual Channels** or spatially compose  
3. **Optional ops**: RAW10→RGB888, resize, crop, basic ISP stubs  
4. **CSI-2 TX** outputs one stream to AP (or **HDMI/DSI** bridge path if required) :contentReference[oaicite:9]{index=9}

---

## 🧪 Real-World Projects & Reference Designs

- **Lattice 4-to-1 Image Aggregation (CrossLink-NX)**: 4× CSI-2 in → single output; includes HDMI demo; VC-based approach. :contentReference[oaicite:10]{index=10}  
- **Lattice CSI-2 Virtual Channel Aggregation**: Generic N:1 VC aggregator IP/guide. :contentReference[oaicite:11]{index=11}  
- **CircuitValley USB3 MIPI CSI-2 RX (CrossLink-NX + FX3)**: Open HDL; tested with IMX219/IMX477. :contentReference[oaicite:12]{index=12}  
- **CSI-2 Image Simulator to Jetson Nano** (CrossLink): Emulates an IMX219 over CSI-2; great learning resource. :contentReference[oaicite:13]{index=13}  
- **Conference/Partner demos**: Multi-sensor aggregation and video bridging with CrossLink-NX. :contentReference[oaicite:14]{index=14}

---

## 🏆 Strengths

- **Native MIPI**: Less power/effort vs soft D-PHY; up to **~10 Gb/s per D-PHY** interface (2.5 Gb/s per lane, 4 lanes) and soft D-PHY possible on HP I/O for extra links. :contentReference[oaicite:15]{index=15}  
- **Low power, small footprint**: Well-suited to drones/robots/cameras; Nexus 28 nm FD-SOI. :contentReference[oaicite:16]{index=16}  
- **Robustness**: Lattice reports very low SER for the class. :contentReference[oaicite:17]{index=17}  
- **Good ecosystem** for camera/display bridging (reference designs, IP).

---

## ⚠️ Limitations

- **Resource scale**: 17–40 K LC class; not for huge DSP/NN workloads  
- **Lane count**: Two hard D-PHY blocks; more links require soft D-PHY or external parts  
- **Ecosystem**: Smaller 3rd-party IP pool vs large-vendor FPGAs (but improving)

---

## 🧰 Developer Tools

- **Lattice Radiant** (synthesis/P&R), IP for CSI-2/DSI, VC aggregation, HDMI bridges  
- **Eval HW**: CrossLink-NX Evaluation Board (FMC, PMOD, Raspberry Pi header, PCIe edge) for rapid bring-up. :contentReference[oaicite:18]{index=18}  
- **CrossLinkU-NX Eval**: Adds hardened USB 2.0/3.2 Gen 1 for camera-over-USB designs. :contentReference[oaicite:19]{index=19}

---

## 🧷 Variants & Notable Devices

- **LIFCL-17 / LIFCL-40**: Classic CrossLink-NX density points (≈17 K / 40 K LCs)  
- **LIFCL-33 / LIFCL-33U**: 33 K LC variant; “U” adds **USB 2.0/3.2 Gen 1 hard PHY** and Always-On features. :contentReference[oaicite:20]{index=20}

---

## 🔌 Compatible / Complementary Items

- [[CSI-2]], [[D-PHY]], [[C-PHY]], [[DSI]], [[HDMI]], [[USB 3.0]], [[FX3]] (Cypress), [[GMSL]], [[FPD-Link III]], [[Jetson Family]], [[Raspberry Pi]], [[Sensors]] (image), [[SBCs]]

---

## 📚 External Resources

- **Product page** (family overview, low power claims) and **datasheets** (CrossLink-NX / LIFCL-33/33U)  
- **Reference designs**: **4-to-1 Aggregation**, **CSI-2 VC Aggregation**  
- **Evaluation boards**: CrossLink-NX EVN (FMC, PMOD, RPi), CrossLinkU-NX EVN  
- **Community projects**: CircuitValley CSI-2 RX + USB3 (FX3), CSI-2 simulator to Jetson Nano  
- **Context articles/videos**: Automotive embedded vision with CrossLink-NX; Lattice talks/demos on MIPI/aggregation  
:contentReference[oaicite:21]{index=21}

---

## 📝 Related Notes / See Also

- [[CSI-2]] (Camera Serial Interface 2)  
- [[D-PHY]] (MIPI physical layer for CSI-2/DSI)  
- [[C-PHY]] (Alternative MIPI physical layer)  
- [[SLVS-EC]] (High-speed sensor interface alternative)  
- [[GMSL]] (Gigabit Multimedia Serial Link)  
- [[FPD-Link III]] (Automotive serializer/deserializer)  
- [[FX3]] (Cypress USB 3.0 controller)  
- [[Jetson Family]] (NVIDIA SoCs)  
- [[SBCs]] (Single-Board Computers)  
- [[HDMI]] (Display interface)

---

## 🧾 Summary

**CrossLink-NX** is the go-to **MIPI-centric, low-power FPGA** for robotics and embedded vision when you need deterministic **sensor aggregation/bridging** without the power/size penalty of larger FPGAs. It offers hardened D-PHY, tiny packages, and ready-to-use aggregation reference designs, making it a pragmatic way to feed multi-camera rigs into resource-limited SoCs. :contentReference[oaicite:22]{index=22}
