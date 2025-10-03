# DroneForge Nimbus 🌀

A developer-focused overview of **DroneForge Nimbus** (the Nimbus SDK / Nimbus app) as a platform for building autonomous / pilotless drone systems — emphasising radio/receiver compatibility (ELRS / CRSF / Crossfire / others), integration with small whoops like the **BETAFPV Air65**, OS/runtime support (Ubuntu vs vendor OS), telemetry/range/latency characteristics, and practical compatibility notes for engineers.

---

## 🔎 Overview
DroneForge "Nimbus" is an SDK + desktop app for building, testing and deploying AI-enhanced drone behaviors. It explicitly targets ELRS (ExpressLRS) radio links and CRSF/serial integration, provides real-time video panels with AI overlays (YOLO, monocular depth), a node-based visual flow (NimbusMap), telemetry/link statistics, and an 8-channel RC interface for control/override. The SDK ships releases/ AppImage builds for Linux and desktop platforms and exposes serial port (CP210x) integration for connecting physical hardware.

---

## 📝 Summary
- **Primary integration:** ExpressLRS / CRSF (full support shown in the Nimbus README).
- **Channels:** UI shows 8-channel display (ROLL, PITCH, THROTTLE, YAW + AUX) and supports channel hardcoding/override.
- **OS/Run modes:** Nimbus provides Linux AppImage releases and instructions to run on typical Linux machines (e.g., Ubuntu) — connecting over USB (CP210x) or serial. In short: **yes, it runs on standard Ubuntu** (AppImage / .sh launcher usage suggested in README).
- **Target hardware:** ELRS-enabled receivers and flight controllers using CRSF/SBUS/serial outputs; small drones like the BetaFPV Air65 (which commonly ships with integrated Serial ELRS/CRSF RX) are explicitly compatible in practice.

---

## ⚙️ Core Concepts
- **ELRS (ExpressLRS):** open-source radio link (2.4GHz / 900MHz) supporting FLRC/LoRa modes, extremely configurable packet rates (up to 1000Hz in 2.4GHz FLRC), telemetry and CRSF serial output for flight controllers. Nimbus uses CRSF framing to talk to the receiver/FC.
- **CRSF (Crossfire serial protocol):** popular serial transport used by Crossfire and supported as the receiver→FC serial output by many ELRS receivers — low-latency channels + telemetry packets. Nimbus explicitly notes CRSF integration.
- **Telemetry & Control Flow:** Nimbus collects RSSI, SNR, link quality, and channel values; it can overlay AI outputs on FPV and also send channel overrides to the FC (i.e., autopilot/closed-loop control).

---

## 🔬 How it works (high level)
1. **Physical link:** an ELRS receiver on the drone receives radio link from a compatible TX (ExpressLRS module / TX or TBS Crossfire etc), then outputs serial (often CRSF) to FC/OSD stack. Nimbus listens to CRSF packets via a USB serial interface (e.g., CP210x from a connected module or via a flight-controller passthrough). 
2. **Telemetry & channels:** ELRS transmits channels + telemetry; the FC can provide additional telemetry via MSP/MAVLink which ELRS can pack into telemetry bursts. Nimbus shows link stats and can log them.
3. **AI + control:** Nimbus runs vision stacks (ONNX/YOLO) locally, computes target/positioning, and writes channel overrides or higher-level setpoints to the FC via CRSF / serial; it also visualises video and depth overlays.

---

## 📡 Compatible radios & receivers (practical list)
> **Primary**: ExpressLRS-compatible receivers (ELRS Nano, ELRS Nano VTX combos, ELRS-enabled 5-in-1 boards), because Nimbus explicitly targets ELRS and CRSF.

- **ExpressLRS (ELRS) receivers** — ELRS Nano, ELRS R9 variants, many modern ELRS RX boards. These provide CRSF serial output or SBUS/other outputs (selectable / hardware dependent). ELRS supports multiple output protocols (CRSF, SBUS, SUMD, SmartAudio, etc).
- **Betaflight-style 5-in-1 FCs with Serial ELRS** — e.g., the BetaFPV Air brushless FC integrates a *Serial ELRS 2.4G RX* that outputs CRSF to the FC. Nimbus can interface with such setups (via serial passthrough or direct USB).
- **TBS Crossfire / CRSF receivers** — Crossfire uses CRSF for FC telemetry/control; Nimbus supports CRSF-level integration, so CRSF receivers are usable where serial is available. Note Crossfire TX modules are proprietary hardware; Crossfire and ELRS both can produce CRSF for FCs but are different radio links. 
- **Other protocols** (less direct): you can convert other RX outputs (PWM/SBUS/FPort) to serial/CRSF on the FC side — Nimbus depends on serial CRSF visibility for many of its UI features. ELRS receivers can often be configured to output SBUS or CRSF depending on hardware.

---

## 🧭 Supported drone sizes / examples
- **Whoops / micro (28–75 mm wheelbase):** BetaFPV Air65 (65 mm) — integrated ELRS receiver and tiny 5-in-1 FC make it straightforward to use Nimbus for higher-level AI/telemetry and channel overrides. Nimbus's focus on ELRS/CRSF and small-device telemetry suits micro/whoop form factors where integrated serial RX is present. 
- **Racing / Freestyle micros and minis (3–5 inch):** Any FC that exposes CRSF via a serial port and connects to an ELRS/Crossfire RX will be compatible. Nimbus expects a serial-visible CRSF stream (or passthrough mode) to provide live channel values.
- **Larger platforms ( > 5 inch / long-range):** As long as the airframe contains an ELRS/Crossfire receiver that outputs CRSF to the flight stack, Nimbus can interface for high-level commands/telemetry. For heavy autopilot demands, Nimbus is best used as a companion/control plane rather than replacing a certified autopilot.

---

## 📶 Range / Bandwidth / Latency — practical expectations
> These values are determined primarily by the radio link (ELRS or Crossfire) and hardware configuration, not by Nimbus itself.

- **ExpressLRS (ELRS):**
  - **Latency:** configurable by packet rate. ELRS supports very high refresh (up to 1000 Hz on some 2.4 GHz setups). In practice, pilots run 50–500 Hz depending on range/conditions; higher packet rates lower latency. ELRS community reports sub-5ms to few-ms link latencies at high packet rates in optimal setups. 
  - **Range:** extremely variable — with high-gain antennas and high-power modules, experienced users report 10s of kilometers in ideal conditions; ELRS emphasizes both long-range and high refresh flexibility. Realistic hobbyist ranges: hundreds of meters to many kilometers depending on antenna/tx power/regulatory limits.
  - **Telemetry bandwidth:** ELRS exposes configurable telemetry bursts (LINK vs DATA) and shares bandwidth with MSP/MAVLink if used. Bandwidth is sufficient for channel transport and periodic telemetry; advanced telemetry allocation is configurable.
- **TBS Crossfire:**
  - **Latency:** historically very low and consistent; marketed for "ultra-low latency" with high telemetry bandwidth. Crossfire typically has slightly higher base-latency than high-rate ELRS modes but remains low enough for precise control. Practical latency figures vary (~few ms to tens of ms depending on mode/config).
  - **Range:** designed for very long range (many pilots report multiple kilometers to tens of kilometers with high power modules/antenna setups).

> **Bottom line:** Nimbus inherits whatever latency/range your radio link provides. For low-latency closed-loop AI control (fast maneuvers), run ELRS at a high packet rate (333–1000Hz where supported) and ensure the FC/OS path latency (serial/processing) is minimized.

---

## 🔗 Signals sent in & out (protocols & payloads)
- **Inbound to Nimbus (observed):**
  - CRSF channel packets (channel values, stick positions).
  - Telemetry packets (RSSI, SNR, link stats, possibly FC telemetry forwarded via ELRS telemetry).
  - FPV video stream (Nimbus expects a live video feed for AI overlays — typically via capture device).
- **Outbound from Nimbus (control / commands):**
  - CRSF channel overrides (writing channel values) and possibly direct serial commands to FC via MSP/MAVLink if configured. Nimbus documents channel hardcoding and override features.
- **Other:** ELRS telemetry can encapsulate Flight Controller telemetry (MSP/MAVLink) via passthrough; Nimbus can surface these stats in its telemetry panels.

---

## ✅ Strengths (where Nimbus shines)
- Tight ELRS/CRSF integration (native support for link metrics and channel streaming).
- Built-in AI pipelines (YOLO, depth estimation) and visual node-based programming (NimbusMap) for non-trivial autonomy experiments.
- Desktop-centric developer UX (AppImage on Linux, serial port picker, live overlays, data export).
---

## ⚠️ Weaknesses / Caveats
- **Early-stage / proprietary maturity:** repo shows early releases; API docs are "work in progress". For production-grade autonomy, validate safety/failsafe behavior independently.
- **Radio dependence:** the functional capabilities (latency, telemetry richness) are highly dependent on the chosen TX/RX hardware and settings (ELRS vs Crossfire etc). Nimbus cannot improve raw RF physics.
- **Limited community-project examples (so far):** few public third-party projects currently reference Nimbus by name — expect to prototype and debug integration yourself.

---

## ⚖️ Pros / Cons (concise)
- **Pros:** ELRS-first SDK, real-time AI overlays, Linux desktop AppImage, CRSF channel control, telemetry/logging.
- **Cons:** Early docs/API maturity, dependency on serial passthrough for some features, limited known community integrations/examples.

---

## 🧩 Variants & Releases
- Releases include Linux AppImage GPU-enabled builds; check the GitHub Releases for the latest AppImage and release notes (e.g., Nimbus v0.2.0). Nimbus repo contains `droneforge-crsf-service` and other support components.

---

## 🔧 Developer Tools & Installation notes
- **Prereqs:** OpenCV 4.x, ONNX Runtime (for AI models), CUDA/toolkit for GPU acceleration where applicable. Nimbus README emphasizes these.
- **Linux run:** download AppImage or run bundled script; README suggests `./Droneforge-Nimbus-<version>.AppImage` or run the `droneforge_nimbus.sh` launcher on Linux/Ubuntu. Use a proper data-capable USB-C cable and select CP210x serial at 115200 for device port.
- **Connecting to an FC / RX:** If using a BetaFPV Air65 or similar, enable serial passthrough or connect to the FC UART configured for CRSF/MSP. Many modern BetaFPV boards expose the Serial ELRS RX directly to FC.

---

## 🧪 Examples / Projects found
- **Official repo samples / releases:** Nimbus SDK repo includes example assets and a `droneforge-crsf-service`. Releases show "run the AppImage; plug in Nimbus USB-C; select CP210x" style instructions (see repo releases).
- **Third-party adoption:** as of searches, public third-party projects referencing Nimbus SDK are limited; mostly the project is early and community examples are scarce — you may need to prototype. (No extensive third-party project list found in public search results.)

---

## 📚 External resources (recommended reading & references)
- Droneforge Nimbus SDK (official repo & README) — explains CRSF/ELRS integration and provides Linux AppImage instructions.
- ExpressLRS main site & FAQ — details on packet rates, modes (FLRC/LoRa), telemetry, and what ELRS provides (latency, range). Essential to tune link settings for Nimbus use.
- ExpressLRS telemetry & serial protocols page — details on CRSF and receiver output modes. Useful to understand serial options for FC integration.
- BetaFPV Air65 product and Air 5-in-1 FC pages — shows that the Air65 ships with Serial ELRS RX (CRSF output) on the integrated FC. Good practical example for small-whoop integration.
- TBS Crossfire product pages & comparisons — for design decisions if you plan to use Crossfire instead of ELRS.

---

## 🔭 Further reading & suggested experiments
- **Verify hardware flow:** get a BetaFPV Air65 (or any ELRS-equipped FC) and practice binding to your ExpressLRS TX, observe CRSF packets in Betaflight/CLI, then connect Nimbus and watch link stats.
- **Latency tests:** run ELRS at different packet rates (100/250/500/1000 Hz) and measure end-to-end latency from stick input → CRSF → FC → channel output, and compare with Nimbus measured input→CRSF packet latency panel. Use the Nimbus input latency metric to validate.
- **Telemetry bandwidth tuning:** experiment with ExpressLRS telemetry burst settings if you need heavier telemetry (MAVLink/MSP passthrough) while maintaining control update rates.

---

## 🔗 Related Concepts / Notes
- - [[ELRS]] (open-source RC link)  
- - [[CRSF]] (Crossfire serial protocol)  
- - [[BETAFPV Air65]] (example micro airframe with Serial ELRS)  
- - [[Betaflight]] (flight controller firmware that will show CRSF channels)  
- - [[MAVLink]] (when using autopilots or MAVSDK passthrough telemetry)  
- - [[FPV Video Capture]] (hardware needed for Nimbus video overlays)  
- - [[ONNX]] (model format Nimbus supports)  
- - [[YOLO]] (object detection model family used by Nimbus)  

---

## ✅ Key takeaways (quick)
- Nimbus is ELRS/CRSF-first: **best compatibility** is with ExpressLRS receivers (ELRS Nano / serial ELRS) and any FC that exposes CRSF or a serial passthrough.
- **Runs on Ubuntu/Linux** via AppImage / shell wrapper; it does not require a proprietary "Omarchy" OS — you can run on standard Ubuntu desktops that meet prerequisites (OpenCV, ONNX runtime, drivers).
- **Works well with BetaFPV Air65** and similar whoops that include integrated Serial ELRS RX outputs; bigger platforms are supported provided they have ELRS/CRSF serial visibility to Nimbus.

---

## 🧾 Notes about safety & productionization
- Nimbus can send channel overrides — **test in a safe environment** and implement robust failsafes on the FC (geofencing, arming checks, hardware kill) before any untethered/autonomous flights. Always comply with local UAV regulations.

---
