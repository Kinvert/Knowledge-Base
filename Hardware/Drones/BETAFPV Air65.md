# BETAFPV ELRS Air65 — Deep Dive (Air65)

The **BETAFPV Air65 (ELRS)** is a modern ultra-light 65 mm whoop designed for FPV fun, racing, and as a tiny research/learning platform. The Air65 ships with a highly-integrated “Air” 5-in-1 flight controller (FC+ESC+OSD+VTX+Serial-ELRS RX), runs Betaflight firmware out of the box, and is available in different motor/prop configurations (Racing, Freestyle, Champion). This note is a practical, engineer-focused reference: how to get one flying, what radios work, how to approach autonomy (ROS2 / PX4 / ArduPilot), how to use PufferLib for training, how to integrate with DroneForge Nimbus, and realistic upgrade paths for “self-flying” experiments.

---

## ⚙️ Quick TL;DR (what you can do with an Air65)
- **Out of the box:** Fly FPV with ExpressLRS (ELRS) radio control and Betaflight’s flight modes (Angle, Horizon, Acro). Great for manual or assisted flights.
- **Autonomy options (realistic):**  
  1. *Lightweight experiments* — vision-based/human-in-the-loop approaches by streaming video to a ground computer and sending RC stick commands (or using ELRS to carry commands).  
  2. *Serious autonomy* — swap to a PX4/ArduPilot-compatible autopilot (or add a companion computer + MAVLink-capable autopilot) to run missions/ROS2 stacks. PX4/ROS2 has robust offboard interfaces for autonomy.
- **Radio:** ELRS transmitter or module required to use integrated ELRS receiver; you can also wire in an external receiver (SBUS/SRXL/SPEKTRUM) to the FC if you prefer another radio ecosystem.

---

## 🧭 What’s in the Air65 (hardware & firmware)

**Key hardware (typical ELRS Air65 BNF / AIO variants):**
- Frame: Air65 ultralight canopy/frame (≈ 65 mm wheelbase).
- Flight Controller / ESC: BetaFPV **Air Brushless Flight Controller (G4 / STM32G473)** 5-in-1 AIO (FC + 1S ESCs + integrated OSD + integrated VTX + *Serial ELRS receiver* on the 5IN1 variant). ICM42688P gyro, STM32G473 MCU, 16 MB Blackbox. Runs **Betaflight** (BetaFPV recommends Betaflight 4.x builds for Air G4).
- Camera: C03 (or similar micro FPV) and onboard VTX (25–400 mW selectable).
- Radio link: integrated **Serial ELRS 2.4 GHz** (ExpressLRS) on the 5IN1 variant; firmware V3.x in many shipping units (bind phrase/firmware must match TX). The serial ELRS receiver talks CRSF over UART to the FC.

---

## 🛒 Minimal shopping / parts checklist (to get flying right away)
- Air65 (ELRS 5-in-1 variant) — BNF (bind-and-fly) for quick start.
- ELRS TX: a radio with built-in ELRS or an external ELRS TX module (examples: RadioMaster with ELRS module, Jumper / Radiomaster radios, LiteRadio 2 SE with ELRS option). See ExpressLRS quick-start for module wiring/flashing.
- FPV goggles / monitor that accept analog 5.8 GHz input (if using analog VTX)  
- Batteries: 1S Z-folding or micro LiPo (as specified by BetaFPV for flight time)  
- Charger and small toolkit (precision screwdriver, soldering iron optional)  
- Optional: a micro-USB cable, Betaflight Configurator on your laptop (Windows/Mac/Linux) for setup and tuning.

---

## 🔌 Radios & ELRS — what you need to know

### Do you need a radio to fly it?
Yes — the Air65 includes an ELRS serial receiver (5IN1 variant) so **an ELRS transmitter (TX)** or TX module is required for RF control. If you don’t have an ELRS TX you must either:
- attach an external receiver compatible with your existing radio (via one of the FC UARTs: SBUS/CRSF/SBUS-in), or  
- replace/disable the internal ELRS and add a serial Spektrum/SBUS receiver, or  
- get an ELRS TX module for your existing radio.

### Which radios / modules work?
- **ExpressLRS (ELRS)** is the native link (2.4 GHz or 915/868 MHz options). Popular choices: RadioMaster family (TX16S, TX12) with ELRS modules, Jumper T-Pro/T-Lite with ELRS module, LiteRadio 2 SE (BETAFPV), Radiomaster Ranger, TBS CRSF/Tango 2 (Crossfire) alternatives if you swap RX. ExpressLRS docs and community strongly recommend using matched ELRS versions and binding phrases.
- **Spektrum radios**: you **can** use a Spektrum radio *with the Air65* — but not by binding it to the Air65’s integrated ELRS RX. Instead you would add a small Spektrum-compatible receiver (SRXL/SRXL2/DSM-satellite) and wire it to a spare UART/SBUS input on the Air FC, then configure Betaflight to use that receiver. Spektrum also has tiny serial receivers (SRXL2/serial micro) that connect over a single data wire and are supported by flight stacks that accept serial receivers. See vendor docs for wiring and binding details.

---

## 🛠️ First-time setup (practical step-by-step)

1. **Unpack & inspect**: look for the 5IN1 FC variant if you want built-in ELRS; check canopy and connectors.
2. **Charge battery** and bench-power the Air65 for the first time to verify VTX and camera power up. Use a bench strap for props OFF testing.  
3. **Bind ELRS**: power the quad on/off 3× (or follow BetaFPV bind procedure) and put your ELRS TX in bind mode. BetaFPV mentions power-cycling for internal serial ELRS bind mode. You can also update ELRS receiver firmware via Wi-Fi (some BetaFPV units) or through Betaflight serial passthrough + ExpressLRS Configurator.
4. **Connect to Betaflight Configurator**: plug micro-USB, open Betaflight Configurator (Desktop), confirm `Receiver` tab shows channels (set `Serial` + provider `CRSF` for the integrated ELRS). BetaFPV recommends specific Betaflight versions for Air G4 family (see BetaFPV support).
5. **Set flight modes**: configure an arm switch, Angle (self-level) for safe flights, and a failsafe config. Do a simple bench test of motors and ESC direction.  
6. **PID / rates**: start with BetaFPV recommended CLI file/firmware settings (they provide .hex and recommended CLI). Fine-tune later outdoors.

---

## 🧩 Will it work with DroneForge Nimbus & how?

**Short answer:** *Yes — but with conditions.* DroneForge Nimbus is positioned as a hardware-agnostic SDK/platform for deployable drone AI; their SDK and product pages indicate integrations with ELRS and common flight stacks. For practical integration you have two common paths:

1. **Companion-computer + PX4 (recommended path for autonomy & Nimbus)**  

2. **Radio/ELRS command injection (edge case)**  
   - If you want to keep the tiny Air65 hull and still run high-level autonomy, you can run Nimbus agents on a ground or companion computer and send control by emulating RC inputs (e.g., inject stick commands into an ELRS TX module or use a telemetry/command channel). This is doable but fragile and latency-sensitive; it’s a pragmatic hack for very light tiny whoops that cannot carry a companion computer. Use only for low-risk experiments.

**Practical steps** (high level):
- Decide whether you’ll **(A)** add PX4/ArduPilot autopilot and run Nimbus / ROS2 offboard via MAVLink, or **(B)** keep Betaflight & use an external computer to send RC commands into an ELRS TX module. Option A is cleaner for robust autonomy.

---

## 🧠 How to make it “fly itself” — realistic options (ranked)

### 0) Beginner / lightweight (no hardware change)
- Use Betaflight self-level (`Angle`) and tune `Rates/PID`. Not true autonomy — good for safety and assisted flight.

### 1) Vision-assisted / ground-based autonomy (practical for tiny whoops)
- Run a perception stack on a ground computer (YOLO, optical flow), estimate target/pose, send *stick commands* from the ground over ELRS (or over a telemetry radio). This lets you implement simple following behaviours w/o adding weight. Latency and safety are big caveats.

### 2) Companion computer + offboard (recommended for research)
- Replace or augment FC with a PX4-capable autopilot (or add a companion computer that connects to a PX4 via MAVLink). Run **ROS2** + perception/planning (or DroneForge Nimbus) on the companion computer, use **MAVROS / MAVSDK** to command offboard setpoints. This is the standard research stack for real autonomy.

### 3) Full autopilot path (best for robust autonomy)
- Use a proper autopilot (Pixhawk / CUAV / Holybro) on a larger frame, use PX4/ArduPilot for GPS/RTK/mission planning, and connect ROS2 via MAVROS or px4-ros2 bridge. This is how industry and serious research teams do autonomous missions.

---

## 🔁 PufferLib + Air65 — what is feasible & how to integrate

**PufferLib** is an RL/experiment tooling and training library (excellent for simulation training). Typical workflow to use PufferLib with a drone:

1. **Train in simulation** — use PufferLib with an environment that simulates your drone dynamics (Gym/Gazebo/IsaacSim). Train a policy (e.g., observation → throttle/roll/pitch/yaw or higher-level velocity commands).
2. **Sim2Real transfer** — use domain randomization, dynamics randomization, and closed-loop tests in hardware-in-the-loop.  
3. **Deployment** — export the model (`TorchScript`/ONNX) and run it on a companion computer. The companion sends offboard setpoints to PX4 (via MAVROS/MAVSDK) or generates RC stick values and pushes them into the radio. For Air65 specifically, you must either (A) provide a companion computer on the drone (hard due to weight), (B) run the policy on a ground laptop and stream commands, or (C) move to a slightly larger frame that can carry a Jetson/NVIDIA board for onboard inference.

**Bottom line:** PufferLib is excellent for the *training/simulation* side — deployment requires careful hardware choices.

---

## 🔄 Flight controller comparison: Air-FC (G473 & Betaflight) vs Pixhawk / ArduPilot / PX4

| Characteristic | Air G473 (Betaflight) | Pixhawk / PX4 / ArduPilot |
|---|---:|---|
| Typical use | FPV, acro, high-rate attitude control | Autonomy, navigation, mission planning |
| Firmware/OS | Betaflight (BF 4.x recommended for Air G4). Lightweight, tuned for low-latency acro flight. | PX4 / ArduPilot — full autopilot stacks with MAVLink, mission planner, GPS/RTK, failsafes.
| Serial receiver protocol | CRSF/Serial ELRS to FC (UART) on 5IN1 version. | Supports many serial/sbus/srxl receiver types; direct MAVLink telemetry radios supported |
| Offboard / ROS2 support | Very limited (not native MAVLink); possible via hacks/RC injection | Native / first class (MAVROS, ROS2 bridges, offboard, uORB topics). |
| Weight & integration | Ultra-light (good for tiny whoops) | Heavier, larger (needs larger frames) |
| Recommendation | Best for FPV + low-weight experiments | Best for autonomy research and production systems |

---

## 📊 Comparison chart — Air65 vs peers (whoops + autonomy choices)

| Model / Class | Type | Weight (nominal) | FC (native) | Native Autonomy fit | Good for learning autonomy? |
|---|---:|---:|---:|---:|---:|
| **BETAFPV Air65 (ELRS)** | 65 mm whoop | ~17 g (no battery) | Air G4 5IN1 (G473) + Serial ELRS | Low — Betaflight (no MAVLink). Requires swaps for full autonomy. | Good for perception experiments; poor if you need onboard MAVLink autonomy |
| **BetaFPV Meteor65 / Meteor65 Pro** | 65 mm whoop | ~22–23 g | Meteor F4 AIO (older, heavier) | Similar to Air65; some variants have different FCs; limited autonomy without changing FC. | Same class — good for FPV / edge experiments |
| **Happymodel Mobula6** | 65 mm whoop | ~17–20 g | SuperX ELRS AIO 5IN1 | Similar—built-in ELRS; small, not PX4-friendly natively. | Good for tiny whoop experimentation |
| **3" Autonomy dev kit (Pixhawk on 3" frame)** | 3" multirotor (DIY) | 200–800 g | Pixhawk / Cube + companion | Excellent — PX4/ArduPilot + MAVROS for ROS2 | Strongly recommended if you want to learn real autonomy (vision, path planning, ROS2). |
| **Commercial autopilot drone (Matrice / DF Fleet)** | Enterprise | 2–10 kg | Built for autopilot stacks | Best — enterprise tools for mission planning & SDKs | If budget allows, best for end-to-end autonomous fleet learning (DroneForge offers fleet/SDK solutions). |

---

## ✅ Strengths (Air65)
- Extremely light & agile — superb low-inertia flight characteristics for learning control and perception.
- Integrated ELRS — modern, low-latency link good for frequent control updates.
- Low cost and widely available — good for large fleets / many learning trials.

## ❌ Weaknesses (Air65)
- Betaflight + tiny FC architecture makes **true autonomous missions (MAVLink, GPS, mission planning) difficult** without hardware changes.
- Very little payload capacity — companion computers (Jetson/USB boards) are usually too heavy to mount without changing the frame.  
- Limited GPS/positioning capability in a micro whoop form factor.

---

## 🔧 Upgrade & modification paths (how to get autonomy)

1. **Add a companion computer on a larger frame** (Jetson Nano/Xavier NX, Raspberry Pi 4): run ROS2 + perception and connect to a PX4 autopilot (via MAVLink) to control vehicle. This is the canonical research setup.
2. **Swap the FC to a PX4/ArduPilot compatible board** (if a physical replacement exists for the Air65 footprint — usually not; better is to move to a slightly larger frame that accepts a Pixhawk/Cube). Use PX4 for navigation + MAVROS for ROS2 integration.
3. **Use ground-side autonomy + RC injection**: run autonomy on a ground PC and stream RC channel commands into your transmitter (or implement a small UART->radio link). This keeps the tiny hull but is less robust.  
4. **Optical flow / VIO + tiny autopilot**: research teams sometimes use optical-flow sensors (PX4Flow style) + VIO stacks for GPS-less indoor position hold — but this is heavy on sensors and calibration.

---

## 🧾 Practical setup notes & gotchas
- **ELRS versions matter**: receiver and TX should run compatible major ELRS releases and the same regulatory domain/bind phrase. BetaFPV often ships RX with a default ELRS V3 build — update via ExpressLRS Configurator if needed.
- **Serial receiver binding on internal ELRS**: BetaFPV documents entering bind mode by cycling power 3× and using Betaflight passthrough to flash RX firmware if required.
- **Betaflight vs MAVLink**: don’t expect Betaflight to be a drop-in for MAVLink/ROS2 mission workflows. It’s optimized for human-piloted FPV; autonomous research workflows favor PX4/ArduPilot.

---

## 🔗 Related notes (put these in your vault)
- [[ExpressLRS]] (ELRS, CRSF wiring, flashing)  
- [[Betaflight]] (Configurator, CLI, flight modes)  
- [[PX4]] (autopilot, ROS2 offboard)  
- [[ArduPilot]] (autopilot alternative)  
- [[ROS2]] (MAVROS, mavsdk, offboard)  
- [[PufferLib]] (RL training & sim → sim2real workflows)  
- [[DroneForge Nimbus]] (Nimbus SDK & integrations)  
- [[Meteor65]]  
- [[Mobula6]]  
- [[Pixhawk]] (family & who to use for autonomy)

---

## 📚 External resources / further reading
- BetaFPV Air65 product page and specifications.
- BetaFPV support: Air-FC firmware & Betaflight recommendations (firmware downloads & CLI files).
- ExpressLRS quick-start and wiring (CRSF serial receiver wiring to FC).
- DroneForge Nimbus SDK (official repo / docs — hardware-agnostic SDK for drone AI).
- PX4 + ROS2 integration guide (offboard / companion computer workflows).
- PufferLib docs and paper (training in sim; sim2real workflows).
- https://www.amazon.com/BETAFPV-Brushless-Quadcopter-Ultralight-Controller/dp/B0DQPP4DR8/

---

## 🏁 Key takeaways / recommended path if your goal = **self-flying farm drone swarm**
1. **Prototype perception/control in simulation** (PufferLib + Gazebo/IsaacSim) and iterate quickly.
2. **Use a platform that supports MAVLink & companion computer** (Pixhawk/Cube on 3" or larger frames) for realistic ROS2 + PX4 integration — this is the standard production/research path.
3. **Use Air65 / other tiny whoops for low-weight vision/edge prototyping** (perception, low-latency experiments, swarming ideas) — but accept the hardware limits for carrying a computer or running PX4.
4. **If you must keep tiny whoop frames**: implement autonomy by ground-side compute + RC injection, or consider ultra-light companion devices (experimental) — but expect reliability limitations.

---

## ✅ Quick checklists (copy/paste friendly)

**Initial bench setup**
- `Install Betaflight Configurator`  
- `Connect Air65 via USB` → validate FC recognized  
- `Receiver tab` → set `Serial` mode and `Serial Receiver Provider = CRSF` (for internal ELRS)  
- `Modes` → add Arm + Angle (safe)  
- `PID` → load BetaFPV recommended CLI if provided

**To add ROS2 autonomy (recommended minimal stack)**
- Get a PX4-compatible FC (Pixhawk family) + companion computer (Ubuntu 22.04 + ROS2 Humble or later)  
- On companion: install `mavros` / `mavsdk` and connect via serial/UDP to autopilot  
- Use ROS2 nodes for perception/planning and publish offboard setpoints (see PX4 ROS2 offboard examples).

---

## 🧭 Final notes (practical judgment)
The Air65 is an **excellent tiny FPV platform** — light, fun, and modern (ELRS). For **research/production autonomy** (ROS2, mission planning, robust offboard control), you will either need to (A) move to a larger frame that accepts a Pixhawk/Cube and companion computer, or (B) accept brittle, ground-based tricks for commanding the tiny whoop. Use Air65 for perception and control experiments where weight matters; use Pixhawk/PX4 for full autonomy and ROS2 integration.
