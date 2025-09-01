# Camera Sensor Aggregators — MIPI CSI-2 (CamArray / CrossLink / MultiCam)

Camera **sensor aggregators** (aka camera bridges, multi-camera aggregators, sensor mux/merge boards) let you connect more image sensors to a host with a limited number of MIPI CSI-2 ports. They appear in robotics and edge-vision systems when you need more cameras than the SoC provides, or when you need hardware-level synchronization across multiple sensors (e.g., 4 × OV9281 global-shutter cameras). This note focuses on off-the-shelf options (and the reference designs behind them) that work with **CSI-2 ribbon cables** used on Raspberry Pi (CM / RPi 5 family) and NVIDIA Jetson boards, plus practical integration tips for robotics projects.

---

## ⚙️ Quick TL;DR

- **Aggregator vs multiplexer:** a *multiplexer* switches/selects one camera at a time. An *aggregator/bridge* merges or stitches multiple CSI-2 streams (via MIPI Virtual Channels or side-by-side stitching) into one CSI-2 output the host sees as a single camera. Use aggregators for simultaneous capture; use multiplexers when sequential sampling is sufficient.  
- **Best off-the-shelf pick for 4× OV9281 synchronized capture:** Arducam CamArray / Quadrascopic kits and some e-con Systems / Connect Tech multi-camera solutions. Lattice CrossLink-NX is the common FPGA backbone for custom aggregator designs.  
- **Interfaces:** look for “4 × 2-lane” or “4 × 4-lane” physical inputs and a single CSI-2 output (or a carrier board with multiple CSI connectors). Jetson/CM connectors expect CSI-2 ribbons or custom coax/GMSL solutions with serdes.  
- **When you need GMSL / FPD-Link instead:** long cables, PoC (power over coax), automotive-grade links — vendors like Connect Tech and e-con provide GMSL platforms that guarantee simultaneous frames over long runs.

---

## 🧭 Core Concepts (short)

- **MIPI CSI-2 Virtual Channels (VC):** lets multiple sensor streams share the same physical CSI link by tagging packets with a VC ID; aggregation can be performed by the bridge/FPGA.  
- **Side-by-side stitch vs VC merge:** stitch merges pixel data to form a larger frame; VC merge sends interleaved packets and requires host or ISP to demultiplex.  
- **Hardware sync (frame-sync / trigger):** to get truly simultaneous exposures, pick sensors with a hardware sync/trigger pin (OV9281 supports sync) **and** use an aggregator/hat that either passes the sync or drives a shared clock.  
- **PHY lanes:** sensors can be 1/2/4-lane. Ensure your aggregator and host lanes (and bandwidth) match your resolution & FPS need.

---

## 🔬 How it works — typical aggregator block

1. Each camera → MIPI D-PHY receiver on aggregator FPGA (or serdes receiver for GMSL).  
2. Pixel/line/frame alignment + optional ISP/byte-to-pixel handling.  
3. Virtual channel tagging or side-by-side packaging.  
4. Single CSI-2 transmitter to host (or multiple outputs when supported).  
5. Control plane: I²C/CCI pass-through or aggregator acts as a camera hub (camera registers accessible through the bridge).  

---

## 📋 Comparison Chart — common off-the-shelf options

| Option (type) | Interface (host) | Sync | Typical max cams | OV9281-ready? | Est. price (single-unit) | Power notes | Short pros | Short cons |
|---|---:|:---:|---:|:---:|---:|---:|---|
| **[[Arducam CamArray]] / Quadrascopic** (aggregator kit) | MIPI CSI-2 ribbon (RPi, Jetson) | Yes — synchronized 4-ch mode | 4 | Yes (Arducam sells 4× OV9281 kits). | Quad kit ~$200–$350 (kits vary). | Low; 4× OV9281 ≈ ~0.8W reported (hat + sensors). | Plug-and-play kits, targeted docs, inexpensive for prototyping. | RPi/Jetson driver/kernel quirks; signal integrity & cable length constraints. |
| **Lattice [[CrossLink-NX]] (reference / custom boards)** (aggregator IP) | Build for CSI-2 ribbons | Yes (VC or stitch; design dependent) | 2–8 (depending on part) | Yes (if you wire sensors & firmware) | IC + board: varies — custom board cost + dev cost. | Very low FPGA power; reference demos <1.5W for 1080p setups. | Flexible: stitch/VC/duplication; low power; production suitable. | Requires FPGA work or vendor board / IP license integration. |
| **Leopard Imaging 4× MIPI adapter (carrier)** (carrier/adapter) | Breaks host into 4 MIPI CSI connectors | Depends (pass-through) | 4 | Yes (pins & lanes) | LI-JXAV-MIPI-ADPT-4CAM listed ≈ $169 (board only) | Low (passive adapter) | Simple carrier for Jetson boards; low cost. | Not a synchronized aggregator — host must accept multiple ports or you need aggregator logic. |
| **e-con Systems multi-camera kits** (synchronized cameras + baseboard) | MIPI CSI-2 or custom baseboard | Yes — synchronized multi-camera kits | 2–4 (or more for carrier boards) | Yes (they offer OV9281 variants and many sensors) | Quad kits: a few hundred to ~€300–€800 depending on sensor resolution | Moderate (sensors + ISP + cables) | Robust, engineered for Jetson; ISP + driver support. | More expensive; OEM shipping times & configuration options. |
| **Connect Tech / GMSL platforms** (serializer/deserializer) | SerDes (coax) → baseboard → Jetson | Yes — designed for simultaneous capture over coax | 4–8 (platform dep.) | OV9281 via specific modules (or similar) | Platform ~€400+; cameras per ~$199 (sample) | Higher per-camera (PoC, serializer) but supports long cable runs | Long-cable, automotive-grade, PoC support, guaranteed sync. | Higher cost & complexity; different physical layer (coax) vs CSI-2 ribbon. |
| **Arducam Multi-Camera Adapter (multiplexer)** (mux) | Single CSI-2 port | No — usually sequential (switches) | 4 | Yes (but not simultaneous) | <$50–$80 (adapter board) | Very low | Cheap, simple when simultaneous capture is NOT required. | Not suitable for simultaneous capture or high-precision stereo. |

*(sources cited below — Arducam, Lattice CrossLink docs, Leopard Imaging, e-con, Connect Tech product pages and community forums)*

---

## ✅ When to pick which

- Use **Arducam CamArray** or **e-con** if you want *out-of-the-box* 4-camera synchronized capture with CSI-2 ribbons on RPi / Jetson (good for prototyping & robotics).  
- Use **Leopard Imaging** carriers if your Jetson has extra lanes and you just need to bring more physical connectors to the carrier board (not an aggregator).  
- Use **Lattice CrossLink-NX** (reference design or custom board) when you need compact low-power custom aggregation (side-by-side stitching or VC aggregation) for production.  
- Use **GMSL/FPD-Link** (Connect Tech / e-con solutions) if you need **long cable lengths / PoC / automotive** robustness and guaranteed sync across distance.

---

## ⚖️ Pros / Cons (aggregators in general)

Pros
- Simultaneous capture across many sensors (when combined with sensor sync).  
- Host sees one logical device — simpler application layer if aggregator stitches frames.  
- Low-latency pass-through options exist (especially with CrossLink).

Cons
- Complexity: device tree + driver + libcamera adjustments on Jetson / RPi.  
- Bandwidth limits: watch lane count, resolution, framerate and host ISP/DMAs.  
- Signal integrity & cable length: CSI-2 ribbons are short; long runs need serdes (GMSL).  
- Heat & power: many sensors + aggregator + SoC processing can increase system power.

---

## 🔎 Synchronization (getting *all* 4 frames at the same time)

Checklist:
- Choose sensors with **hardware frame sync / trigger** support (OV9281 does).
- Ensure aggregator either *passes* the sync or *drives* it; CamArray default synchronized mode handles sync for supported modules.
- Confirm host ISP/driver will accept the aggregated stream (some aggregators present stitched frames as one big camera; others present multiple virtual channels). Lattice CrossLink reference designs support both VC merge and side-by-side stitching; choose the method your ISP can handle.
- If you need absolute deterministic exposure timing across sensors (for stereo depth or motion capture), use an external hardware trigger and confirm the sensors’ exposure timing and readout latency match.

---

## 🔌 Physical & Software integration notes (Jetson & Raspberry Pi 5)

- **Jetson boards:** check lane availability — Orin/NX boards commonly expose up to 8 lanes (so two 4-lane cameras or four 2-lane cameras). Use carrier/adapters that match lane counts.
- **Raspberry Pi 5 / Compute Module:** Pi5 has dual CSI connectors (and new smaller connectors) — Arducam provides adapter/config docs for RPi 5 compatibility; CamArray and Arducam adapters have worked on RPi family with config changes.
- **Drivers:** expect to work with libcamera/picamera2 on Raspbian; on Jetson you’ll work with the L4T device tree + V4L2 bindings and vendor drivers for complex aggregator modes (some vendors provide kernel patches).

---

## 🔎 Price & Power (estimates & ranges)

- **Arducam Quad OV9281 bundle**: commonly listed ~$200–$350 for 4× kit (camera modules + CamArray HAT). Exact kit price varies by sensor (mono vs color) and reseller. Power: users report ~0.8 W for 4× OV9281 + HAT (sensors are ~0.16 W each); aggregator/FPGA add only small overhead.
- **Leopard Imaging LI-JXAV-MIPI-ADPT-4CAM**: board only ≈ $169 (adapter/carrier price seen on product page). Passive adapter — low power.
- **e-con synchronized 4-camera kits**: range from a few hundred to >€800 depending on sensor resolution and features (their high-res kits are pricier).
- **Connect Tech GMSL platforms**: board/platform costs often €400+, plus per-camera module (~$199 sample price for some NileCAM modules). GMSL adds serializer power but gains long-cable & PoC benefits.
- **Custom CrossLink-NX solution**: upfront dev cost + board BOM; production BOM varies, but CrossLink-NX is positioned as low-power — plenty of demos show <1.5 W in certain video pipelines.

---

## 🧰 Developer Tools & SW ecosystem

- **Reference IP / Designs:** Lattice CrossLink-NX offers N:1 MIPI CSI-2 camera aggregator reference designs (VC merge / stitch) and source packages. Good starting point for custom boards.
- **V4L2 / libcamera / device tree:** you’ll likely need device tree edits, custom driver overlays, or vendor kernel modules on Jetson/RPi. Check vendor docs first (Arducam / e-con / Leopard provide guides).
- **FPGA dev:** CrossLink-NX evaluation boards and IP bundles (Lattice Diamond / Nexus toolchain) are the path for bespoke aggregation features.

---

## 🧭 Use-case examples

- **Robotics perception rig (4 × OV9281 monochrome global-shutter):** Arducam CamArray kit for rapid prototyping; synchronize frames for stereo/depth algorithms.
- **Industrial inspection (4 × high-res sensors at 60+ FPS):** e-con Systems synchronized kit or custom CrossLink-NX aggregator feeding Jetson AGX Orin / Xavier with sufficient bandwidth.
- **Autonomous vehicle / long-cable cameras:** GMSL/FPD-Link with Connect Tech or OEM GMSL boards; PoC + guaranteed sync over meters of coax.

---

## 🔗 Related notes / concepts

- [[Kinvert MultiCam]]  
- [[CrossLink-NX]] (reference IP & aggregator designs)  
- [[Arducam CamArray]]  
- [[Leopard Imaging]]  
- [[e-con Systems]]  
- [[GMSL]] (serializer/deserializer camera links)  
- [[MIPI CSI-2 Protocol]]  
- [[OV9281]] — global shutter 1MP sensor

---

## 📚 External resources (starter list)

(see **Sources** below for direct vendor pages, reference designs and docs)

---

## ✅ Quick checklist before buying

1. Confirm **host CSI lanes** / ISP capability.  
2. Match **sensor lanes** (2-lane OV9281 is common) and aggregator lane output.  
3. Confirm **synchronization method** — hardware trigger vs virtual channel stitching.  
4. Check vendor **driver support** for your OS and JetPack / RPi OS version.  
5. Consider cable length — otherwise prefer GMSL for long runs.  
6. Budget for power & thermal headroom on host board (Jetson boards already run hot).

---
