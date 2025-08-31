## 1) What MultiCam actually does (and what it uses)

Function: Takes 2–4 [[MIPI CSI-2 Protocol]] camera inputs and produces one CSI-2 output. Arducam markets both “virtual-channel merge” (multiple streams over one link) and stitching (combine into a single, larger frame). Their docs and listings explicitly mention “combine four cameras into one frame.”

Device on board: Multiple independent reports and vendor Q&A indicate MultiCam uses a Lattice CrossLink family FPGA (purpose-built for MIPI bridging/aggregation). Lattice themselves promote a 4-to-1 CSI-2 aggregator reference design that does either virtual channels or side-by-side stitch.

Throughput example (one MultiCam variant): “Input up to 1 Gb/s per lane, output 2-lane up to 1 Gb/s/lane.” That’s ~2 Gb/s total on the output for some kits, which is fine for 4× 1MP@30 but not enough for 4× 1080p@30.

Takeaway: the [[CrossLink]]/[[CrossLink-NX]] [[FPGA]] route is the proven architecture for this job.

## 2) Feasibility & bandwidth for your target (4× 1080p30 mono)

Quick math (per entire 1×4 frame):

1080p = 1920×1080 = 2.07 MP per camera

4 cameras → ~8.29 MP per stitched frame

At 30 fps, RAW8 ≈ 1.99 Gb/s, RAW10 ≈ 2.49 Gb/s, RAW12 ≈ 2.99 Gb/s pixel data only; add ~20–30% for CSI-2 packet/blanking overhead. Realistic link budget: ~2.5–3.8 Gb/s depending on bit depth.

So you’ll want a 4-lane CSI-2 output at ≥1 Gb/s/lane (ideally 1.5–2.5 Gb/s/lane headroom). Many Jetson carriers offer a 4-lane port; Raspberry Pi 4/CM4 typically exposes 2-lane per connector (CM4 has two ports but they aren’t mergeable into one input), which will bottleneck you at this resolution/fps. Jetson Nano supports MIPI CSI-2 up to 1.5 Gb/s per lane, with x2/x4 configurations (depends on carrier board). 
OpenZeka | NVIDIA Embedded Distribütörü
pbrobinson.fedorapeople.org

Bottom line: For 4× 1080p30 mono, plan on Jetson (Nano/Xavier/Orin) with a 4-lane camera port. A Pi might work if you drop resolution/bit-depth/fps significantly or move to Pi 5 + custom tricks, but it’s tight given the 2-lane limit on typical connectors. 
pip.raspberrypi.com
Raspberry Pi
Geizhals

## 3) Sensor choices (global shutter with hard sync)

If 1080p is a must: Look at onsemi AR0234 (1920×1200 GS, up to 120 fps, MIPI CSI-2, supports external trigger). It’s widely integrated for Jetson and Pi by vendors. 
onsemi
sunnywale.com
e-con Systems

If 1MP is enough: [[OV9281]] (1280×800 GS) is ubiquitous and also supports hardware frame sync (FSIN). Easy mode for proving out the FPGA and synchronization first.

For hard sync, prefer sensors with an FSIN/trigger pin. OV9281/OV2311/AR0234 all support external trigger/sync; Arducam documents this for Pi/Jetson use. 
docs.arducam.com

## 4) System architecture (recommended)

```
[ Master Sync/Trigger ]
           |
       (fanout)
           |---------------------> FSIN to all 4 sensors
           |
        [FPGA w/ D-PHY RX x4]  <-- 4x CSI-2 (2 lanes each typical)
             |  RAW unpack + line buffers + stitch (1×4)
             v
        [FPGA D-PHY TX x1]  --> single CSI-2 (4 lanes) to host
             |
           I²C mux/bridge <--> control all 4 sensors + FPGA
             |
        [Host: Jetson]  ----> sees ONE camera @ (7680×1080) or (4×1920 wide)
```

FPGA: Lattice CrossLink or CrossLink-NX parts have hardened MIPI [[D-PHY]] RX/TX and ship with a “4:1 CSI-2 Video Aggregator” reference design that can stitch side-by-side or combine via virtual channels. You customize the “tile-by-tile” compositor in fabric.

Control plane: One I²C path to program the FPGA plus a [[TCA9546A]]/[[TCA9548A]]-type I²C mux to reach four identical sensors (same default address).

Clocking: Provide a low-jitter 24–27 MHz MCLK to each sensor (use a fanout buffer), and a clean oscillator for the FPGA. Use the sensors’ FSIN for exposure start alignment (one sensor can be “master” or a small MCU/FPGA GPIO can be the strobe).

Output format: Advertise a single RAW8/10 mode with width = 4×single-cam width (e.g., 7680×1080 @ 30 fps). That keeps the host drivers simple.

## 5) Why not ESP32 (or most MCUs)?

They don’t have a MIPI CSI-2 D-PHY. Even if you managed a parallel DVP hack, 4×1080p30 is far beyond MCU bandwidth and memory. You need CSI-2 PHYs and SERDES-class throughput, i.e., an FPGA/ASIC built for MIPI.

## 6) Step-by-step build plan
**A. Up-front decisions**

Pick your host.

[[Jetson Family]] Jetson Nano/Xavier/Orin with a 4-lane camera connector is ideal. Check your carrier: many expose a 4-lane port; NVIDIA’s Nano dev kit connectors are 2-lane each, but third-party carriers (e.g., Auvidea JN30) support 4-lane. 
pbrobinson.fedorapeople.org
auvidea.eu

Pi 4/CM4 typically gives you 2-lane per port (CM4 has two ports, both 2-lane). That’s a hard ceiling for your target throughput. 
pip.raspberrypi.com
static.raspberrypi.org

Pick your sensor module.

AR0234 if you need true 1080p GS; OV9281/OV2311 for lower-risk bring-up. Confirm FSIN is broken out on your module. 
onsemi

Choose the FPGA.

Lattice CrossLink-NX (e.g., LIFCL-40) or classic CrossLink (LIF-MD). Reference designs exist for 4-to-1 CSI-2 aggregation/stitching.

**B. PCB design (high-speed highlights)**

MIPI lanes: 100 Ω differential, tight intra-pair length matching (±5–10 mil typical), keep inter-pair skew small, minimize stubs/vias, short runs, solid reference plane. Follow MIPI D-PHY layout guidance (trace spacing, return paths, ESD).

Connectors: 22-pin/15-pin FFC for camera inputs; one 4-lane FFC to the host. Keep connector-to-FPGA runs as short and symmetric as possible.

Power: Separate analog/digital rails for sensors (AVDD/DVDD/IOVDD), quiet LDOs for PLL/analog domains. Bulk + high-freq decoupling near each device.

Clocking: Low-jitter oscillator(s), clock fanout to sensors, impedance-controlled routes.

I²C mux & GPIOs: A TCA9546A/9548A to reach four identical sensors; dedicated reset lines per sensor help during bring-up.

Trigger nets: Route FSIN as matched single-ended lines from your trigger source to all sensors.

**C. FPGA/firmware work**

CSI-2 RX x4: Use Lattice CSI-2 RX IPs to receive four RAW streams.

De-packetize RAW8/10/12 into pixel+line timing; line buffers/FIFOs per input to handle small skew.

Stitcher: Horizontally concatenate the four lines (A|B|C|D) → produce one wider line. Keep per-line buffering only (no full-frame RAM needed), which keeps FPGA BRAM usage modest.

CSI-2 TX x1: Re-packetize to one output stream. Program timing as if it were a single sensor at 4× width.

I²C bridge/control: Soft I²C master in FPGA (or small MCU) to push sensor register sequences, trigger mode, exposure, gain, etc.

Sync: Either drive FSIN from the FPGA (one GPIO → 4 fanouts) or designate one sensor as master that outputs a strobe others slave to (depends on sensor features). OV9281/AR0234 both support external sync. 
docs.arducam.com

Optionally: Allow a “debug VC mode” where you pass the four cameras as separate virtual channels over one link (Jetson can ingest multi-VC). This helps verify each path before enabling the stitcher. Lattice’s reference covers VC aggregation.

**D. Sensor register bring-up**

MCLK & PLL setup → streaming OFF.

Program common mode (bit depth, blanking, link rate) on all four.

Enable external trigger/FSIN mode, same exposure/integration time, same gain/black-level. (AR0234/OV9281 have documented external-trigger modes.) 
onsemi

Turn on streaming, verify that all four assert frame start within a few µs of each other.

**E. Host (Jetson) driver & device tree**

Simplest host model: Present your board as one sensor with a custom DT entry (one CSI-2 port, 4 lanes, RAW8/10, resolution = (4×W)×H). Jetson supports 4-lane CSI-2 up to 1.5 Gb/s per lane on Nano, higher on Orin. 
OpenZeka | NVIDIA Embedded Distribütörü

Frameworks: [[V4L2]] [[subdevice]] + [[nvcsi]] + [[libargus]] (Jetson). If you ever run multi-VC mode, Jetson can expose multiple /dev/video nodes. (NVIDIA’s docs & community posts cover nvcsi/DT plumbing.) 
Embien

Pi path: Libcamera/V4L2 on Pi expects a single sensor stream; bandwidth may cap you to lower modes due to 2-lane connectors. 
pip.raspberrypi.com

**F. Performance/optimizations**

Bit depth: If 10-bit is enough for your application, use RAW10 to save ~20% bandwidth vs RAW12.

Lane rate: Run output at the highest stable lane rate your host tolerates (Jetson Nano can do up to 1.5 Gb/s/lane; Orin supports much more). 
OpenZeka | NVIDIA Embedded Distribütörü
NVIDIA Developer

Crop/scale in sensors if you don’t need full FOV — less pixels → less bandwidth.

DMA packing: Keep MIPI packetization at RAW10/RAW8 to reduce host unpack overhead.

Latency: Pure line-buffer stitching keeps latency to “a few lines,” essentially instantaneous for robotics.

## 7) Alternative architectures (when you don’t need literal stitching)

Virtual channels only (no stitch): Send four streams over one 4-lane link using CSI-2 VCs; let Jetson capture four nodes and tile in CUDA. Much less FPGA logic; highest flexibility. (CamArray variants do this.)

[[GMSL2]]/[[FPD-Link]] to MIPI: If cameras are far away, 4× serializers → one deserializer with quad-to-MIPI output (e.g., MAX96712 family) and then either VC merge or stitch in an FPGA. Good for long cable runs, but adds cost/complexity.

## 8) Rough BOM (for a 4×1080p30 GS prototype)

FPGA: Lattice CrossLink-NX (LIFCL-40/17) with CSI-2 RX/TX IP.

Sensors: 4× AR0234 (or start with 4× OV9281 to prove sync path). 
onsemi

I²C mux: TI TCA9546A/9548A.

Clocking: 1× low-jitter XO for FPGA; 1× 24/27 MHz XO + fanout for sensors.

Power: Rail set for sensors (AVDD/DVDD/IOVDD), FPGA core/IO rails, plenty of bypassing.

Connectors: 4× 22-pin FFC in; 1× 22-pin out (4-lane).

ESD: Low-cap ESD diodes on MIPI pairs near connectors.

Misc: A few GPIOs for per-sensor RESET, TRIGGER (FSIN), status LEDs, [[JTAG]] header.

If you don’t want to spin a board immediately, you can prototype with a CrossLink(-NX) eval kit + MIPI FMC/mezzanine and a Jetson carrier with 4-lane CSI (Auvidea/J20x/JN30) to validate the RTL first. 
auvidea.eu

## 9) Bring-up checklist

One camera → one output through the FPGA (bypass mode).

Add cameras one at a time; verify each lane mapping, deskew, data type (RAW10).

Enable external trigger on two, then four; scope FSIN vs. frame start. 
docs.arducam.com

Turn on stitcher (side-by-side). Confirm final timing/resolution.

Host DT: expose the stitched mode (e.g., 7680×1080@30). Use [[v4l2-ctl]] --stream-mmap --stream-count=100 smoke tests.

Validate throughput at target lane rate; watch for FS errors (frame start) and CSI bit error counters.

Calibrate exposure & gains to match all quadrants; optional per-tile black-level offset.

## 10) Flowcharts (high-level)

Data flow (RX → stitch → TX):
```
[CSI-2 RX0]   [CSI-2 RX1]   [CSI-2 RX2]   [CSI-2 RX3]
   |              |             |             |
 [Unpack]      [Unpack]      [Unpack]      [Unpack]
   |              |             |             |
 [Line FIFO]   [Line FIFO]   [Line FIFO]   [Line FIFO]
        \         |           |          /
          ------ [1×4 Stitch (A|B|C|D)] -----> [CSI-2 TX] --> Host
```

Control & sync:
```
                 [I2C Master in FPGA] --(via TCA9548A)--> [4x Sensors]
                              |
[Trigger Source / FPGA GPIO] -+---> FSIN to all sensors
```
## 11) Practical notes & traps

Address conflicts: identical sensors often share the same I²C address → use an I²C mux.

Lane mapping: MIPI is polarity sensitive; keep pair polarity and order consistent.

Skew: Mind sensor cable length differences; that’s why each input needs a small line FIFO.

Thermals: Four sensors + FPGA in a tight space get warm; plan airflow or a small heatspreader.

Host caps: Jetson Nano dev kit camera sockets are 2-lane each; use a carrier that exposes a 4-lane port for your stitched stream. 
pbrobinson.fedorapeople.org

Pi limits: Many Pi docs show 2-lane CSI and ~800 Mb/s/lane nominal; that caps stitched modes unless you downscale/deframe. 
Microchip
