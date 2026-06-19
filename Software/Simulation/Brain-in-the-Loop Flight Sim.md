# Brain-in-the-Loop Flight Simulation

This note is a practical walkthrough for a `brain in the vat` flow: a real autopilot stack reads simulator telemetry (IMU/Pitot/GPS-like signals) and sends actuator/surface/RPM commands back, while still running as if it were in flight.

The goal is to cover both software-first (`ArduPilot`-based) and hardware (`Pixhawk`-based) paths, on both Linux and Windows, with enough concrete settings and commands to execute.

---

## 🎯 What this page connects

- `[[ArduPilot vs Pixhawk]]` for abstraction: stack vs board framing
- `[[X-Plane]]` for high-fidelity visuals/atmospheric model
- `[[Flight-Sim RL Contract]]` for RL environment design
- `[[ROS2 Aerial Integration]]` when offboard brains are running in ROS

You should read this note as a loop-level map, not a full flight safety guide.

---

## 🧠 Core loop: brain-in-the-vat architecture

| Loop side | Role | Common protocol | Typical example |
|---|---|---|---|
| Simulator (`X-Plane`, `jMAVSim`, etc.) | Generates physics + env + sensor data | Custom UDP/TCP bridge | X-Plane sends position/velocity/attitude + `49001` stream |
| Autopilot (`ArduPilot` or `PX4` running on `Pixhawk`) | Runs control stack and mode logic | MAVLink streams | `SERVO_CONTROL`, attitude/controller updates |
| Ground/Companion | Logging, setpoint policies, rewards, telemetry | MAVLink/MAVSDK/ROS2 | Puffer-style step loop bridge |

A complete brain-in-the-loop system needs deterministic timing and explicit bridges for both ways:

- simulator -> autopilot: accelerometers, gyro, magnetometer, pitot/airspeed, GPS-like position
- autopilot -> simulator: servo/pwm commands, control mode state, arm/disarm intent

In ArduPilot/PX4 this is usually done as multiple UDP/TCP links, where one link is the estimator/control traffic and another is telemetry/logging.

---

## 🛠 ArduPilot-only setup (Linux-first)

This is the most direct and documented path for X-Plane coupling.

### 1) Linux prerequisites

```bash
git clone --recursive https://github.com/ArduPilot/ardupilot.git
cd ardupilot
./Tools/environment_install/install-prereqs-ubuntu.sh -y
```

Then open a new shell so environment variables are active.

### 2) Build/prepare SITL

```bash
./waf configure --board sitl
./waf plane
```

For generic launch and options:

```bash
./Tools/autotest/sim_vehicle.py --help
```

Set up X-Plane networking first before launching.

### 3) Configure X-Plane (Linux and Windows both)

In X-Plane 11 (`Settings -> Data Output`):

- Enable at least one data output used by `ArduPilot`
- UDP rate: `50.0`
- Check `Send network data output`
- Set destination IP to the ArduPilot host
- Destination UDP port: `49001`

If you are on one host for both simulator and SITL, this can be `127.0.0.1`.

### 4) Run SITL + X-Plane

```bash
sim_vehicle.py -v ArduPlane -f xplane --console --map
```

- SITL is reachable by GCS over TCP `5760`
- If you use Mission Planner / MAVProxy, you can switch to `--map` for the integrated map output
- On Windows hosts, if using WSL, point the X-Plane UDP destination to the Windows-side IP that WSL sees, then run SITL in the Linux side

To find that IP from PowerShell:

```powershell
ipconfig.exe
```

Look for the `vEthernet (WSL)` interface; that host IP is the target for cross-OS routing.

---

## 🖥️ ArduPilot on Windows (SITL)

You can do this with or without compiling in Linux, depending on how custom you need to get:

### Path A — mission-planner simulation mode

1. Install Mission Planner on Windows
2. In X-Plane, set output to host IP/port expected by Mission Planner bridge
3. In Mission Planner simulation tab:
   - select model `xplane`
   - select `Plane` (legacy path is Plane-oriented)
   - start sim link
4. Connect at TCP `5760`

This is the quickest path for pilot-in-the-loop testing.

### Path B — compile in WSL2 then bridge back to Windows

1. Install Ubuntu in WSL2 (recommended)
2. Build ArduPilot SITL in WSL (same Linux steps above)
3. Launch `sim_vehicle.py` inside WSL with your project directory and board config
4. Point X-Plane network data fields to the WSL address from Windows
5. Connect Mission Planner or MAVProxy on Windows host to the right exported socket

In practice, this path is better for code-level autopilot changes and stable reproducibility compared to native Windows-only flows.

---

## 🧱 Pixhawk + ArduPilot: two practical hardware modes

`Pixhawk` is a hardware family, not a flight stack. These are common patterns for `brain in the vat`.

### A) Simulation-on-hardware (same FC runs sim code)

ArduPilot documents this as a supported workflow where simulation is run inside the FC firmware path.

#### Linux/Windows build and upload flow

Linux build path example (adapt board/vehicle/frame values):

```bash
./Tools/scripts/sitl-on-hardware/sitl-on-hw.py \
  --board CubeOrange-SimOnHardWare \
  --vehicle plane \
  --simclass QuadPlane \
  --frame quadplane-tilttri \
  --defaults ./Tools/autotest/default_params/quadplane-tilttri.parm \
  --upload
```

- `SIM_OPOS_LAT`, `SIM_OPOS_LNG`, `SIM_OPOS_ALT`, `SIM_OPOS_HDG` let you seed start position/heading in sim
- This path preserves real FC timing but has known parameter-sharing constraints when switching between real flight and simulation profiles

### B) Legacy HIL via Mission Planner bridge (older, Plane-only in docs)

ArduPilot’s archived X-Plane HIL page outlines a Mission Planner-centered workflow with explicit serial and port wiring:

- X-Plane UDP in: mission bridge output
- X-Plane-to-APM default direction: around `49005` input / `49000` output in the archived guidance
- Mission Planner serial + `115200` baud connect
- Select **APM Simulation** mode and start the loop

This remains useful for quick regression and human-in-the-loop validation, but it is explicitly described as archived/legacy.

### Recommended split

- If you need **control development and code changes**: prefer Linux build + simulation-on-hardware
- If you need **quick Plane-only benching**: legacy Mission Planner flow can still be used when available

---

## 📦 Pixhawk + PX4: when you want `Pixhawk` hardware but not ArduPilot

PX4 does provide hardware in the loop support, but in PX4 docs this is centered on jMAVSim/Gazebo rather than X-Plane.

### Linux path

- Flash PX4 firmware to Pixhawk via QGroundControl
- Select a HITL airframe (`SYS_AUTOSTART`) compatible with your vehicle class
  - `1001` (HIL Quadcopter X)
  - `1002` (HIL Standard VTOL QuadPlane)
- Configure simulator in QGC and bridge MAVLink to FC over USB/UART
- Verify UDP/GCS ports (`14550` for GCS, `4560` for simulator local connection in SITL-style architecture)

### Windows path

- Build/run companion sim in WSL or Linux host if needed
- Flash custom binaries with QGC in Windows (required for USB/serial access)
- Keep GCS on Windows and simulator on Linux/WSL if performance is acceptable

PX4 HITL is best for autopilot-level validation, but it is generally not the primary `X-Plane` bridge path.

---

## 🌐 iNav: related but not equivalent

`iNav` can be used with X-Plane in experimental workflows, mostly through community channels.

A concrete community command example:

```bash
inav_SITL --sim=xp --simip=192.168.0.162 --simport=49000 --chanmap=S01-01,S02-02,S03-03,M01-04
```

That indicates `iNav` users are doing sensor/RC channel passthrough style loops, but this remains project/community driven rather than a polished X-Plane stack like ArduPilot-SITL docs.

If your goal is official support and repeatability, prioritize ArduPilot/PX4 for the main loop and treat iNav as an alternate stack.

---

## 🧩 Comparison quick sheet (how to pick)

| Stack | Stack software | Hardware target | Linux setup friction | Windows setup friction | Best use |
|---|---|---|---|---|---|
| ArduPilot SITL + X-Plane | ArduPilot | none required | Medium | Low with Mission Planner | Rapid control-stack iteration |
| ArduPilot HIL on hardware | ArduPilot | Pixhawk/Cube-like boards | Medium-High | Medium-High | Timing-accurate FC-in-the-loop testing |
| ArduPilot Mission Planner HIL | ArduPilot | Plane-only legacy path on compatible boards | Low (via WSL only workaround) | Medium | Quick bench validation |
| PX4 HITL (jMAVSim/Gazebo) | PX4 | Pixhawk-family boards | Medium | Medium (QGC flashing from Win) | Modern autopilot/HITL validation |
| iNav SITL experiments | iNav | many FCs, community dependent | Medium | Medium | Optional comparator stack |

---

## ✅ Common failure points

- IP mismatch between WSL and host: autopilot sends sensor data to the wrong interface
- Port mismatch: X-Plane input/output ports are not mirrored correctly (`49000` vs `49001` / `49005` varies by mode)
- Serial driver behavior on Windows: autopilot appears, disappears, or stays locked at wrong COM
- Mode mismatch: fixed-wing workflows in legacy HIL are often Plane-only
- Parameter drift: switching real-flight and sim-mode params without backup/reset causes confusing pre-arm behavior

For ArduPilot sim-on-hardware, save params/missions first; in docs, params are often reset via defaults flow and reloaded after sim.

---

## 🔗 Related notes

- [[ArduPilot vs Pixhawk]]
- [[X-Plane]]
- [[Flight-Sim RL Contract]]
- [[ROS2 Aerial Integration]]
- [[PufferLib Flight Sim Integration Gaps]]

---

## 📚 Sources

- ArduPilot SITL docs: https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html
- ArduPilot X-Plane SITL docs: https://ardupilot.org/dev/docs/sitl-with-xplane.html
- ArduPilot Simulation on Hardware: https://ardupilot.org/dev/docs/sim-on-hardware.html
- ArduPilot HITL simulators index: https://ardupilot.org/dev/docs/hitl-simulators.html
- X-Plane archived HIL page (legacy): https://ardupilot.org/dev/docs/x-plane-hardware-in-the-loop-simulation.html
- Mission Planner firmware upload to Pixhawk: https://ardupilot.org/planner/docs/common-loading-firmware-onto-pixhawk.html
- ArduPilot Mission Planner install: https://ardupilot.org/planner/docs/mission-planner-installation.html
- PX4 HITL (official): https://docs.px4.io/main/en/simulation/hitl
- PX4 hardware simulation overview: https://docs.px4.io/main/en/simulation/hardware
- PX4 port reference and WSL note: https://docs.px4.io/main/en/dev_setup/dev_env_windows_wsl.html
- iNav discussion with XP simulation + command: https://github.com/iNavFlight/inav/discussions/9246
- iNav config issue thread: https://github.com/iNavFlight/inav/issues/9247
