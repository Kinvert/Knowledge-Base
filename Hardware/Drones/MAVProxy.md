# MAVProxy

`MAVProxy` is a lightweight, modular command-line GCS for MAVLink systems (`ArduPilot`-focused by default in ecosystem practice, but usable with any MAVLink-capable autopilot).

It is built for scriptable operator workflows, stream-forwarding, and custom module loading.

---

## 🧭 What it is and what it is not

- `MAVProxy` is a full operator layer, but intentionally not a large desktop GUI.
- Think of it as:
  - an interactive CLI shell
  - a stream router
  - a mission and parameter command surface
  - a module/plugin host
- If you need a mission editor first and scripting second, `[[QGroundControl]]` is usually the easier start.

---

## 🧩 Installation and platform setup

### Windows

- Install from the provided `MAVProxySetup-latest.exe` package and invoke via `mavproxy.exe`.
- Docs also call out WSL as a common alternative for better stability in some Windows environments.

### Linux (Debian/Ubuntu family)

Typical user path:

- Install Python deps for `wx` (or skip wx if headless).
- `python3 -m pip install PyYAML mavproxy --user`
- Ensure user binaries are in path (often `~/.local/bin`).
- Add serial permissions: `sudo usermod -a -G dialout <username>`
- For headless deployment, GUI packages like wx can be omitted.

### MacOS

- Install Python and required package dependencies (`wxPython`, `gnureadline`, `billiard`, `numpy`, `pyparsing`) via `pip` in a matching package environment.

All major environments can update with:
- `python3 -m pip install mavproxy pymavlink --user --upgrade`

---

## ⚡ Quickstart connection paths

MAVProxy can autodetect a single USB/autopilot connection when only one serial endpoint exists.

Minimal examples:

- USB Linux: `mavproxy.py --master=/dev/ttyUSB0`
- USB Windows: `mavproxy --master=COM14`
- Network input: `mavproxy.py --master=udp:127.0.0.1:14550`

If multiple radios or redundant links are present, pass multiple `--master` entries.

For serial baud overrides:
- `mavproxy.py --master=/dev/ttyUSB0 --baudrate=57600`

To run non-interactively with UI modules:
- `mavproxy.py --master=/dev/ttyUSB0 --console --map --load-module graph`

---

## 🧱 Startup options that matter in real workflows

Important startup flags often used in day-to-day operations:

- `--master` (required if multiple endpoints)
- `--out` for forwarding telemetry to downstream GCS tools
- `--out=udp:127.0.0.1:14550` (common local relay into another station)
- `--out=COM17,57600` / `--out=/dev/ttyACM0,57600` for serial onward relays
- `--sitl=127.0.0.1:5501` for SITL RC/source injection
- `--streamrate` for MAVLink packet cadence
- `--source-system`, `--target-system`, `--source-component`
- `--aircraft <name>` and `--mission <name>` to structure log folders
- `--console` and `--map` for GUI modules on demand
- `--load-module <name>` for specific add-ons
- `--non-interactive` + `--daemon` for automation-style sessions

Multi-link and forward patterns:

- For local GCS relay with autopilot on Linux:
  - `mavproxy.py --master=/dev/ttyACM0 --baudrate 115200 --out 127.0.0.1:14550`
- Then connect another GCS client to UDP `127.0.0.1:14550`.
- TCP forwarding is possible, but docs caution about joystick responsiveness under TCP latency in forwarding scenarios; UDP is common for low-latency control forwarding.

---

## 🧩 Modules and command surface

`MAVProxy` supports runtime modules:

- `module list`
- `module load map`
- `module load graph`
- `module load waypoint` (via default module collection context)

Module loading can be done on startup (`--load-module`) or interactively.

Useful command examples:

- `rc N PWM` to inject RC override
- `link list`, `link set N`
- `wp list`, `wp set`, `wp draw`
- `fence enable`, `fence disable`
- `param show X`, `param set X N`
- `param download`, `param help X`
- `mode loiter`, `mode rtl`, `mode guided LAT LON ALT`
- `arm throttle`, `disarm`, `disarm force`
- `relay set N [0|1]`
- `terrain check LAT LON`

---

## 🔁 MAVProxy in ArduPilot simulation loops

`MAVProxy` is repeatedly referenced in `ArduPilot` developer guidance as the command-oriented interface for SITL-style iteration because its startup model maps cleanly to scripted runs.

Common practical pattern:

1. Start SITL/autopilot process.
2. Run MAVProxy on `--master` link.
3. Use `--out` links to feed one or more local clients (for example, `QGroundControl` as a visual mission/telemetry client).
4. Run command scripts against console/commands for deterministic workflows.

The docs also frame forwarding as a first-class capability for running multiple station clients from one simulator/autopilot run.

---

## ⚠️ Limitations and gotchas

- Serial permissions:
  - Add user to `dialout`/device group on Linux if USB open fails.
- Device naming differences:
  - Use explicit `--master` path carefully when multiple endpoints are attached.
- One-vehicle expectation:
  - Core documentation states one vehicle per MAVProxy session; use one session per vehicle for clean control separation.
- Command-line stability:
  - Avoid TCP forwarding for joystick-heavy operations when latency matters; UDP is the common recommendation.
- GUI-less path:
  - On minimal systems, headless mode (`--non-interactive`) is cleaner than a full interactive shell.

---

## ⚖️ Comparison chart

| Tool | Core model | Mission editing | Scriptability | Best use case |
|---|---|---|---|---|
| `MAVProxy` | CLI + modules + stream router | Basic mission commands (not full WYSIWYG editor) | Very high | Test loops, deterministic automation, SITL bridges |
| `[[QGroundControl]]` | Desktop GUI + map mission tools | Strong | Medium-low | Visual plan/fly for multi-stack users |
| `[[ArduPilot Mission Planner]]` | GUI-first ArduPilot workflows | Strong (Windows-first) | Medium | Field ops and Windows-native tuning |
| `MAVSDK` | API-level application/library | External APIs, custom apps | Very high | Production autonomy apps and service integration |
| `MAVROS` | ROS<->MAVLink bridge | Not mission editor | Medium-high | ROS-oriented control stacks |
| `MAVLink Console / scripts` | Direct MAVLink CLI or API | Manual | Varies | Embedded/debug operations |

---

## ✅ When MAVProxy is the right pick

- You want deterministic command logs and scriptable test loops.
- You need forwarding to multiple consumer tools from one stream source.
- You are doing frequent SITL/bench runs and care more about repeatability than UI comfort.

## ⚠️ When to avoid it

- You need first-touch operator UI for large mission geometry work.
- You want deep map-first mission planning in one-click workflows.
- You need a non-terminal-centric onboarding path for infrequent operators.

---

## 🔗 Related notes

- [[QGroundControl]]
- [[ArduPilot Mission Planner]]
- [[MAVLink]]
- [[SITL]]
- [[Brain-in-the-Loop Flight Sim]]
- [[ArduPilot]]

---

## 🗂 Sources (official)

- https://ardupilot.org/mavproxy/index.html
- https://ardupilot.org/mavproxy/docs/getting_started/download_and_installation.html
- https://ardupilot.org/mavproxy/docs/getting_started/quickstart.html
- https://ardupilot.org/mavproxy/docs/getting_started/starting.html
- https://ardupilot.org/mavproxy/docs/getting_started/forwarding.html
- https://ardupilot.org/mavproxy/docs/getting_started/cheatsheet.html
- https://ardupilot.org/mavproxy/docs/modules/index.html
- https://ardupilot.org/mavproxy/docs/getting_started/cheatsheet.html
- https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html

