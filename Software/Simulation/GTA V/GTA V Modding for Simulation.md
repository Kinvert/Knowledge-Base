# GTA V Modding for Simulation

This note focuses on GTA V as a high-fidelity simulation base for research/experiments, especially:

- [[Reinforcement Learning]]-style self-driving car control loops
- FPV drone simulation pipelines with image/action traces
- physics data collection pipelines that can feed [[OpenAI Gym]]-style ML/RL workflows

If you are building plugins, this also relies heavily on [[ASI Files]] and the ScriptHook/dinput8 stack.

All claims below are linked to external sources.

If you are running the same patterns in other simulators first, these notes are typically paired with this one: [[AirSim]], [[CARLA]], [[Isaac Sim]], [[CoppeliaSim]], [[Gazebo Garden]], and [[OpenAI Gym]]/[[Gymnasium]].

For a curated list of candidate mods, RC/aerial/UGV/farming projects, and experiment ideas, see [[GTA V Simulation Mod Index]].

## 1) What to use for GTA V simulation stacks

### Core dependencies
- **`ScriptHookV`** is the native hook layer for GTA V `.asi` plugins and is designed for custom single-player modding; it explicitly stops game play in GTA Online when multiplayer is entered [source](https://www.dev-c.com/gtav/scripthookv/), [source](https://scripthookv.io/script-hook-v-critical-error/).
- **`dinput8.dll` (ASI Loader)** is required so GTA can load plugins in the install folder [source](https://www.dev-c.com/gtav/scripthookv/).
- `ScriptHookV.dll`, `ScriptHookV.ini` equivalents, and related plugin files must be kept version-matched with the current GTA V build; mismatches commonly produce startup failures and critical errors [source](https://www.dev-c.com/gtav/scripthookv/), [source](https://scripthookv.io/script-hook-v-critical-error/).
- **`ScriptHookVDotNet`** is a `.NET` runtime runner for GTA V scripts, distributed via GitHub, and exposes API versions and compatibility notes [source](https://github.com/scripthookvdotnet/scripthookvdotnet), [source](https://github.com/scripthookvdotnet/scripthookvdotnet/blob/main/README.md).
- **`OpenIV`** is the common archive/data editor for `.rpf` resources and can install mods under the `mods` folder with ASI manager support [source](https://openiv.co/), [source](https://openiv.com/).
- The `OpenIV` guide explicitly says GTA mods are not supported in GTA Online and renaming `dinput8.dll` disables `.asi` loading and lets users try to use online play [source](https://github-wiki-see.page/m/5mods/tutorials/wiki/Quick-start-overview-of-modding-Grand-Theft-Auto-V).

### Why this setup is useful for simulation
- Hooks let you expose in-engine state and control points without rewriting the game engine, which is how most GTA V simulation wrappers are built (self-driving car research and drone control both rely on this pattern) [source](https://scripthookv.io/script-hook-v-critical-error/), [source](https://github.com/aitorzip/DeepGTAV), [source](https://github.com/aitorzip/VPilot), [source](https://github.com/Sentdex/pygta5).
- This mirrors common environment contracts used by [[PufferLib]] and [[Offline RL for Robotics]]-grade sim stacks that depend on stable telemetry and action formatting.

## 2) Self-driving car pipelines in GTA V (practical)

### A. DeepGTAV + custom agent loop
- DeepGTAV is explicitly a GTA V plugin for vision-based self-driving research and listens on [[TCP]] (default port `8000`) for control/data messages [source](https://raw.githubusercontent.com/aitorzip/DeepGTAV/master/README.md).
- Control and config messages are structured as JSON commands (`Start`, `Config`, `Commands`, `Stop`) and data is returned in structured payloads (frame + telemetry-like payloads) [source](https://raw.githubusercontent.com/aitorzip/DeepGTAV/master/README.md).
- Example messages in the project docs show steering/brake/throttle command fields and JSON negotiation [source](https://raw.githubusercontent.com/aitorzip/DeepGTAV/master/README.md).

### B. VPilot companion interface
- VPilot is described as “scripts and tools to communicate with DeepGTAV,” using JSON over [[TCP]] in Python examples for dataset collection and driving loops [source](https://raw.githubusercontent.com/aitorzip/VPilot/master/README.md).
- That README is the shortest practical bridge for building a trainer/inference loop around DeepGTAV [source](https://raw.githubusercontent.com/aitorzip/VPilot/master/README.md).

### C. Sentdex-style autonomous-driving workflow
- Sentdex’s GTA V project was explicitly rebooted with a new architecture: automatic data collection, centralized trainer/server architecture, and live streaming training instead of manual sample labeling [source](https://raw.githubusercontent.com/Sentdex/pygta5/master/README.md).
- Sentdex documents a central `Server` + `Data Collectors` + `Trainer` + `Player` flow for multi-instance collection + learning + inference [source](https://raw.githubusercontent.com/Sentdex/pygta5/master/README.md).
- Their setup uses a dual-camera training layout (third-person + hood camera) and streams model outputs during training [source](https://raw.githubusercontent.com/Sentdex/pygta5/master/README.md).
- This pattern is a practical match for [[Flight-Sim RL Contract]] when you need explicit data contracts between collect/train/deploy.

### D. Sentdex GANTheftAuto reference implementation
- `GANTheftAuto` documents a custom GTA V mod + Python mod pair for data collection and steering-command control with road-node path generation and multi-stream collection [source](https://github.com/Sentdex/GANTheftAuto).
- It also documents that the data path uses saved action labels and observations (steering/action labels), showing a concrete supervised dataset format for GTA-based driving experiments [source](https://github.com/Sentdex/GANTheftAuto).

## 3) FPV / drone simulation in GTA V

### A. Quadcopter-Redux (GTA V FPV)
- The official mod package is documented through the Quadcopter-Redux mod page and guide repo with practical install and controls [source](https://www.gta5-mods.com/scripts/quadcopter-redux), [source](https://github.com/fredakilla/Quadcopter-Redux-Guide).
- The guide lists required files (`ScriptHookV`, `ScriptHookVDotNet`, Quadcopter-Redux) and expected root layout in GTA V (`dinput8.dll`, `.asi`, `.dll`, `scripts\Quadcopter\...`) [source](https://github.com/fredakilla/Quadcopter-Redux-Guide/blob/main/ReadMe.md).
- Controls are explicitly listed (switch modes, reset, camera tilt, flight-mode toggle, mission controls), and config supports controller bindings/rates presets [source](https://raw.githubusercontent.com/fredakilla/Quadcopter-Redux-Guide/main/ReadMe.md).
- The GTA5-mod page notes this mod is not fully compatible with GTA V Enhanced and is currently paused in favor of TRYP FPV development [source](https://www.gta5-mods.com/scripts/quadcopter-redux).

### B. FiveM drone scripts (server-side simulation extensions)
- FiveM is a multiplayer framework that lets servers expose custom cars/maps/scripts and explicitly says it does not modify GTA V installation files, making switching between GTA Online and FiveM easier [source](https://fivem.net/documentation/).
- FiveM documentation says it is source-available and contributions are open via GitHub/repo resources [source](https://fivem.net/documentation/), [source](https://github.com/citizenfx/fivem).
- Example community drone resources exist as open repositories/scripts (e.g., `nzkfc_drone`) with FPV and companion-drone behaviors documented in forum threads with code release links [source](https://forum.cfx.re/t/nzkfc-drone-a-companion-drone-script/5389587), [source](https://github.com/nzkfc/nzkfc_drone).
- Another ESX/QB conversion style repo (`frebespinal/fivem-drone`) exists publicly and includes implementation details, showing the pattern for server/client drone resources [source](https://github.com/frebespinal/fivem-drone).
- That architecture is closest to a [[ROS2]] data bridge pattern with [[ROS2 Publishers]] and [[ROS2 Subscribers]].

## 4) Comparison table (recommended starting points)

| Project | Focus | Interface | Open/Closed | Strengths | Weakness |
|---|---|---|---|---|---|
| [DeepGTAV](https://github.com/aitorzip/DeepGTAV) | Self-driving research plugin | TCP JSON (`Start`/`Config`/`Commands`/`Stop`) [source](https://raw.githubusercontent.com/aitorzip/DeepGTAV/master/README.md) | GitHub repo (public listing) | End-to-end message protocol for vision-based data/control; simple start/config/command loop | Older GTA build compatibility (explicitly version-limited) [source](https://raw.githubusercontent.com/aitorzip/DeepGTAV/master/README.md) |
| [VPilot](https://github.com/aitorzip/VPilot) | DeepGTAV controller layer | Python API over TCP sockets, dataset + drive examples [source](https://raw.githubusercontent.com/aitorzip/VPilot/master/README.md) | GitHub repo | Ready-made bridge to DeepGTAV, good for quick experiments | Focused on DeepGTAV only; not a full end-to-end trainer |
| [Sentdex/pygta5](https://github.com/Sentdex/pygta5) | Autonomous-driving systems infrastructure | Central server/collector/trainer/player architecture [source](https://raw.githubusercontent.com/Sentdex/pygta5/master/README.md) | GitHub repo | Richer data flow, live training stream design | Heavy project, older model ecosystem assumptions |
| [Sentdex/GANTheftAuto](https://github.com/Sentdex/GANTheftAuto) | Data-first GTA driving dataset + baseline models | Custom GTA mod + Python environment for steering and environment control [source](https://github.com/Sentdex/GANTheftAuto) | GitHub repo | Good for reproducible offline experiments and baseline datasets | Cannot redistribute full collector script [source](https://github.com/Sentdex/GANTheftAuto) |
| [Quadcopter-Redux](https://www.gta5-mods.com/scripts/quadcopter-redux) | FPV drone simulator in GTA V world | In-game keys + controller bindings [source](https://www.gta5-mods.com/scripts/quadcopter-redux) and guide install files list [source](https://github.com/fredakilla/Quadcopter-Redux-Guide/blob/main/ReadMe.md) | Distribution via GTA5-Mods + guide repo | Best practical FPV “feel” for GTA V world physics [source](https://www.gta5-mods.com/scripts/quadcopter-redux) | Not fully compatible with Enhanced Edition; mod distribution is primarily package-based [source](https://www.gta5-mods.com/scripts/quadcopter-redux) |
| [nzkfc_drone](https://github.com/nzkfc/nzkfc_drone) | FiveM companion drone script | Server resource behavior (FPV mode, companion follow) [source](https://forum.cfx.re/t/nzkfc-drone-a-companion-drone-script/5389587), [source](https://github.com/nzkfc/nzkfc_drone) | GitHub repository | Lightweight server-side drone behavior experimentation | FiveM server integration and multiplayer assumptions |
| [frebespinal/fivem-drone](https://github.com/frebespinal/fivem-drone) | FiveM drone script conversion | Lua/resource-style repo with gameplay scripts [source](https://github.com/frebespinal/fivem-drone) | GitHub repository | Good reference for QB/ESX style conversions | Minimal docs depth in repo listing [source](https://github.com/frebespinal/fivem-drone) |

## 5) Starter architectures you can copy

### 5.1 Self-driving car in live loop (low-friction)
1. Install clean SP GTA V and matching GTA version.
2. Install `ScriptHookV` + `ASI loader` + `ScriptHookVDotNet` if needed [source](https://www.dev-c.com/gtav/scripthookv/), [source](https://github.com/scripthookvdotnet/scripthookvdotnet).
3. Install DeepGTAV files in GTA root and launch until JSON control server is available on TCP [source](https://raw.githubusercontent.com/aitorzip/DeepGTAV/master/README.md).
4. In Python, connect using VPilot-like socket flow: send `start` config, receive `Frame/Data`, send `commands` for steering/brake/throttle [source](https://raw.githubusercontent.com/aitorzip/DeepGTAV/master/README.md), [source](https://raw.githubusercontent.com/aitorzip/VPilot/master/README.md).

```text
-> Start DeepGTAV
   - Send JSON: start({drivingMode, scenario, dataset params})
   - Receive: frame + data payloads
-> Trainer loop
   - Predict action
-> Send JSON: Commands({throttle,brake,steering})
-> Receive next frame/data
```

### 5.2 FPV drone evaluation loop
1. Install required GTA V mod stack (`ScriptHookV`, `dinput8`, `ScriptHookVDotNet`, Quadcopter mod files) [source](https://github.com/fredakilla/Quadcopter-Redux-Guide/blob/main/ReadMe.md), [source](https://www.gta5-mods.com/scripts/quadcopter-redux).
2. Enable mod mode, start in Story Mode, enter drone mode with `G` as documented [source](https://www.gta5-mods.com/scripts/quadcopter-redux).
3. Use direct-input controller mapping for realistic inputs or xbox fallback [source](https://github.com/fredakilla/Quadcopter-Redux-Guide/blob/main/ReadMe.md).
4. Capture state + control logs for downstream policy learning.

   - Relevant cross-project notes: [[Robot Data Collection and Teleoperation]], [[ROS2]], [[RLBench]].

```text
G        -> enter drone mode
F10      -> drone settings
PageUp   -> tilt camera up
PageDown -> tilt camera down
R        -> reset
F        -> flight-mode toggle (angle/acro)
```

### 5.3 FiveM-compatible drone extension path
1. Stand up a FiveM server in dev mode [source](https://fivem.net/documentation/).
2. Build/port drone gameplay as a resource script.
3. Use existing public resources as references and migrate behavior gradually [source](https://github.com/nzkfc/nzkfc_drone), [source](https://github.com/frebespinal/fivem-drone).
4. Use standalone or framework integrations and keep server/client boundaries explicit.

## 6) Internal notes to follow-up
- [[Reinforcement Learning]]
- [[OpenAI Gym]]
- [[Gymnasium]]
- [[PufferLib]]
- [[Robot Data Collection and Teleoperation]]
- [[ROS2]]
- [[ROS2 Node]]
- [[ROS2 Publishers]]
- [[ROS2 Subscribers]]
- [[TCP]]
- [[CARLA]]
- [[AirSim]]
- [[Isaac Sim]]
- [[Gazebo Garden]]
- [[CoppeliaSim]]
- [[RLBench]]
- [[Flight-Sim RL Contract]]
- [[PufferLib Flight Sim Integration Gaps]]

## 7) What to avoid / operational cautions
- **Do not use these mod stacks in online play**: official docs and community guides repeatedly flag script/ASIs as offline/SP-only and not for GTA Online [source](https://scripthookv.io/script-hook-v-critical-error/), [source](https://github-wiki-see.page/m/5mods/tutorials/wiki/Quick-start-overview-of-modding-Grand-Theft-Auto-V).
- `ScriptHookV` can be version-sensitive; update it with GTA updates or your mod layer fails [source](https://scripthookv.io/script-hook-v-critical-error/), [source](https://www.dev-c.com/gtav/scripthookv/).
- FiveM can be safer for multiplayer experimentation since it is documented as not modifying GTA V install files, but it is a separate framework and not the same as SP mod injection [source](https://fivem.net/documentation/).

## 8) Useful external references
- ScriptHookV (official): https://www.dev-c.com/gtav/scripthookv/
- ScriptHookV Critical Errors: https://scripthookv.io/script-hook-v-critical-error/
- ScriptHookVDotNet: https://github.com/scripthookvdotnet/scripthookvdotnet
- OpenIV official: https://openiv.com/
- 5mods quickstart/disable guide (mod/folder handling): https://github-wiki-see.page/m/5mods/tutorials/wiki/Quick-start-overview-of-modding-Grand-Theft-Auto-V
- DeepGTAV: https://github.com/aitorzip/DeepGTAV
- VPilot: https://github.com/aitorzip/VPilot
- Sentdex/pygta5: https://github.com/Sentdex/pygta5
- Sentdex/GANTheftAuto: https://github.com/Sentdex/GANTheftAuto
- Quadcopter-Redux Guide: https://github.com/fredakilla/Quadcopter-Redux-Guide
- Quadcopter-Redux mod page: https://www.gta5-mods.com/scripts/quadcopter-redux
- FiveM docs: https://fivem.net/documentation/
- FiveM repo: https://github.com/citizenfx/fivem
- nzkfc_drone: https://github.com/nzkfc/nzkfc_drone
- frebespinal/fivem-drone: https://github.com/frebespinal/fivem-drone
