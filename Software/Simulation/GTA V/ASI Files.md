# ASI Files for GTA V Modding

An `.asi` file is a native GTA plugin format used mainly in Script Hook-style modding on Windows builds.  
In practice, it behaves like a binary plugin loaded at game start through the ASI loader (`dinput8.dll`) so custom code can run inside GTA V’s process.

---

## 🧠 Overview

`*.asi` files are small native plugins (usually C++ code) that expose scripts and game hooks to GTA V.  
Mod loaders discover them from specific folders and inject them with `dinput8.dll` before GTA V gameplay loads.

In the GTA V ecosystem, `.asi` files are most commonly used for:

- telemetry tools
- custom behavior scripts
- automation and control bridges
- training-loop integrations for research simulations
- quality-of-life gameplay modding

---

## ⚙️ How `.asi` loading works

### 1) Loader layer
- `dinput8.dll` in the GTA V root is the ASI loader entry point for many mod stacks.
- The loader enumerates `.asi` files and DLL-style plugins listed by its rules, then injects them into the GTA V process.
- `ScriptHookV` and ScriptHook-compatible stacks coordinate the injected API contract with GTA internals.

### 2) Plugin type
- `.asi` files are typically DLLs with a `.asi` extension.
- They load similarly to `.dll` plugins but are managed by the mod loader’s discovery path rather than manually requiring explicit user launch.

### 3) Execution and compatibility
- On version mismatch (old loader with new game binary), startup can fail or produce loader critical errors.
- These plugins are generally used for single-player-style workflows; online-mode usage is commonly blocked by platform terms and mod frameworks.

---

## 🧩 What makes `.asi` different from other mod formats

| File type | Loader mechanism | Typical role | Common risk |
|---|---|---|---|
| `.asi` | ASI loader (`dinput8`) + ScriptHook compatibility | Runtime behavior injection / hooks | version/compatibility-sensitive |
| `.dll` | Manual plugin loading or loader-specific | Native plugin library | Generic load order issues |
| `.rpf` | Archive format | Game data replacement/modded assets | Corruption or bad replacement packs |
| `.oiv` / mod package | OpenIV/OIV metadata wrappers and tooling | Asset installation, packaging | installer path mismatches |
| `.zip` scripts | Framework-defined packaging | Distribution of script bundles | broken extraction/install scripts |

---

## 🧱 GTA V folder conventions

In a typical SP mod setup, you’ll usually see plugins near the GTA V root:

- `dinput8.dll`
- `ScriptHookV.dll`
- `ScriptHookV.ini` (or compatible loader metadata)
- `scripts\` folder for script-related assets
- `*.asi` files at the root or in recognized plugin directories

The exact expected tree depends on the hook stack and script framework version you use.

---

## 🔧 Practical development workflow (simulation research use)

1) Keep an isolated clean SP install dedicated to experiments.  
2) Install matching loader stack versions for your GTA binary/build.  
3) Drop `.asi` plugins only from trusted sources and confirm checksums when possible.  
4) Start a minimal plugin list and expand one plugin at a time.  
5) Validate launch behavior and log output before adding networking or input-control layers.

For long-running data-collection loops (e.g. car/FPV control), keep a strict allowlist for files/commands and collect telemetry separately so plugin crashes are contained.

---

## 🚫 Safety and legal boundaries

- `.asi` mod stacks are generally intended for single-player or test rigs.
- Avoid mixing online play when hooks are active.
- Be explicit in permissions: many users keep modded installs separate from any networked online profiles.
- If a framework blocks play in online mode, that behavior is usually by design.

---

## 🧠 Comparison chart for adjacent options

| Option | Ease of setup | Runtime control | Stability for long runs | Best for |
|---|---:|---:|---:|---|
| `.asi` + ScriptHookV | Medium | High | Medium | Research loops, behavior-heavy mods |
| Native game-script APIs (`.dll` integrations) | Low | Very high | Medium | Custom deep integrations |
| FiveM server resources (`.lua`) | Medium | Medium | High (server boundary) | Multiplayer drone/car experiments |
| Full custom C++ engine mod (without ASI) | Hard | Very high | Low–Medium | extreme low-level control |
| External simulator APIs (CARLA/AirSim) | Medium | High | High | rapid RL prototyping outside GTA |

---

## 🗂️ Related notes

- [[GTA V Modding for Simulation]]
- [[Reinforcement Learning]]
- [[TCP]]
- [[RLBench]]
- [[PufferLib]]
- [[Robot Data Collection and Teleoperation]]
- [[GTA V Simulation Mod Index]]

---

## 📚 Sources

- [ScriptHookV official](https://www.dev-c.com/gtav/scripthookv/)
- [ScriptHookV critical errors](https://scripthookv.io/script-hook-v-critical-error/)
- [ScriptHookVDotNet](https://github.com/scripthookvdotnet/scripthookvdotnet)
- [OpenIV modding ecosystem](https://openiv.com/)
- [DeepGTAV](https://github.com/aitorzip/DeepGTAV)
- [VPilot](https://github.com/aitorzip/VPilot)
