# Libretro

Libretro is a simple C API that abstracts emulator cores behind a common interface. It is the foundation that [[RetroArch]] uses. For RL research it is a robust non-web, non-Mujoco way to turn classic games into deterministic programmatic simulators with raw state extraction.

---

## 🧠 Overview

Libretro defines a very thin ABI boundary on top of GPU, input, audio, environment callbacks, and RAM access. This means emulator cores written for Libretro all expose a consistent interaction layer. For RL you can bypass rendering, sample framebuffer, or even directly read memory to build a custom structured state vector.

---

## 🧩 Core Concepts (RL Relevant)

- libretro core is a `.so` or `.dll` implementing the libretro API entrypoints like `retro_run`
- Rendering is pushed each frame to your callback; you can intercept the framebuffer
- Libretro exposes a `retro_get_memory_data` to inspect emulator RAM
- You can use `retro_serialize` and `retro_unserialize` for save-state jumping (perfect for RL episode resets)
- You can run headless, no real UI needed
- RayLib can be used as a small window/input abstraction layer if you want to use Libretro outside RetroArch, but often RL runs headless and RayLib is only needed when you want a tiny UI for debugging policies

---

## 🥇 RL Capabilities

- deterministic stepping per frame (ideal for on-policy methods like [[PPO]])
- build an observation from any combination of:
  - raw RAM (symbolic features)
  - framebuffer (pixel-based)
  - custom memory parsing (game-specific state machines)
- implement rewards from direct game-logic signals rather than “score” if you have known RAM layout
- create curriculum by save-state jump
- parallelize: load multiple cores and step them independently for massive sample throughput

---

## 🔁 Comparison Chart

| Tech | Purpose | RL Utility | Determinism | Access to RAM? | Licensing Notes |
|---|---|---|---|---|---|
| Libretro | Interface to emulators | High | High | Yes | Many cores are questionable grey area |
| OpenAI Gym Retro | Similar to libretro but pythonic | Good | Good | Yes | Deprecated and stalled |
| ALE (Atari Learning Env) | Specific Atari suite | Excellent | Excellent | Yes | Only Atari |
| SDL2 + custom emulator | DIY emulator | Depends | Depends | If implemented | Depends |
| RayLib | lightweight graphics library | Medium | N/A | N/A | zlib |

---

## 🧰 Strengths

- minimal overhead, pure C ABI
- widely available cores
- easier than writing an emulator
- trivial to call from Python via ctypes or FFI

---

## ❌ Weaknesses

- legal/licensing is messy for commercial robots (ROMs)
- RAM layout reverse engineering is required for crisp structured RL observations
- documentation is spotty

---

## 🧪 Use Cases (RL Focus)

- training value-based RL like [[DQN]] on NES games with direct memory access
- frame-skip experiments
- compare symbolic reward vs score reward in same game
- curriculum via rewind, save states, injecting known world states

---

## 🔗 Related Concepts / Notes

- [[RetroArch]]
- [[FFI]] (Foreign Function Interface)
- [[ctypes]] (Python)
- [[OpenAI Gym]]
- [[PPO]] (Proximal Policy Optimization)
- [[DQN]] (Deep Q Network)

---

## 📚 External Resources

- https: // libretro.com
- Core API header file: `libretro.h` (canonical contract)
