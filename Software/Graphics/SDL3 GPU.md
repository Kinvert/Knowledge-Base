# SDL3 GPU

**SDL3 GPU** (`SDL_gpu`) is a cross-platform, mid-level graphics and compute API built into [[SDL3]]. It abstracts over [[Vulkan]], [[D3D12]], and [[Metal]], providing explicit GPU control (command buffers, render passes, compute shaders) with dramatically less boilerplate than raw Vulkan — roughly 100 lines for a basic setup vs ~1,170 for equivalent Vulkan code. It ships as part of SDL3 itself with zero additional dependencies.

---

## ⚙️ Overview

SDL3 GPU follows the design philosophy of modern explicit APIs — the developer fills command buffers and submits them to the GPU, making the CPU-GPU boundary transparent. Unlike SDL2's `SDL_Renderer` (a simplified 2D blitter), SDL3 GPU exposes 3D rendering, compute dispatch, and explicit data transfers while automating the hardest parts: memory management, pipeline barriers, and descriptor handling.

SDL2's `SDL_Renderer` still exists in SDL3 but now uses SDL_GPU as its internal backend.

**Key design principles** ([wiki.libsdl.org - CategoryGPU](https://wiki.libsdl.org/SDL3/CategoryGPU)):
- GPUs are separate processors — data transfer is explicit
- Command buffers collect work, then submit in batch
- No hidden threads — SDL only acts when you call functions
- Automatic barriers and memory management
- Resource "cycling" avoids manual synchronization

---

## 🧑‍💻 Origins

The API was funded by an **Epic MegaGrant** to Ryan C. Gordon (icculus) and built primarily by the **FNA team**, funded by **Re-Logic** (Terraria developers, who donated $100K to FNA in 2023). The FNA team had prior experience with two cross-platform GPU abstractions: FNA3D and Refresh.

| Person | Role |
|--------|------|
| **Evan Hemsley** (thatcosmonaut) | Project lead |
| **Caleb Cornett** (TheSpydog) | Co-designer, Metal port, console ports |
| **Ethan Lee** (flibitijibibo) | Production, QA, debug |
| **Ryan C. Gordon** (icculus) | Original API proposal, shader tooling |

Merged into SDL3 on August 29, 2024. SDL3 reached its first stable release (3.2.0) on January 21, 2025.

**Sources:**
- [Merge commit 2e7d5bb](https://github.com/libsdl-org/SDL/commit/2e7d5bb429b5fa9ad9c15e69ca7d952827420a52)
- [Epic MegaGrant pitch](https://gist.github.com/icculus/f731224bef3906e4c5e8cbed6f98bb08)
- [Re-Logic donation](https://www.gamingonlinux.com/2023/09/terraria-developer-re-logic-donates-100k-to-godot-engine-and-fna/)
- [FNA GPU API announcement](https://icculus.org/finger/flibitijibibo?date=2024-06-15&time=13-14-16)

---

## 🧠 Core Concepts

- **Command Buffer** — Collects GPU instructions for batch submission. Typically one per frame. Created with `SDL_AcquireGPUCommandBuffer()`, submitted with `SDL_SubmitGPUCommandBuffer()`
- **Render Pass** — Where drawing happens. Supports up to 4 color targets + 1 depth target. Begin/end with `SDL_BeginGPURenderPass()` / `SDL_EndGPURenderPass()`
- **Compute Pass** — GPU compute dispatch. Can interleave with render passes in a single command buffer
- **Copy Pass** — Data transfer operations (uploading textures, buffers to GPU)
- **Graphics Pipeline** — Pre-compiled rendering state (`SDL_GPUGraphicsPipeline`)
- **Compute Pipeline** — Pre-compiled compute state (`SDL_GPUComputePipeline`)
- **Cycling** — Unique mechanism where transfer buffers, buffers, and textures act as ring buffers. When `cycle=true` and a resource is bound, it rotates to the next unbound internal resource, avoiding manual sync
- **Fences** — Optional synchronization primitives for CPU-GPU coordination

**Source:** [wiki.libsdl.org - CategoryGPU](https://wiki.libsdl.org/SDL3/CategoryGPU)

---

## 🖥️ Backends

| Backend | Platforms | Requirements |
|---------|-----------|--------------|
| [[Vulkan]] | Windows, Linux, Android, Nintendo Switch | Vulkan 1.0 + extensions (~99% of hardware) |
| [[D3D12]] | Windows 10+, Xbox One, Xbox Series X/S | Feature Level 11_0, Resource Binding Tier 2+ |
| [[Metal]] | macOS 10.14+, iOS/tvOS 13.0+ | Varies by OS version |

**Not supported:** [[OpenGL]], OpenGL ES, WebGL, D3D11, software renderers. A D3D11 backend was attempted but abandoned due to driver bugs. A WebGPU backend was proposed ([PR #12046](https://github.com/libsdl-org/SDL/pull/12046)) but not merged. Console backends (PS5, Switch, Xbox) exist in a separate NDA fork.

**Source:** [wiki.libsdl.org - SDL_HINT_GPU_DRIVER](https://wiki.libsdl.org/SDL3/SDL_HINT_GPU_DRIVER)

---

## 🔧 Shader Story

SDL3 GPU does not mandate a shader language — it only accepts compiled binaries per backend:

| Backend | Required Format |
|---------|----------------|
| Vulkan | SPIR-V |
| D3D12 | DXIL (SM6.0) |
| Metal | MSL or MetalLib |

Three approaches exist:

1. **Ship pre-compiled shaders** per backend (most control, most work)
2. **SDL_shadercross** (satellite library) — takes HLSL or SPIR-V as input, outputs all formats. Supports offline CLI compilation and runtime cross-compilation. Built on SPIRV-Cross and DXC. Also provides automatic resource reflection (JSON output of samplers, buffers, textures). [github.com/libsdl-org/SDL_shadercross](https://github.com/libsdl-org/SDL_shadercross)
3. **SDLSL** (future, WIP) — a new shader language in development at [github.com/libsdl-org/SDL_shader_tools](https://github.com/libsdl-org/SDL_shader_tools). Has lexer/parser/semantic analysis but no bytecode generation yet — "not yet ready for anything, let alone production use"

**Source:** [Introducing SDL_shadercross - MoonsideGames](https://moonside.games/posts/introducing-sdl-shadercross/)

---

## 📊 Comparison Chart

| Feature | SDL3 GPU | Raw [[Vulkan]] | WebGPU / wgpu | bgfx | sokol_gfx |
|---------|----------|------------|-------------|------|-----------|
| **Language** | C | C/C++ | C/Rust | C++ | C |
| **Abstraction** | Mid-level | Low-level | Mid-level | High-level | Mid-level |
| **Backends** | Vulkan, Metal, D3D12 | Vulkan only | Vulkan, Metal, D3D11, D3D12, WebGPU | GL, Vulkan, Metal, D3D11, D3D12 | GL, Metal, D3D11, WebGPU |
| **Console Support** | Yes (Switch, PS5, Xbox) | No | No | Unknown | No |
| **Compute Shaders** | Yes | Yes | Yes | Yes | No |
| **Web Support** | No (planned) | No | Yes | Yes | Yes |
| **Boilerplate** | ~100 lines | ~1,000+ lines | Moderate | Moderate | Minimal |
| **Dependencies** | None (part of SDL) | Vulkan SDK | Significant | Significant | None |
| **Maturity** | New (2024-2025) | Mature | Maturing | Mature | Mature |

Key differentiators from community discussion ([HN thread](https://news.ycombinator.com/item?id=41396260)):
- **vs Vulkan:** "An order of magnitude less code" — handles barriers, memory, descriptors automatically
- **vs WebGPU:** Supports consoles; WebGPU does not. Avoids inventing a new shader language
- **vs bgfx/sokol:** Exposes command buffers directly (lower level) while handling barriers. No hidden threading
- **vs wgpu:** "SDL is for gamedevs, it supports consoles. wgpu is not, it doesn't"

---

## ✅ Strengths

- Write once, deploy to Windows, macOS, Linux, iOS, Android, Switch, PS5, Xbox
- Dramatically less boilerplate than raw Vulkan
- Pure C, zero dependencies beyond SDL itself
- Automatic memory management, barriers, and descriptor handling
- Full compute shader support
- Console support (major advantage over WebGPU, wgpu, sokol, bgfx)
- Production-tested through FNA (hundreds of shipped game SKUs)
- Integrates with RenderDoc for GPU debugging
- Dear ImGui backend available (`imgui_impl_sdlgpu3.cpp`)
- Active maintenance — Valve employees contribute full-time

---

## ⚠️ Weaknesses

- No web support (no WebGPU backend yet)
- Limited Android support due to inconsistent Vulkan drivers across devices
- New API — fewer tutorials and community resources vs OpenGL/Vulkan
- Intentionally excludes cutting-edge features: no bindless textures, mesh shaders, or raytracing
- SDLSL shader language still WIP — must use HLSL/SPIR-V with shadercross for now
- Render pass and resource creation are expensive (need to pre-allocate and cache)
- Cycling mechanism requires careful understanding to avoid data consistency bugs

---

## 🎮 Games and Projects

- **FNA titles** — Celeste, Terraria, FEZ, Dust: An Elysian Tail, Bastion, and hundreds more ship through FNA, which uses SDL3 GPU ([SIGGRAPH 2025 transcript](https://wiki.libsdl.org/SDL3/SIGGRAPH2025PanelTranscript))
- **Samurai Gunn 2** — Built with MoonWorks (predecessor framework using Refresh). PS5 + Steam ([Steam page](https://store.steampowered.com/app/1397790/Samurai_GUNN_2/))
- **Foster Framework** — Indie framework by Celeste creator Noel Berry, moving to SDL3 GPU ([GitHub issue #1](https://github.com/FosterFramework/Foster/issues/1))
- **Dear ImGui** — Official SDL_GPU backend with pre-compiled shaders for all backends

---

## 🔗 Related Concepts

- [[Vulkan]] (one of SDL3 GPU's backends)
- [[D3D12]] (one of SDL3 GPU's backends)
- [[Metal]] (one of SDL3 GPU's backends)
- [[OpenGL]] (legacy API that SDL3 GPU replaces)
- [[CUDA]] (GPU compute — different domain, but related hardware)

---

## 📚 External Resources

- SDL Wiki GPU Category: https://wiki.libsdl.org/SDL3/CategoryGPU
- SDL_shadercross: https://github.com/libsdl-org/SDL_shadercross
- SIGGRAPH 2025 Panel Transcript: https://wiki.libsdl.org/SDL3/SIGGRAPH2025PanelTranscript
- SDL3 GitHub: https://github.com/libsdl-org/SDL
- MoonsideGames blog: https://moonside.games
- HN discussion (GPU API merge): https://news.ycombinator.com/item?id=41396260
