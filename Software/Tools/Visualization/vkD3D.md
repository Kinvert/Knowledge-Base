# vkD3D

vkD3D is an open-source translation layer that maps Microsoft Direct3D 12 (D3D12) API calls to Vulkan. It allows Windows games and applications that use D3D12 to run on platforms where Vulkan is supported, including Linux and macOS (via MoltenVK). vkD3D is especially relevant for engineers and researchers interested in graphics API translation, cross-platform rendering, and performance optimization in both gaming and simulation environments.

---

## 🧭 Overview

vkD3D operates as a runtime library intercepting D3D12 calls and converting them into equivalent Vulkan commands. Unlike emulators, vkD3D doesn’t simulate hardware but leverages Vulkan’s abstraction to execute GPU instructions natively. It is commonly used in combination with Wine or Proton to enable Windows-only games on Linux or other non-Windows platforms.

---

## 🧩 Core Concepts

- **API Translation**  
  Converts D3D12 commands into Vulkan calls while attempting to maintain feature parity and performance.

- **Command Lists & Queues**  
  D3D12's explicit resource and execution model are mapped to Vulkan’s command buffers and queues.

- **Resource Binding & Descriptors**  
  Handles conversion of descriptor heaps, root signatures, and pipeline state objects to Vulkan equivalents.

- **Synchronization & Barriers**  
  D3D12's fine-grained barriers are mapped to Vulkan pipeline barriers to preserve correctness.

- **Shader Translation**  
  HLSL shaders used in D3D12 are compiled into SPIR-V for Vulkan execution.

- **Integration with Wine/Proton**  
  Acts as a drop-in replacement for D3D12.dll when running Windows applications under Wine.

---

## 🔍 Comparison Chart

| Framework / Layer | Target API | Platform | Strength | Weakness | Typical Use Case |
|------------------|------------|---------|----------|----------|-----------------|
| **vkD3D** | D3D12 → Vulkan | Linux, macOS | Native GPU execution, good performance | Not 100% feature complete, requires Vulkan drivers | Running Windows D3D12 games cross-platform |
| DXVK | D3D9/10/11 → Vulkan | Linux | Mature, highly optimized | Only supports older D3D versions | D3D9-11 games on Linux |
| WineD3D | D3D9-11 → OpenGL | Linux, macOS | Broad support | Slower, legacy approach | Older Direct3D applications |
| MoltenVK | Vulkan → Metal | macOS/iOS | Vulkan on Apple platforms | Translation overhead, some Vulkan features missing | Cross-platform Vulkan deployment |
| DirectX 12 (native) | D3D12 | Windows | Full feature support | Windows-only | Native Windows gaming / simulation |

---

## 🛠️ Use Cases

- **Cross-Platform Gaming**  
  Enables Windows-only DirectX 12 titles to run on Linux and other OSes.

- **Research in Graphics API Translation**  
  Study how explicit GPU APIs can be mapped across different backend implementations.

- **Simulation & RL Visualization**  
  Running Windows-based graphics simulations in Linux environments for Reinforcement Learning experiments.

- **Performance Analysis**  
  Investigate Vulkan vs D3D12 performance in complex rendering scenarios.

---

## ⭐ Strengths

- Enables D3D12 applications to run on non-Windows platforms  
- Leverages Vulkan’s low-level GPU access for near-native performance  
- Supports modern D3D12 features like ray tracing and compute shaders (partial)  
- Actively maintained alongside Wine and Proton updates

---

## ⚠️ Weaknesses

- Incomplete support for some D3D12 advanced features  
- Shader translation may fail for some HLSL constructs  
- Dependent on quality of Vulkan drivers and GPU hardware  
- Occasionally requires per-game tweaks for full compatibility

---

## 🧰 Developer Tools & Features

- **Debug Layer**  
  Use Vulkan validation layers to debug translated calls.

- **Shader Compilation Logging**  
  Helps identify unsupported HLSL features during SPIR-V conversion.

- **Wine/Proton Integration**  
  Drop-in replacement for D3D12.dll in Wine prefix.

- **Performance Profiling**  
  Analyze frame times and GPU utilization in Vulkan using standard tools.

---

## 🧠 How It Works

vkD3D intercepts D3D12 runtime calls and constructs equivalent Vulkan objects. Resource creation, pipeline states, and command lists are translated to Vulkan buffers, pipelines, and command buffers. HLSL shaders are compiled to SPIR-V, then bound and executed on the GPU. Synchronization primitives like fences and barriers are mapped to Vulkan equivalents, ensuring correct execution order. The translation is mostly “just-in-time,” allowing high performance while preserving D3D12 semantics.

---

## 🧪 Capabilities

- Translation of core D3D12 graphics and compute APIs to Vulkan  
- Support for root signatures, descriptor heaps, pipeline state objects  
- GPU-based ray tracing (experimental / partial)  
- Integration with cross-platform runtime environments like Wine  
- Logging and debugging hooks for developers

---

## 🔄 Variants & Related Projects

- **vkD3D-Proton** — Optimized fork for Proton, enhancing gaming compatibility  
- **DXVK** — Older DirectX (9-11) to Vulkan translation layer  
- **MoltenVK** — Vulkan-to-Metal translation for Apple platforms  
- **WineD3D** — Direct3D to OpenGL translation (legacy)

---

## 🧩 Compatible Items

- Vulkan 1.2+ drivers  
- D3D12 Windows applications  
- Wine/Proton environments  
- SPIR-V shader compiler  
- Vulkan validation layers

---

## 📚 External Resources

- GitHub: `https://github.com/HansKristian-Work/vkd3d`  
- Documentation: Readme & Wiki on GitHub  
- WineHQ compatibility database for D3D12 games  
- Vulkan SDK for debugging and validation

---

## 🔗 Related Concepts / Notes

- [[Vulkan]] (Cross-platform modern graphics API)  
- [[Direct3D 12]] (Microsoft low-level graphics API)  
- [[SPIR-V]] (Intermediate shader representation)  
- [[DXVK]] (DirectX 9/10/11 → Vulkan translation)  
- [[MoltenVK]] (Vulkan → Metal)  
- [[Wine]] (Windows API compatibility layer)  
- [[Proton]] (Valve’s gaming Wine fork)

---

## 📝 Summary

vkD3D bridges the gap between Windows-exclusive Direct3D 12 software and cross-platform Vulkan-capable systems. By translating D3D12 commands to Vulkan in near real-time, it allows modern games and graphics applications to run outside Windows while preserving performance and rendering correctness. It is particularly relevant for engineers exploring cross-API graphics, simulation, or RL visualization environments.
