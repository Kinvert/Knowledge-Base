# DirectX

**DirectX** is Microsoft's collection of APIs for multimedia and game development on Windows and Xbox platforms. Originally released in 1995, it provides hardware-accelerated access to graphics, audio, input devices, and compute resources. The name "DirectX" comes from the "Direct" prefix shared by its component APIs (Direct3D, DirectSound, DirectInput), with "X" as a placeholder for any specific component.

---

## ⚙️ Overview

DirectX abstracts hardware capabilities into consistent programming interfaces, enabling developers to target diverse hardware without vendor-specific code. The most prominent component is [[D3D12|Direct3D]] for 3D graphics, but DirectX encompasses a complete multimedia stack including 2D rendering, text, audio, input, and GPU compute.

Key milestones:
- **1995:** DirectX 1.0 released for Windows 95 game development
- **2002:** DirectX 9 - long-lived standard, shader model 2.0/3.0
- **2006:** DirectX 10 - shader-only pipeline, no fixed-function
- **2009:** DirectX 11 - tessellation, compute shaders, multithreading
- **2015:** DirectX 12 - low-level explicit API, reduced CPU overhead
- **2020:** DirectX 12 Ultimate - raytracing, mesh shaders, VRS

---

## 🧩 Component APIs

| Component | Purpose | Notes |
|-----------|---------|-------|
| **Direct3D** | 3D graphics rendering | Core API - D3D9, D3D11, [[D3D12]] |
| **Direct2D** | 2D vector graphics | Hardware-accelerated, replaced DirectDraw |
| **DirectWrite** | Text rendering | Subpixel positioning, ClearType |
| **DirectCompute** | GPU compute shaders | General-purpose GPU computing |
| **DXGI** | Graphics infrastructure | Swapchains, adapters, display management |
| **XAudio2** | Low-level audio | Replaced DirectSound, submixing/effects |
| **XInput** | Controller input | Xbox controller support, replaced DirectInput |
| **DirectStorage** | Asset streaming | GPU-direct loading from NVMe SSDs |
| **DirectML** | Machine learning | Hardware-accelerated ML inference |

---

## 📊 Direct3D Version Comparison

| Version | Release | Shader Model | Key Features | Min Windows |
|---------|---------|--------------|--------------|-------------|
| D3D9 | 2002 | SM 2.0/3.0 | Fixed + programmable pipeline | XP |
| D3D10 | 2006 | SM 4.0 | Geometry shaders, no fixed-function | Vista |
| D3D10.1 | 2008 | SM 4.1 | Feature levels introduced | Vista SP1 |
| D3D11 | 2009 | SM 5.0 | Tessellation, compute, multithreading | 7 / Vista SP2 |
| D3D11.1 | 2012 | SM 5.0 | Stereoscopic 3D, improved interop | 8 |
| [[D3D12]] | 2015 | SM 5.1+ | Explicit API, command lists, low overhead | 10 |
| D3D12 Ultimate | 2020 | SM 6.x | DXR raytracing, mesh shaders, VRS | 10 (1903+) |

---

## 🎯 Feature Levels

Feature levels decouple API version from hardware capabilities, replacing D3D9's "capability bits":

| Feature Level | Hardware Era | Capabilities |
|---------------|--------------|--------------|
| 9_1, 9_2, 9_3 | D3D9 GPUs | SM 2.0/3.0, basic features |
| 10_0, 10_1 | D3D10 GPUs | Geometry shaders, stream output |
| 11_0, 11_1 | D3D11 GPUs | Tessellation, full compute |
| 12_0, 12_1 | D3D12 GPUs | Resource binding, typed UAV |
| 12_2 | D3D12 Ultimate | Raytracing, mesh shaders, VRS |

D3D12 requires minimum Feature Level 11_0. Lower levels require D3D11 API.

---

## 🔧 Key Technologies

**DirectX Raytracing (DXR):**
- Hardware-accelerated ray tracing for realistic lighting
- BVH traversal, ray-triangle intersection in hardware
- DXR 1.1 adds GPU work creation, inline tracing

**Mesh Shaders:**
- Replace vertex/geometry shader pipeline
- GPU-driven LOD selection and culling
- Process meshlets (small mesh subsets) efficiently

**Variable Rate Shading (VRS):**
- Per-tile shading rate control
- Reduce work in low-detail areas (shadows, periphery)
- Significant performance gains with minimal quality loss

**DirectStorage:**
- Bypass CPU for asset loading
- GPU decompression of textures/meshes
- Requires NVMe SSD for full benefit

---

## 🌟 Strengths

- Deep Windows/Xbox integration and optimization
- Comprehensive multimedia stack (graphics, audio, input)
- Excellent tooling (PIX, Visual Studio Graphics Debugger)
- Long-term backward compatibility
- Industry-leading raytracing support
- Extensive documentation and samples

---

## ⚠️ Weaknesses

- Windows/Xbox only - no cross-platform
- Multiple API versions create fragmentation
- Legacy D3D9/11 codebases slow to migrate
- Competes with cross-platform [[Vulkan]]
- Xbox requires separate GDK licensing

---

## 🆚 DirectX vs Alternatives

| Aspect | DirectX | [[Vulkan]] | [[OpenGL]] | Metal |
|--------|---------|--------|--------|-------|
| Platform | Windows/Xbox | Cross-platform | Cross-platform | Apple |
| Control | D3D12: Explicit | Explicit | Implicit | Semi-explicit |
| Ecosystem | Complete stack | Graphics/compute only | Graphics only | Complete stack |
| Raytracing | DXR | RT extensions | No | RT API |
| Tooling | Excellent | Good | Varies | Good |
| Adoption | Dominant on Windows | Growing | Legacy | Apple standard |

---

## 🔗 Related Notes

- [[D3D12]]
- [[Vulkan]]
- [[OpenGL]]
- [[GPU]]
- [[Swapchain]]
- [[CUDA]]

---

## 🌐 External Resources

- [DirectX Developer Blog](https://devblogs.microsoft.com/directx/)
- [DirectX Landing Page](https://learn.microsoft.com/en-us/windows/win32/directx)
- [DirectX Specs (GitHub)](https://microsoft.github.io/DirectX-Specs/)
- [PIX on Windows](https://devblogs.microsoft.com/pix/)
- [DirectX Tool Kit](https://github.com/microsoft/DirectXTK12)

---

## 📝 Summary

DirectX is Microsoft's comprehensive multimedia API collection, powering graphics, audio, and input on Windows and Xbox for nearly three decades. While Direct3D handles 3D rendering (from the legacy D3D9 to the modern low-level D3D12), supporting APIs cover 2D graphics, text, audio, controllers, and GPU compute. DirectX 12 Ultimate brings cutting-edge features like raytracing, mesh shaders, and DirectStorage to the Windows gaming and visualization ecosystem.
