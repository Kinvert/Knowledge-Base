# Swapchain

A **swapchain** is a core graphics concept—especially in APIs like Vulkan and Direct3D—representing a queue of images used for rendering and presenting frames to the screen. In Vulkan, it is fundamental to controlling latency, synchronization, and frame pacing. When your friend asked you to maximize the window to observe FPS changes, the swapchain was involved in reallocating images and synchronizing rendering at the new resolution.

---

## 🛰️ Overview

A swapchain is a collection of images (often 2–3) that cycle between being drawn to, queued for presentation, and displayed. In Vulkan, the application explicitly manages almost every aspect of the swapchain, including format, image count, vsync mode (present mode), and resizing behavior. This gives high performance and low-latency control but demands careful engineering.

---

## 🧠 Core Concepts

- **Images/Buffers**: The swapchain contains multiple images (typically double or triple buffering).
- **Present Modes**: FIFO (vsync), MAILBOX (low latency), IMMEDIATE, etc.
- **Surface Capabilities**: Determined by the windowing system (WSI).
- **Synchronization**: Semaphores and fences control rendering → presentation order.
- **Recreation**: When a window resizes, loses focus, or changes refresh behavior, the swapchain must be recreated.
- **Backbuffer**: The render target currently being drawn to.
- **Frontbuffer**: The image visible to the user.

---

## 📊 Comparison Chart

| Feature               | Vulkan Swapchain | OpenGL Swap Buffers | DirectX 12 Swapchain | Metal Drawable Queue | WebGPU Swapchain |
|----------------------|------------------|----------------------|-----------------------|-----------------------|-------------------|
| Control Level        | Very High        | Low                 | High                 | Medium               | Medium            |
| Explicit Sync        | Yes              | No (implicit)       | Yes                  | Partial              | Partial           |
| Resize Handling      | Manual           | Automatic           | Semi-manual          | Automatic            | Abstracted        |
| Present Modes        | Many             | Limited             | Vsync/AllowTearing   | System defined       | Implementation    |
| Ideal Use Case       | Engines, GPU RL  | Fast prototyping    | High-performance     | Apple-only apps      | Browser compute   |

---

## 🔧 How It Works

- The GPU renders into a swapchain image not currently being displayed.
- After finishing the render pass, the app signals the presentation engine that the image is ready.
- The presentation engine swaps images (or copies, depending on mode) to make the new one visible.
- Synchronization primitives ensure no two GPU stages access the same image incorrectly.
- When window size or surface capabilities change, the swapchain becomes "out-of-date" and must be recreated with new parameters.

---

## 🚀 Use Cases

- **High-performance game engines**
- **RL environments requiring GPU-accelerated visualization**
- **Scientific visualization**
- **GPU-based UI rendering**
- **Profiling + benchmarking to see how resolution affects FPS**

---

## ⭐ Strengths

- Fine-grained control over latency and frame pacing
- Explicit GPU synchronization
- Customization of buffer counts and formats
- Necessary for low-latency UIs, VR, and high-FPS games

---

## ⚠️ Weaknesses

- Complex API surface
- Must be recreated on resize or surface loss
- Easy to get synchronization wrong
- Performance varies across drivers/platforms

---

## 🔩 Variants (in Vulkan Present Modes)

- **FIFO** (vsync, safe default)
- **FIFO_RELAXED**
- **IMMEDIATE** (no vsync, can tear)
- **MAILBOX** (triple-buffered low-latency)
- **SHARED / Shared Demand Refresh** (rare)

---

## 🧪 Example Behaviors When Resizing Window

- Higher resolution → more pixels → higher GPU load → lower FPS
- Swapchain resize → reallocation of image memory
- Present mode might switch if unsupported at new size
- Frame pacing changes if GPU becomes bottlenecked

---

## 🛠️ Developer Tools

- Vulkan validation layers
- RenderDoc swapchain inspection
- `vulkaninfo` for surface/present details
- GPUView for frame pacing
- Nsight Graphics

---

## 📚 Related Concepts

- [[Vulkan]] (Graphics API)
- [[GPU]] (Graphics hardware)
- [[Framebuffer]] (Off-screen rendering target)
- [[Vsync]] (Vertical synchronization)
- [[Latency]] (Frame timing)
- [[Triple Buffering]] (Swapchain strategy)
- [[WSI]] (Window System Integration)
- [[Render Pipeline]] (GPU stages)

---

## 🔗 External Resources

- Vulkan Specification (KHR)
- Khronos Vulkan Tutorials (e.g., vulkan-tutorial.com)
- NVIDIA and AMD swapchain best practices
- RenderDoc documentation

---

## 🏁 Summary

Swapchains are the backbone of rendering output in Vulkan and similar graphics APIs. They define how frames move from GPU pipelines to the display, controlling latency, vsync, and buffer management. When you maximized the window and noticed FPS changes, the swapchain was recreated with larger images, increasing GPU workload and affecting performance.
