# Linux Requirements

This note maps processor and platform requirements for running Linux, and compares when to choose full Linux, tiny Linux builds, or no-MMU options for robotics and embedded systems.

---

## 🧠 Core idea

The key requirement is not “Can the CPU run code?”; it is “Can the CPU provide **Linux’s memory model**?”

For normal Linux behavior, you generally need:

- a paged MMU,
- standard Linux boot support,
- enough RAM to hold kernel + drivers + rootfs + runtime,
- storage for kernel + init/system images.

When MMU is missing, Linux can still be built for constrained (`NOMMU`) targets, but it is a restricted compatibility mode.

---

## ⚙️ What standard Linux requires

At minimum, Linux can target most general-purpose 32/64-bit architectures with a PMMU and a proper compiler toolchain port.

Practical platform requirements:

- **MMU**: required for typical Linux process isolation and full userspace assumptions.
- **Boot entry**: firmware/bootloader path that your target architecture supports (BIOS/UEFI, U-Boot, EFI stub, TF-A, etc.).
- **RAM**:
  - **128 MB+**: extremely minimal embedded/learning images can start here.
  - **256–512 MB+**: minimum practical band for stable base Linux with a few services.
  - **512 MB–1 GB+**: realistic for Python utilities, ROS middleware basics, and small networking stacks.
- **Storage**:
  - **4–8 GB** for very stripped images.
  - **16+ GB** for comfortable package updates and logs.
- **CPU features**:
  - architecture ported by kernel + toolchain,
  - sane interrupt/clocking support,
  - enough cache and DMA path quality for your workload.

These are not fixed across all distros; they are practical engineering starting points.

---

## 🧪 No-MMU Linux (historically uClinux-style)

NOMMU Linux can run on architectures without an MMU, but kernel behavior is intentionally constrained.

Common differences:

- no normal `fork()`, and `clone()` typically needs `CLONE_VM`,
- restricted `mmap()` behavior,
- weaker inter-process protection,
- less reliable paging-like behavior,
- heavier constraints on contiguous memory and fragmentation,
- generally no swap/page-fault-driven memory expansion model.

Use `NOMMU` only when you need Linux APIs on an MMU-less class of hardware and can design for its limits.

For most teams, no-MMU Linux is better classified as “legacy-compatible” rather than “easy default”.

---

## 📦 Tiny Linux stacks: Buildroot / OpenWrt style

These are useful when you still want Linux user-space style APIs but cannot afford a full desktop-like image.

### Buildroot (custom)

Buildroot exists to make a fully custom rootfs, kernel, and bootloader for embedded boards.

- Use for: fixed-purpose robotics edge nodes, sensor gateways, small industrial gateways.
- Typical starting point: BusyBox userspace + small custom daemons.
- RAM tends to be low but still usually practical at **32–128 MB+** depending on services.
- Flash budget depends on chosen userspace; compressed images can stay relatively small.

### OpenWrt / router-style images

Router-class OpenWrt guidance commonly treats tiny RAM/flash devices as heavily constrained:

- 64 MB RAM can work for very basic roles, but is often marginal.
- 128 MB RAM is much safer for practical packages and updates.
- larger flash is required as you add packages and TLS/network stacks.

In other words, these paths are viable, but every package you add consumes both RAM and flash headroom.

---

## 🧭 Processor-to-stack decision matrix

| Target goal | Recommended stack | Why |
|---|---|---|
| Basic shell + ssh + simple scripts | Standard Linux + minimal distro | Simpler toolchain + debugging |
| Small robot with Python services + networking | Standard MMU Linux, stripped distro | `fork`, libraries, package manager, robust tooling |
| Edge ML + multi-process + logs + containers | Standard Linux, more RAM (>=1 GB) | Requires stable VM + cgroups/containers |
| Motor control + deterministic timing + no MMU hardware | RTOS or bare-metal, avoid full Linux | Better timing guarantees than Linux/MMU |
| Sensor gateway on very cheap MCU-class board | `NOMMU` Linux or MicroPython path | MMU absent, must accept constrained API set |
| Hard real-time + high reliability in one box | Linux + RT tuning + MMU (or dedicated RTOS path) | Choose based on latency budget and certification |

---

## 🧠 Python, MicroPython, and what “small” means

If your target has MMU and enough RAM, standard Linux can run CPython comfortably.

If target is MMU-less:

- you lose a big part of the standard Linux memory/process model,
- C/C++ or RTOS firmware is often cleaner,
- **[[MicroPython]]** is the practical Python option for constrained devices.

MicroPython is explicitly designed for microcontrollers with limited RAM/flash and has memory-management patterns around tiny heaps and frozen modules.

---

## ✅ Must-have minimum checklist

- MMU presence:
  - yes → standard Linux path,
  - no → investigate `NOMMU` or non-Linux firmware.
- RAM headroom:
  - enough for kernel + page tables + service footprint (not just “idle state”).
- Storage headroom:
  - kernel + rootfs + overlays/journals + temporary buffers.
- Boot chain:
  - power-on init, bootloader, kernel args, device tree (or equivalent board support).
- Peripheral model:
  - serial, GPIO, network, SPI/I2C, DMA characteristics for your sensors/actuators.

---

## 🧪 Use Cases in Robotics

- **Robotics perception node**: full Linux with MMU unless footprint is extremely constrained.
- **Navigation and actuation loop**: RTOS + small firmware often wins determinism.
- **Mixed architecture robots**: run Linux only where it is needed; keep hard control on MCU firmware.
- **Field device in low-power mode**: sometimes `NOMMU` + strict C code is the only viable path.

---

## 🔗 Related Concepts

- [[Linux]]
- [[MMU]]
- [[IOMMU]]
- [[MicroPython]]
- [[RTOS]]
- [[OpenWrt]]
- [[ROS2]]
- [[PCIe]]

---

## 📚 Further Reading

- [Linux kernel: MMU expectation in README](https://docs.kernel.org/6.8/admin-guide/README.html)
- [Linux `NOMMU` memory mapping behavior](https://docs.kernel.org/admin-guide/mm/nommu-mmap.html)
- [Linux memory management overview](https://docs.kernel.org/admin-guide/mm/concepts.html)
- [OpenWrt supported device requirements (practical RAM/flash guidance)](https://openwrt.org/supported_devices/864_warning)
- [Buildroot manual](https://buildroot.buildroot.org/downloads/manual/manual.html)
- [MicroPython on microcontrollers (resource constrained environments)](https://docs.micropython.org/en/latest/reference/constrained.html)
