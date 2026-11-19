# MMU

The **MMU** (Memory Management Unit) is the processor feature that translates virtual addresses to physical RAM addresses. Without it, every process does not get an independent address space in the usual Linux sense.

In practice, the MMU is what makes virtual memory, process isolation, and safe multitasking feasible on a single Linux system.

---

## 🧠 Overview

- **Translation**: map process virtual addresses → RAM addresses via page tables and a **TLB**.
- **Isolation**: prevent one process from reading or writing another process’s memory.
- **Memory management features**: swapping, copy-on-write, demand paging, overcommit, and memory accounting.
- **Kernel flexibility**: user processes can be loaded at different virtual addresses each run without breaking.

Linux assumes these behaviors when it manages memory and scheduling. That is why modern Linux ports generally expect a paged MMU.

---

## 🛠️ Why Linux cares about MMU

Linux runs many components as separate processes/users. An MMU enables:

- `fork()` / `clone()` semantics where processes start with shared read-only pages.
- `mmap()` file and anonymous mappings with per-process permissions.
- per-process stack/heap protections (`rwx` and guard patterns).
- safe device DMA setup with `mmap()`-like user-space access patterns.
- OOM handling and reclaim under pressure.

The official kernel docs summarize this baseline as: Linux is portable to most general-purpose architectures **with a paged MMU**, and only partially to architectures without one.

---

## 🧬 MMU vs No-MMU Linux

No-MMU Linux exists as a constrained path in kernel support (`NOMMU`); historically that world is associated with uClinux. It runs, but features are limited.

`NOMMU` impact highlights:

- `fork()` is unavailable in the usual way.
- `clone()` typically needs `CLONE_VM`.
- memory mappings must be physically contiguous in many cases.
- performance-sensitive operations like `mmap()` may be more constrained and fragmentation-sensitive.
- no full virtual memory model, so cross-process memory protection is weaker.

For many robotics workloads, the practical limitation is not only RAM, but the missing process model that normal Linux tooling assumes.

---

## 🔗 MMU, IOMMU, and MPU

People often mix up these terms:

- **MMU**: process-level CPU address translation and protection.
- **IOMMU**: similar translation for devices’ DMA addresses (same idea, different endpoint).
- **MPU** (Memory Protection Unit): coarse-grained region protection often used in MCUs, not equivalent to an MMU.

---

## 📊 Comparison Chart

| Feature | MMU Linux | No-MMU Linux (NOMMU) | IOMMU | MPU |
|---|---|---|---|---|
| Process virtual addresses | ✔ full | ✖ limited | ✖ (device-side) | ✖ |
| Process isolation | Strong | Weak/missing | Strong for DMA by design | Region-limited |
| `fork()` semantics | Full | `fork()` not in normal form | N/A | N/A |
| `mmap` behavior | Full POSIX-style | Constrained/partial | Device I/O mapping contexts | N/A |
| Swap/page cache | Supported | Usually constrained | N/A | N/A |
| Security boundaries | High (with page permissions) | Lower | High for device DMA | Moderate |
| Typical use | Linux/ROS/devices | legacy embedded Linux niches | virtualization, passthrough, DMA safety | microcontrollers, real-time firmware |

---

## 🧪 What to check on Linux

Quick indicators that MMU is present and active:

```bash
dmesg | grep -i mmu
grep -i --color=always -E "mmu|tlb" /proc/cpuinfo 2>/dev/null
cat /proc/meminfo | head
```

If you see MMU-related lines in `dmesg` and normal process behavior (`fork`, virtual memory errors, separate `/proc/<pid>/maps` behavior), your hardware is likely running in MMU mode.

---

## ✅ Robotics implications

- Real-time control loops: MMU is not required for deterministic bare-metal timing, but it is required for normal Linux process isolation and tooling.
- Sensor fusion + Linux middleware stacks: usually practical only with MMU.
- If you do not have MMU, move toward RTOS/Firmware + [[MicroPython]]/C and keep Linux expectations to a minimum.

---

## 🔗 Related Notes

- [[IOMMU]]
- [[Linux]]
- [[Linux Requirements]]
- [[CPUs]]
- [[RTOS]]
- [[MicroPython]]

---

## 📚 Further Reading

- [Kernel release docs note: MMU expectation](https://docs.kernel.org/6.8/admin-guide/README.html)
- [Linux no-MMU mapping behavior](https://docs.kernel.org/admin-guide/mm/nommu-mmap.html)
- [Linux memory management concepts](https://docs.kernel.org/admin-guide/mm/concepts.html)
- [IOMMU on Linux](https://www.kernel.org/doc/html/v6.6/driver-api/vfio.html)
