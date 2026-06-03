# IOMMU

**IOMMU** stands for **Input-Output Memory Management Unit**. It is the DMA-side equivalent of a CPU [[MMU]]: instead of translating CPU virtual addresses, it translates and restricts memory accesses made by I/O devices such as [[GPUs]], [[NVMe]] drives, NICs, USB controllers, capture cards, and other [[PCIe]] devices.

An IOMMU lets the operating system or hypervisor give devices controlled access to memory. This is what makes modern PCI passthrough, VFIO, DMA isolation, and safer user-space drivers possible.

---

## Overview

Without an IOMMU, a DMA-capable device can often read or write large parts of physical memory directly. That is fast, but it is also dangerous: a buggy or malicious device/driver can corrupt the kernel, leak secrets, or interfere with another VM.

With an IOMMU, device DMA goes through a translation and permission check:

```text
Device DMA address / IOVA
        |
        v
IOMMU translation + access control
        |
        v
System physical memory
```

The device thinks it is DMAing to an address in its own I/O virtual address space. The IOMMU maps that IOVA to real physical memory only if the OS or hypervisor has installed a valid mapping.

---

## Why It Matters

- **PCI passthrough / VFIO**: assign a physical GPU, NIC, NVMe controller, USB controller, or accelerator directly to a VM.
- **DMA attack mitigation**: restrict devices from reading arbitrary system memory.
- **User-space drivers**: allow frameworks like VFIO to expose devices to user space while keeping DMA contained.
- **Legacy DMA support**: let devices with limited DMA address width reach high memory through remapping.
- **Isolation between devices**: keep one device or VM from DMAing into another device's buffers.
- **Interrupt remapping**: route and validate device interrupts, not only memory writes.
- **Heterogeneous systems**: support accelerators, GPUs, and SoC devices that share memory with CPUs.

---

## Vendor Names

| Vendor / Architecture | IOMMU Name |
|---|---|
| Intel x86 | **VT-d**, or Intel Virtualization Technology for Directed I/O |
| AMD x86 | **AMD-Vi**, AMD I/O Virtualization Technology, or AMD IOMMU |
| Arm | **SMMU**, System Memory Management Unit |
| POWER | Partitionable Endpoints and platform-specific IOMMU facilities |
| RISC-V | RISC-V IOMMU extension / platform IOMMU implementations |

In BIOS/UEFI menus, the setting may be named `IOMMU`, `VT-d`, `AMD-Vi`, `SVM`, `Directed I/O`, or hidden under virtualization/security/chipset settings.

---

## Core Concepts

- **DMA**: Direct Memory Access. A device reads or writes system memory without CPU load/store instructions.
- **IOVA**: I/O Virtual Address. The address a device uses for DMA after the driver maps a buffer.
- **DMA address / bus address**: The address placed into a device's DMA descriptor or register.
- **Physical address**: Real system memory address after IOMMU translation.
- **Domain**: A translation context. Devices in the same domain share IOVA mappings.
- **IOMMU group**: The smallest set of devices the kernel believes can be isolated from all other devices.
- **Requester ID**: PCIe identity used by the IOMMU to tell which device generated a transaction.
- **ACS**: PCIe Access Control Services. Helps prevent peer-to-peer PCIe traffic from bypassing the IOMMU.
- **ATS / PRI / PASID**: PCIe features for advanced address translation and shared virtual addressing.

---

## How It Works

1. A driver allocates or pins memory.
2. The driver calls the OS DMA API, such as Linux `dma_map_single()` or `dma_map_sg()`.
3. The kernel chooses an IOVA and programs the IOMMU page tables.
4. The kernel returns a DMA address to the driver.
5. The driver writes that DMA address into the device.
6. The device performs DMA using the IOVA.
7. The IOMMU translates the IOVA to physical memory and checks permissions.
8. The driver unmaps the buffer when the device is done.

This indirection is why kernel drivers should use the generic DMA API instead of assuming that device-visible DMA addresses equal CPU physical addresses.

---

## IOMMU Groups

An **IOMMU group** is the unit of safe ownership for VFIO and PCI passthrough. If two devices are in the same group, the kernel treats them as not safely separable.

This happens because isolation is not only about the IOMMU itself. It also depends on PCIe topology:

- A multifunction device may allow internal traffic between functions.
- A PCIe switch or bridge may allow peer-to-peer redirection.
- A conventional PCI bridge can hide multiple devices behind one requester ID.
- Missing ACS support can mean transactions can avoid the IOMMU.

For passthrough, the practical rule is:

**Every non-bridge device in the IOMMU group must either be assigned to the VM or bound to a safe stub/VFIO driver.**

If a GPU is in a group with its HDMI audio function, pass both. If it is grouped with unrelated host-critical devices, passthrough may be unsafe or impossible without changing slots, BIOS settings, motherboard, kernel patches, or topology.

---

## VFIO

**VFIO** is the Linux framework for secure direct device access from user space and VMs. It replaced older direct-assignment approaches and is commonly used with KVM/QEMU.

VFIO depends on IOMMU isolation because a passed-through device behaves like a user-space driver from the host kernel's perspective. Without an IOMMU, that device could DMA anywhere in host memory.

Typical VFIO use cases:

- GPU passthrough to a Windows or Linux VM.
- Passing a USB controller to a VM.
- Low-latency NIC or accelerator access.
- User-space drivers for high-performance devices.

Newer Linux kernels also include **IOMMUFD**, a newer user API for managing I/O page tables. It is intended to support advanced DMA features and eventually replace older VFIO container/IOMMU plumbing.

---

## Linux Setup

Enable IOMMU in firmware first:

- Intel: enable `VT-d` / `Directed I/O`.
- AMD: enable `IOMMU` / `AMD-Vi`; sometimes also enable `SVM` for virtualization.

Common Linux kernel command-line parameters:

```text
intel_iommu=on
amd_iommu=on
iommu=pt
iommu.passthrough=0
iommu.passthrough=1
iommu.strict=1
iommu.strict=0
```

Parameter notes:

- `intel_iommu=on` enables the Intel IOMMU driver.
- `amd_iommu=on` is commonly used in distro guides, but AMD IOMMU may already initialize automatically when firmware exposes it.
- `iommu=pt` uses passthrough identity mappings for host devices where possible, often to reduce overhead while keeping IOMMU available for assigned devices.
- `iommu.passthrough=0` means translated DMA by default.
- `iommu.passthrough=1` means bypass IOMMU by default.
- `iommu.strict=1` favors stronger isolation by invalidating IOMMU TLB entries synchronously.
- `iommu.strict=0` uses lazy invalidation when supported, improving throughput at some isolation cost.

Check whether Linux sees IOMMU:

```bash
dmesg | grep -Ei 'DMAR|IOMMU|AMD-Vi|VT-d'
find /sys/kernel/iommu_groups/ -type l
ls /sys/kernel/iommu_groups/
```

List IOMMU groups:

```bash
for g in /sys/kernel/iommu_groups/*; do
  echo "IOMMU Group ${g##*/}:"
  for d in "$g"/devices/*; do
    lspci -nns "${d##*/}"
  done
done
```

---

## GPU Passthrough Checklist

1. Enable IOMMU / VT-d / AMD-Vi in firmware.
2. Enable CPU virtualization support if needed: Intel VT-x or AMD SVM.
3. Boot Linux with IOMMU enabled.
4. Confirm `/sys/kernel/iommu_groups/` exists and has groups.
5. Identify the GPU and its companion functions, usually VGA/3D + HDMI audio.
6. Confirm the target devices are isolated in a viable group.
7. Bind the target devices to `vfio-pci` early enough that the host driver does not claim them.
8. Pass every required device/function to the VM.
9. Avoid passing host root ports or bridges unless you know exactly why.
10. Keep a recovery path: SSH, second GPU, iGPU, or boot entry without VFIO binding.

---

## Performance

IOMMUs add translation work and IOTLB pressure, but the overhead is often small compared with the benefit of isolation. Performance depends on:

- DMA mapping churn.
- IOTLB size and invalidation behavior.
- Scatter-gather depth.
- Page size used for mappings.
- Whether host devices use passthrough/identity domains.
- ATS/PASID support for advanced devices.
- NUMA topology and PCIe root complex placement.

Common tuning tradeoffs:

| Mode | Benefit | Risk / Cost |
|---|---|---|
| Strict invalidation | Stronger isolation after unmap | More overhead |
| Lazy invalidation | Better throughput | Weaker immediate revocation |
| Passthrough domains | Lower overhead for trusted host devices | Less DMA isolation for those devices |
| Huge IOMMU pages | Lower IOTLB pressure | Less granular mappings |

For security-sensitive systems, prefer translated/strict modes. For single-user performance systems, `iommu=pt` is common when VFIO still needs the IOMMU for assigned devices.

---

## Security Notes

IOMMU is necessary for DMA containment, but it is not magic.

- Devices in the same IOMMU group are not safely isolated from one another.
- Bad PCIe topology can defeat per-device isolation.
- ACS override patches can make groups look better without changing the hardware path. That may be useful for hobby passthrough, but it weakens the isolation model.
- Thunderbolt/USB4 DMA protection depends on firmware, OS policy, and IOMMU configuration.
- Passthrough devices are trusted with the guest; a malicious guest with a buggy device or firmware may still attack the host if isolation is incomplete.
- Interrupt remapping matters too; DMA remapping alone is not the whole story.

---

## Common Problems

| Symptom | Likely Cause | Fix |
|---|---|---|
| No `/sys/kernel/iommu_groups/` | IOMMU disabled or not exposed by firmware | Enable firmware setting, update BIOS, add kernel parameter |
| Empty or huge groups | Bad topology or missing ACS | Try another PCIe slot, update BIOS, use different board |
| VM fails because group is not viable | Some group device still bound to host driver | Bind all non-bridge group devices to VFIO/stub |
| Host loses display after binding GPU | Host was using that GPU | Use iGPU/second GPU/SSH/recovery boot entry |
| GPU reset failure after VM shutdown | Device reset quirk or driver issue | Vendor-reset module, newer kernel, different GPU, avoid repeated VM restarts |
| Poor storage/NIC performance | IOTLB pressure or strict invalidation overhead | Test `iommu=pt`, lazy mode, hugepages, newer kernel/firmware |
| Device can only do 32-bit DMA | Limited DMA mask | IOMMU remapping can place buffers above 4 GB while giving device a low DMA address |

---

## Related Concepts

- [[PCIe]]
- [[DMA]]
- [[MMU]]
- [[Virtualization]]
- [[KVM]]
- [[QEMU]]
- [[VFIO]]
- [[GPU Passthrough]]
- [[NVMe]]
- [[CPUs]]

---

## Further Reading

- [Linux Dynamic DMA mapping Guide](https://docs.kernel.org/6.9/core-api/dma-api-howto.html)
- [Linux VFIO documentation](https://docs.kernel.org/6.0/driver-api/vfio.html)
- [Linux VFIO / IOMMUFD documentation](https://www.kernel.org/doc/html/v6.6/driver-api/vfio.html)
- [Linux kernel command-line parameters](https://docs.kernel.org/6.10/admin-guide/kernel-parameters.html)
- [AMD I/O Virtualization Technology IOMMU Specification](https://docs.amd.com/api/khub/documents/GD6kOXjzWsek8QUbn_qMvg/content)
- [Intel VT-d Architecture Specification](https://www.intel.com/content/www/us/en/content-details/831418/intel-virtualization-technology-for-directed-i-o-architecture-specification.html)
- [Red Hat: A Deep-dive into IOMMU Groups](https://docs.redhat.com/en/documentation/red_hat_enterprise_linux/7/html/virtualization_deployment_and_administration_guide/sect-iommu-deep-dive)
- [Arch Wiki: PCI passthrough via OVMF](https://wiki.archlinux.org/title/PCI_passthrough_via_OVMF)
