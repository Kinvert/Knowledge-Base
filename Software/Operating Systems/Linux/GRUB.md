# GRUB (GNU GRUB Bootloader)

GRUB (GRand Unified Bootloader) is the default bootloader used in many Linux distributions. It is responsible for loading the kernel and passing kernel parameters at boot time. GRUB allows customization through configuration files and supports advanced features like dual booting, rescue mode, and passing boot parameters.

---

## ⚙️ Overview

- **Role**: Loads the Linux kernel and initial RAM disk  
- **Location**: Installed in the Master Boot Record (MBR) or EFI partition  
- **Config files**: Main settings live in `/etc/default/grub`, while full configs are generated in `/boot/grub/grub.cfg`  

---

## 🧠 Core Concepts

- **Kernel Parameters**: Options passed to the Linux kernel at boot  
- **Config Edits**: Made in `/etc/default/grub`, then applied with `sudo update-grub` (Debian/Ubuntu) or `sudo grub2-mkconfig -o /boot/grub2/grub.cfg` (Fedora/RedHat)  
- **Menu Entry**: GRUB menu entries can be edited at boot time by pressing `e`  

---

## 📊 Comparison Chart

| Bootloader | Common Use | Config File | Features |
|------------|------------|-------------|----------|
| GRUB       | Default Linux bootloader | `/etc/default/grub` | Kernel params, dual boot |
| LILO       | Legacy Linux bootloader | `/etc/lilo.conf` | Simple, outdated |
| systemd-boot | Lightweight EFI boot | `/boot/loader/entries/` | Simple EFI integration |
| rEFInd     | EFI boot manager | `/EFI/refind/refind.conf` | GUI, Mac-friendly |

---

## 🛠️ Common File Locations

- `/etc/default/grub` → Main configuration file (editable)  
- `/boot/grub/grub.cfg` → Auto-generated config file (do **not** edit manually)  
- `/boot/efi/EFI/` → EFI partition where GRUB resides in UEFI systems  

---

## 🖥️ Useful GRUB Commands

- `grub-install /dev/sda` → Install GRUB to MBR of disk  
- `update-grub` → Regenerate GRUB config (Debian/Ubuntu)  
- `grub2-mkconfig -o /boot/grub2/grub.cfg` → Regenerate config (RHEL/Fedora)  
- `grub-mkpasswd-pbkdf2` → Generate encrypted password for GRUB menu  
- `ls (hd0,gpt1)/` → Explore partitions from GRUB command line  

---

## 🔧 Common Kernel Parameters

These can be temporarily added by pressing `e` on the GRUB menu or permanently added in `/etc/default/grub` (`GRUB_CMDLINE_LINUX_DEFAULT`):

- `nomodeset` → Disables kernel mode setting (useful for GPU issues)  
- `iommu=off` → Disables IOMMU (helps with some hardware passthrough problems)  
- `quiet` → Reduces boot log messages  
- `nosplash` → Disables graphical boot splash  
- `acpi=off` → Disables ACPI (may fix some hardware boot issues but breaks power management)  
- `noapic` → Disables APIC (useful for some boot hangs)  
- `nolapic` → Disables Local APIC  
- `pci=noaer` → Disables PCIe Advanced Error Reporting  
- `rd.blacklist=nouveau` → Blacklists Nouveau GPU driver (NVIDIA troubleshooting)  
- `single` → Boots into single-user mode  
- `systemd.unit=rescue.target` → Boots into rescue mode  

---

## ✅ Pros

- Widely supported across Linux distributions  
- Powerful configuration options  
- Supports dual/multi-boot systems  
- Works with BIOS and UEFI  

---

## ❌ Cons

- Config file can be intimidating for beginners  
- Mistakes in GRUB config can make a system unbootable  
- Kernel parameters often trial-and-error for hardware issues  

---

## 🔗 Related Concepts/Notes

- [[Linux Kernel]]  
- [[UEFI]]  
- [[BIOS]]  
- [[Systemd]]  

---

## 📚 Further Reading

- [GNU GRUB Manual](https://www.gnu.org/software/grub/manual/)  
- [Arch Wiki – GRUB](https://wiki.archlinux.org/title/GRUB)  
- [Ubuntu GRUB Guide](https://help.ubuntu.com/community/Grub2)  
