# UEFI (Unified Extensible Firmware Interface)

UEFI is the modern replacement for the traditional BIOS. It provides a firmware interface between the operating system and the platform firmware, enabling faster booting, secure boot features, larger disk support, and a more flexible pre-boot environment. UEFI is now the standard firmware interface on most modern computers.

---

## ⚙️ Overview

- **Role**: Acts as the low-level interface that initializes hardware and hands control to the operating system’s bootloader (e.g., [[GRUB]])  
- **Replaces BIOS**: Provides more advanced functionality and a modular, extensible architecture  
- **Storage**: Boot entries and firmware data are stored in non-volatile memory (NVRAM)  

---

## 🧠 Core Concepts

- **EFI System Partition (ESP)**: A special FAT32 partition that stores bootloaders, drivers, and system utilities  
- **Secure Boot**: Ensures only signed and trusted bootloaders/kernels can run  
- **Boot Manager**: Manages multiple boot entries for OSes and recovery tools  
- **NVRAM Variables**: Store boot order, device paths, and configuration info  

---

## 📊 Comparison Chart

| Feature         | UEFI                         | BIOS                        |
|-----------------|------------------------------|-----------------------------|
| Disk support    | GPT (up to 9.4 ZB)           | MBR (max 2 TB)              |
| Boot speed      | Faster, parallel init        | Slower, sequential init     |
| Security        | Secure Boot supported        | None                        |
| Interface       | GUI possible, mouse support  | Text-only                   |
| Extensibility   | Drivers, apps, networking    | Very limited                |

---

## 🛠️ Useful UEFI Commands

Most commands are run via `efibootmgr` in Linux (package must be installed):

- `efibootmgr` → List current boot entries  
- `efibootmgr -v` → List boot entries with details  
- `efibootmgr -o 0003,0001,0002` → Set custom boot order  
- `efibootmgr -b 0001 -B` → Delete a specific boot entry  
- `efibootmgr -c -d /dev/sda -p 1 -L "MyLinux" -l '\EFI\ubuntu\grubx64.efi'` → Add a new boot entry  

---

## 🔧 Common File Locations

- `/sys/firmware/efi/` → Confirms system is booted in UEFI mode  
- `/boot/efi/EFI/` → EFI System Partition (stores loaders like `grubx64.efi`, `bootmgfw.efi`)  
- `/etc/fstab` → Mount point for ESP usually `/boot/efi`  
- NVRAM → Stores boot order and UEFI variables  

---

## ✅ Pros

- Supports modern hardware standards  
- Faster and more secure boot process  
- Large disk and GPT support  
- Extensible and modular (network stack, drivers)  

---

## ❌ Cons

- More complex than BIOS  
- Secure Boot can interfere with custom kernels or unsigned bootloaders  
- Some vendor implementations are buggy or inconsistent  

---

## 🔗 Related Concepts/Notes

- [[BIOS]]  
- [[GRUB]]  
- [[Linux Kernel]]  
- [[Secure Boot]]  
- [[EFI System Partition]]  

---

## 📚 Further Reading

- [UEFI Forum Specifications](https://uefi.org/specifications)  
- [Arch Wiki – UEFI](https://wiki.archlinux.org/title/Unified_Extensible_Firmware_Interface)  
- [Microsoft UEFI Documentation](https://learn.microsoft.com/en-us/windows-hardware/drivers/bringup/uefi)  
