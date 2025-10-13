# Diagnosing USB Storage Failures and System Freezes in Linux 💾

Unexpected freezes and USB errors during data transfers are common warning signs of instability in the USB subsystem or connected devices. This document provides a deep dive into analyzing, interpreting, and safely responding to USB storage failures—especially when external hard drives like Western Digital My Passport or Easystore disconnect or corrupt under Linux.

---

## ⚙️ Overview

When USB storage devices misbehave, Linux kernel logs (`dmesg`) often show repeating errors such as `device descriptor read/64, error -71`, `device not accepting address`, or `Cannot enable. Maybe the USB cable is bad?`.  
Such errors indicate failures in low-level USB communication, not necessarily a dead drive. However, persistent issues can freeze the OS as kernel threads wait on I/O completion.

This guide explains:
- What those kernel errors mean  
- How to distinguish between hardware and driver faults  
- Safe recovery procedures for possibly corrupted drives  
- How to verify whether your computer’s USB subsystem is trustworthy for critical data

---

## 🧠 Core Concepts

- **USB Enumeration:** The process by which the OS identifies and configures a USB device. Failures here prevent block device creation (`/dev/sdX`).  
- **UAS (USB Attached SCSI):** A newer, faster driver replacing `usb-storage`, but often buggy in consumer USB-to-SATA bridges.  
- **usb-storage:** The legacy USB mass storage driver, slower but usually stable.  
- **I/O Freeze:** A kernel stall triggered by faulty USB communication, often unresolvable without power cycling.  
- **Filesystem Corruption:** Occurs when power loss or kernel freeze interrupts active writes, especially on NTFS or exFAT volumes.

---

## 🧰 Troubleshooting Procedure

1. **Unplug all USB storage devices.**  
   Prevents cascading failures across ports sharing a controller.

2. **Inspect kernel logs:**  
   Use `sudo dmesg -T | tail -n 200` to find entries like:
   - `usb 3-2: device descriptor read/64, error -71`
   - `usb 3-2: device not accepting address 4, error -71`
   - `usb usb3-port2: Cannot enable. Maybe the USB cable is bad?`
   - `usb 4-5: new SuperSpeed Gen 1 USB device number 2 using xhci_hcd`
   - `usb 4-5: USB disconnect, device number 2`

3. **Change physical connections:**  
   - Swap to a known-good USB 3.0 cable  
   - Use a different USB controller (rear I/O instead of front ports)  
   - Try a powered USB hub for adequate power supply  
   - Test the device on another computer

4. **Verify driver binding:**  
   Run `lsusb` to check the device ID (e.g., `1058:264d` for WD Easystore).  
   If `uas` is listed in `dmesg`, temporarily disable it:  
   `sudo modprobe -r uas && sudo modprobe usb_storage`

5. **Check device recognition:**  
   `lsblk -o NAME,SIZE,MODEL,VENDOR,TRAN,MOUNTPOINT`  
   Confirms if the kernel created `/dev/sdX`.

6. **Identify filesystem and partitions:**  
   `sudo blkid`  
   If the filesystem appears as `ntfs`, `exfat`, or `unknown`, mount it *read-only* only.

7. **Create a disk image before repair:**  
   Use `sudo ddrescue -n /dev/sdX ~/easystore.img ~/easystore.log`  
   This preserves all recoverable sectors before attempting repair.

---

## 🧩 When You Can or Can’t Trust Your USB System

| Situation | Trust Level | Notes |
|------------|--------------|-------|
| Stable enumeration across multiple devices and ports | ✅ Reliable | System USB hardware likely fine |
| Repeated `error -71` or `device not accepting address` on all drives | ❌ Unreliable | Motherboard or controller issue |
| Errors only on one enclosure or cable | ⚠️ Partial | Replace cable/enclosure before trusting |
| Kernel freezes under sustained I/O | ❌ Untrustworthy | Indicates USB driver or controller lockup |
| Works fine under `usb-storage` but fails with `uas` | ⚠️ Stable (with quirks) | Permanently disable `uas` for that device |

---

## ⚖️ Comparison Chart

| Feature | `usb-storage` | `uas` |
|----------|----------------|-------|
| Speed | Slower | Faster |
| Stability | High | Medium–Low (depends on bridge firmware) |
| Error Recovery | Simple retries | Complex queueing prone to hangs |
| Recommended For | Legacy and critical data recovery | New, well-tested enclosures |
| Common Issues | Slightly slower throughput | Kernel hangs, timeouts, disconnects |

---

## 🧱 Safe Recovery Steps (Filesystem Corruption)

1. **Never run `fsck` directly on the damaged drive.** Always work from a cloned image.  
2. **Mount read-only:** `sudo mount -o ro /dev/sdX1 /mnt/test`  
3. **Inspect the filesystem:**  
   - NTFS: `ntfsfix --no-action /dev/sdX1`  
   - exFAT: `fsck.exfat -n /dev/sdX1`  
4. **Recover files from the image:** Use `testdisk` or `photorec` for deep recovery.  
5. **Reformat only after successful recovery.**

---

## 🧯 Verifying System Reliability

After resolving the issue:
- Test multiple USB storage devices under load (`dd if=/dev/zero of=/mnt/testfile bs=1M count=10000`)  
- Observe `dmesg` for I/O or reset errors.  
- If freezes persist, test memory (`memtest86`), inspect PSU rails, and update the motherboard’s BIOS.  
- Persistent controller-level faults mean the system should no longer handle primary storage tasks.

---

## 🔗 Related Concepts

- [[Filesystem Corruption]] (Causes and repair strategies)  
- [[ddrescue]] (GNU tool for block-level data recovery)  
- [[USB Protocol]] (Low-level communication and enumeration)  
- [[UAS]] (USB Attached SCSI protocol)  
- [[NTFS]] (Filesystem type commonly affected by improper shutdowns)  
- [[exFAT]] (Filesystem prone to corruption on unexpected disconnect)  
- [[TestDisk]] (Partition recovery utility)  
- [[SMART]] (Drive health monitoring)  

---

## 📚 Further Reading

- Linux Kernel Documentation: `Documentation/usb/error-codes.txt`  
- Western Digital Support KB: "My Passport drive not recognized in Linux"  
- GNU ddrescue Manual  
- ArchWiki: "USB Storage Troubleshooting"

---
