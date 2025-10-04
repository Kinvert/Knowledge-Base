# Disk Data Recovery

Data recovery from disks is a common troubleshooting task that engineers and technically literate users encounter. Whether dealing with corrupted file systems, unmountable drives, or failed copies that cause sudden inaccessibility, knowing how to approach recovery systematically can make the difference between saving critical files and permanent data loss. This note covers both Windows and Linux environments, common scenarios, free tools, and practical strategies for home users.

---

## 🧭 Overview

Disk failures can stem from many causes: logical errors in file systems, hardware malfunctions, improper ejection, power loss during transfers, or even cross-platform usage (e.g., NTFS disks on Linux). Recovery efforts typically focus on:

- Diagnosing whether the issue is hardware or software
- Attempting safe mounts and read-only access
- Using file system repair tools
- Applying file carving and recovery utilities when repair fails

---

## 🛠️ Core Concepts

- **File System Corruption**: Occurs when metadata structures (e.g., NTFS Master File Table, ext4 journal) are damaged, preventing access.
- **Mounting**: The OS's attempt to make a disk accessible. If mounting fails, recovery tools may still access raw data.
- **Logical vs. Physical Damage**: Logical (corruption, partition errors) can often be repaired; physical (bad sectors, motor failure) often require professional services.
- **Read-Only Recovery**: Best practice is to avoid writing to the failing disk, to prevent overwriting recoverable data.
- **Cross-Platform Issues**: NTFS on Linux, ext4 on Windows, and exFAT inconsistencies can cause errors after improper use.

---

## 🔍 Example Case: NTFS External Drive Corruption

Scenario:
- NTFS 2TB external drive used primarily on Windows
- Plugged into Ubuntu 22, copied files, last file corrupted
- Drive became inaccessible: would not mount on Ubuntu, showed up but wouldn’t open on Windows 7
- `chkdsk` stalled at NTFS detection stage

**Analysis**:
- Likely NTFS metadata corruption (e.g., MFT or index records damaged during copy)
- Linux `lsusb` confirmed hardware visibility, meaning drive controller still works
- Windows recognized the drive letter but could not mount due to logical corruption

**Typical Recovery Steps**:
1. On Linux, attempt read-only mount: `sudo mount -t ntfs-3g -o ro /dev/sdX /mnt/recovery`
2. Use `ntfsfix` (Linux) for quick metadata repair: `sudo ntfsfix /dev/sdX`
3. On Windows, use `chkdsk /f` cautiously (may cause data loss on badly corrupted drives)
4. If repair fails, use recovery utilities like **TestDisk** (partition recovery) or **PhotoRec** (file carving)

---

## 📊 Comparison Chart

| Tool / Method       | Platform | Use Case                          | Strengths                           | Weaknesses |
|---------------------|----------|-----------------------------------|-------------------------------------|-------------|
| chkdsk              | Windows  | NTFS/FAT logical repair            | Built-in, easy to run                | Risk of overwriting damaged data |
| ntfsfix             | Linux    | Quick NTFS fix                     | Fast, safe to run read-only          | Limited repair ability |
| TestDisk            | Both     | Partition and file system recovery | Free, powerful, cross-platform       | Steep learning curve |
| PhotoRec            | Both     | File carving (ignores FS)          | Recovers from severe corruption      | Loses filenames, structure |
| ddrescue            | Linux    | Clone failing disks sector-by-sector | Preserves data for later recovery   | Requires space on second drive |
| Commercial tools (e.g., Recuva, EaseUS) | Windows | User-friendly recovery | GUI, automated workflows | Often limited in free versions |

---

## 📂 Common Scenarios

- **Unmountable Drive**: Shows in `lsusb` or Windows Device Manager but cannot open. Indicates logical corruption.
- **RAW Partition in Windows**: Drive letter appears, but Windows says “needs formatting.” Caused by damaged boot sector or MFT.
- **Clicking Drive**: Mechanical failure; stop using immediately and seek professional recovery.
- **Slow Access / Freezes**: Often due to bad sectors. Use `ddrescue` to image disk before further attempts.
- **Cross-OS Use**: NTFS on Linux or ext4 on Windows often leads to improper dismount issues.

---

## ✅ Strengths of DIY Recovery

- Free and widely available tools
- No risk of sending data offsite
- Possible to recover most logical corruption cases

---

## ❌ Weaknesses / Risks

- Risk of permanent loss if tools write to damaged disk
- Inexperienced use of `chkdsk` or formatting can overwrite data
- Physical failures almost never recoverable without professional tools

---

## 🔧 Compatible File Systems

- **Windows**: [[NTFS]], [[FAT32]], [[exFAT]]
- **Linux**: [[ext2]], [[ext3]], [[ext4]], [[XFS]], [[Btrfs]]
- **Cross-Platform**: exFAT (widely supported, but prone to corruption)

---

## 📚 Related Concepts/Notes

- [[File Systems]] (Overview of common file systems)
- [[NTFS]] (Windows file system)
- [[ext4]] (Linux journaling file system)
- [[Data Backup]] (Prevention strategies)
- [[RAID]] (Data redundancy methods)
- [[ddrescue]] (Disk cloning tool)
- [[chkdsk]]
- [[ntfsfix]]

---

## 🌐 External Resources

- TestDisk & PhotoRec: https://www.cgsecurity.org/
- GNU ddrescue: https://www.gnu.org/software/ddrescue/
- Microsoft Docs – chkdsk: https://docs.microsoft.com/en-us/windows-server/administration/windows-commands/chkdsk
- Ubuntu NTFS support: https://wiki.ubuntu.com/NTFS

---

## 📝 Summary

Disk data recovery involves balancing safe access with targeted repair. In the NTFS external drive case, metadata corruption from a failed copy made the drive inaccessible across both Linux and Windows. Free utilities like TestDisk, PhotoRec, and ntfsfix provide powerful ways to recover data without professional services. However, if the drive shows physical failure symptoms, the safest route is professional recovery.

---
