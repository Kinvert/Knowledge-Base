# dd

`dd` is a low-level command-line utility in Unix-like systems used to copy and convert raw data between files, block devices, or streams. It operates at the byte level, making it powerful for creating disk images, cloning drives, wiping devices, and performing precise data manipulation. However, it is not optimized for failing hardware and should be used with caution when dealing with corrupted or degraded disks.

---

## 🧭 Overview

Originally designed for data conversion, `dd` has become a core Unix tool for tasks involving exact disk and file operations. It can clone entire disks, create bootable USBs, or back up partitions. Its ability to read and write directly to devices makes it extremely flexible but also potentially dangerous if misused.

---

## ⚙️ Core Concepts

- **Input File (`if=`)**: The source (file, disk, or partition).
- **Output File (`of=`)**: The destination (file, disk, or partition).
- **Block Size (`bs=`)**: Size of data chunks read/written per operation.
- **Count (`count=`)**: Number of blocks to copy.
- **Skip/Seek**: Skip input/output blocks, useful for partial copying.
- **Conversion Options (`conv=`)**: Modify data during transfer (e.g., `noerror`, `sync`).

---

## 🔍 How It Works

`dd` performs direct, unbuffered I/O at the block level. For example:
- Copying a full disk: `dd if=/dev/sdX of=/dev/sdY bs=64K conv=noerror,sync`
- Creating an image: `dd if=/dev/sdX of=backup.img bs=4M`
- Restoring an image: `dd if=backup.img of=/dev/sdX bs=4M`

---

## 📊 Comparison Chart

| Tool        | Use Case                   | Strengths                           | Weaknesses |
|-------------|----------------------------|-------------------------------------|-------------|
| dd          | Disk cloning, imaging      | Simple, flexible, always available  | Stops on errors, no progress log |
| ddrescue    | Failing disk recovery      | Error handling, logfile support     | Requires installation, CLI |
| cat         | Simple file copy           | Minimal overhead                    | No block control, no conversion |
| Clonezilla  | Full-disk backup/restore   | User-friendly, partition aware      | More complex setup |
| cp          | File-level copying         | Easy, fast                          | No raw device support |

---

## 📂 Common Scenarios

- **Disk Imaging**: Back up a whole disk to a single file for later restoration.
- **Bootable USB Creation**: Write ISO images directly to USB drives.
- **Data Wiping**: Overwrite a disk with zeros or random data: `dd if=/dev/zero of=/dev/sdX bs=1M`
- **Partial Recovery**: Copying a section of a disk using `skip` and `count`.

---

## ✅ Strengths

- Available on nearly all Unix-like systems by default
- Precise control over data transfer
- Supports both file and raw device operations
- Useful for backups, boot media, and forensic imaging

---

## ❌ Weaknesses

- Stops on read errors (not robust for failing hardware)
- No built-in progress output (requires `status=progress` or external tools like `pv`)
- Easy to accidentally overwrite the wrong device
- No recovery strategies (unlike [[ddrescue]])

---

## 🔧 Example Usage

- Backup entire disk:  
  `sudo dd if=/dev/sdX of=/path/to/disk.img bs=4M status=progress`
- Restore disk from image:  
  `sudo dd if=/path/to/disk.img of=/dev/sdX bs=4M status=progress`
- Wipe a disk:  
  `sudo dd if=/dev/zero of=/dev/sdX bs=1M status=progress`
- Extract boot sector:  
  `sudo dd if=/dev/sdX of=bootsector.bin bs=512 count=1`

---

## 📚 Related Concepts/Notes

- [[ddrescue]] (Error-tolerant alternative to dd)
- [[Disk Data Recovery]] (General strategies)
- [[File Systems]] (Logical structures on disks)
- [[ext4]] (Linux file system)
- [[NTFS]] (Windows file system)
- [[Clonezilla]] (Backup and restore suite)

---

## 🌐 External Resources

- GNU coreutils manual (dd): https://www.gnu.org/software/coreutils/manual/html_node/dd-invocation.html
- man page: `man dd`
- ArchWiki guide on disk cloning: https://wiki.archlinux.org/title/Disk_cloning
- https://serverfault.com/questions/4906/using-dd-for-disk-cloning

---

## 📝 Summary

`dd` is a versatile and fundamental Unix utility for low-level copying and conversion. It excels at tasks like creating disk images, writing ISOs, or wiping devices, but it is not a recovery tool for failing hardware. For data preservation in the face of errors, [[ddrescue]] should be preferred. Still, `dd` remains invaluable for system administration, forensics, and engineering workflows.

---
