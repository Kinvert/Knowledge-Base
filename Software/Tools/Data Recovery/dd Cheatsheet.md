# dd Cheatsheet

The `dd` command is a versatile Unix/Linux utility for copying and converting raw data at the block level. It is commonly used for disk imaging, cloning, backups, and low-level operations. Due to its power, misuse can lead to permanent data loss — always double-check `if=` (input file) and `of=` (output file).

---

## 🧭 Overview

`dd` reads from an input file or device and writes to an output file or device. By adjusting parameters like block size, skip, and count, it provides precise control over data transfer. It is particularly useful in system administration and forensics.

---

## 🔑 Common Syntax

`dd if=<source> of=<destination> [options]`

- `if=` → Input file/device (e.g., `/dev/sdX`)
- `of=` → Output file/device (e.g., `/dev/sdY` or `backup.img`)
- `bs=` → Block size (e.g., `bs=4M`)
- `count=` → Number of blocks to copy
- `skip=` → Skip blocks from input
- `seek=` → Skip blocks on output
- `conv=` → Conversion options (e.g., `noerror`, `sync`)

---

## 📂 Common Use Cases

- **Create disk image**:  
  `sudo dd if=/dev/sdX of=/path/to/disk.img bs=4M status=progress`

- **Restore disk from image**:  
  `sudo dd if=/path/to/disk.img of=/dev/sdX bs=4M status=progress`

- **Create bootable USB from ISO**:  
  `sudo dd if=image.iso of=/dev/sdX bs=4M status=progress oflag=sync`

- **Wipe disk with zeros**:  
  `sudo dd if=/dev/zero of=/dev/sdX bs=1M status=progress`

- **Wipe disk with random data**:  
  `sudo dd if=/dev/urandom of=/dev/sdX bs=1M status=progress`

- **Backup MBR (first 512 bytes)**:  
  `sudo dd if=/dev/sdX of=mbr.img bs=512 count=1`

- **Restore MBR**:  
  `sudo dd if=mbr.img of=/dev/sdX bs=512 count=1`

- **Test write speed of a disk**:  
  `dd if=/dev/zero of=testfile bs=1G count=1 oflag=dsync`

---

## ⚙️ Useful Options

- `status=progress` → Show progress (modern versions)
- `conv=noerror` → Continue after read errors
- `conv=sync` → Pad blocks with zeros if read is short
- `iflag=direct oflag=direct` → Bypass cache for raw benchmarking

---

## 📊 Comparison: dd vs ddrescue

| Feature              | dd                   | ddrescue                |
|----------------------|----------------------|--------------------------|
| Error Handling       | Stops on error       | Skips bad blocks, retries|
| Logging              | None                 | Uses logfile (mapfile)   |
| Progress             | `status=progress`    | Built-in detailed output |
| Best Use             | Healthy disks, imaging | Failing disks, recovery |

---

## 📚 Related Concepts/Notes

- [[ddrescue]] (Error-tolerant alternative for failing disks)
- [[dd]]
- [[Disk Data Recovery]] (General strategies)
- [[Partition Magic ISO]] (Legacy partition tool)
- [[File Systems]] (Logical data organization)
- [[Clonezilla]] (Backup/restore suite)

---

## 🌐 External Resources

- man page: `man dd`
- GNU coreutils dd: https://www.gnu.org/software/coreutils/manual/html_node/dd-invocation.html
- ArchWiki disk cloning: https://wiki.archlinux.org/title/Disk_cloning

---

## 📝 Summary

`dd` is a powerful block-level utility for cloning, imaging, and wiping storage devices. It is best suited for healthy hardware and routine imaging tasks. For recovery from failing disks, [[ddrescue]] is the safer choice. Always verify commands carefully before execution to avoid overwriting valuable data.

---
