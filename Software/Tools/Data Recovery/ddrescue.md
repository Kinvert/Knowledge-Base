# ddrescue

GNU ddrescue is a free data recovery tool designed to copy data from one block device (like a failing hard disk or USB stick) to another, while handling errors gracefully. Unlike simple cloning tools, it is optimized for failing disks, skipping over bad sectors on the first pass and returning later to retry them. This makes it a cornerstone of disk recovery workflows.

---

## 🧭 Overview

`ddrescue` copies data from a source disk to a destination image or disk, intelligently managing errors. It creates a **log file** (mapfile) that tracks which sectors are good, bad, or pending, allowing recovery attempts to resume without losing progress.

---

## ⚙️ Core Concepts

- **Source and Destination**: Input (failing device) and output (disk image or new drive).
- **Logfile/Mapfile**: Records recovery progress so retries are efficient.
- **Pass Strategy**: 
  - First pass skips errors quickly to grab as much good data as possible.
  - Later passes retry damaged areas.
- **Safe Recovery**: Never writes to source; only reads.

---

## 🔍 How It Works

1. Initial run copies all readable data, skipping over bad sectors.
2. Logfile stores map of recovered vs. unrecovered data.
3. Further passes focus on smaller blocks around unreadable sectors.
4. Once cloning is complete, other tools (like `fsck`, `TestDisk`, or `PhotoRec`) can work on the image safely.

---

## 📊 Comparison Chart

| Tool      | Use Case                 | Strengths                       | Weaknesses |
|-----------|--------------------------|---------------------------------|-------------|
| dd        | Raw device cloning       | Simple, universal               | Stops on error, unsafe for failing disks |
| ddrescue  | Disk recovery (failing)  | Skips errors, resumes, log file | CLI only, learning curve |
| Clonezilla| Backup/restore (healthy) | Friendly interface, partitions  | Not optimized for failing drives |
| TestDisk  | Partition recovery       | Rebuilds lost tables            | Not a cloning tool |
| PhotoRec  | File carving             | Works when FS lost              | No structure, filenames lost |

---

## 📂 Common Scenarios

- **Failing hard drive with bad sectors**: Safely clone to an image for later analysis.
- **Unstable USB drive**: Preserve data before it degrades further.
- **Disk-to-disk migration under error conditions**: Recover failing system drives before replacement.

---

## ✅ Strengths

- Reliable recovery for failing hardware
- Logfile allows stopping and resuming
- Efficient error handling strategy
- Works across all file systems (since it copies at block level)

---

## ❌ Weaknesses

- Command-line only (steep learning curve)
- Requires another disk of equal or larger size
- Does not recover files directly (requires separate tools after cloning)

---

## 🔧 Example Usage

- Create disk image with logfile:  
  `sudo ddrescue -f -n /dev/sdX /path/to/disk.img /path/to/mapfile.log`
- Retry bad sectors with more effort:  
  `sudo ddrescue -d -r3 /dev/sdX /path/to/disk.img /path/to/mapfile.log`
- View progress:  
  `sudo ddrescueview /path/to/mapfile.log`

---

## 📚 Related Concepts/Notes

- [[Disk Data Recovery]] (General strategies)
- [[File Systems]] (Logical structure on disks)
- [[TestDisk]] (Partition repair)
- [[PhotoRec]] (File carving from images)
- [[ext4]] (Linux file system)
- [[NTFS]] (Windows file system)

---

## 🌐 External Resources

- GNU ddrescue homepage: https://www.gnu.org/software/ddrescue/
- ddrescueview (GUI mapfile viewer): https://sourceforge.net/projects/ddrescueview/
- Documentation: `man ddrescue`

---

## 📝 Summary

`ddrescue` is the go-to tool for cloning failing disks safely. By separating recovery (at block level) from repair (at file system level), it provides a solid foundation for further work with TestDisk, PhotoRec, or standard file system tools. Its use of a logfile ensures efficient, repeatable recovery even across multiple sessions.

---
