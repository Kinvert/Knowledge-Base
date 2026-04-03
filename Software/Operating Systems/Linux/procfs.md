# procfs

**procfs** (`/proc`) is a Linux virtual filesystem that exposes kernel and process information as regular files. Nothing in `/proc` exists on disk — the kernel generates it dynamically on read. Reading a file queries the kernel; writing to certain files (under `/proc/sys/`) changes kernel parameters at runtime. This is the mechanism behind [[sysctl]] and tools like `top`, `ps`, and `htop`.

---

## ⚙️ Overview

- Mounted at `/proc` on every Linux system by default
- Pseudo-filesystem — no disk I/O, no storage consumed
- Process info lives in numbered directories: `/proc/1/`, `/proc/4523/`, etc.
- Kernel tunables live under `/proc/sys/` — the writable portion
- Everything follows the Unix "everything is a file" philosophy
- Existed since Linux 1.0 (1994), inspired by Plan 9

```bash
# Read CPU info
cat /proc/cpuinfo

# Check memory
cat /proc/meminfo

# Read a kernel tunable directly
cat /proc/sys/fs/inotify/max_user_watches
```

---

## 🧠 Core Concepts

- **Process Directories** (`/proc/[pid]/`)
  Each running process gets a directory named by its PID. Contains status, memory maps, file descriptors, environment, and more.

- **Kernel Tunables** (`/proc/sys/`)
  Writable files that control kernel behavior. [[sysctl]] is a friendly interface for reading/writing these.

- **Hardware Info**
  `/proc/cpuinfo`, `/proc/meminfo`, `/proc/interrupts`, `/proc/devices` — read-only snapshots of hardware state.

- **Dynamic Generation**
  Files don't exist on disk. The kernel generates content on each `read()` call, so values are always current.

- **Self-Referencing** (`/proc/self/`)
  Symlink to the current process's own `/proc/[pid]/` directory. Useful in scripts.

---

## 🗂️ Key Paths

### Process Info (`/proc/[pid]/`)

| Path | Description |
|------|-------------|
| `/proc/[pid]/status` | Process name, state, memory, UID/GID |
| `/proc/[pid]/cmdline` | Full command line (null-separated) |
| `/proc/[pid]/environ` | Environment variables (null-separated) |
| `/proc/[pid]/maps` | Memory-mapped regions (libraries, heap, stack) |
| `/proc/[pid]/fd/` | Directory of open file descriptors (symlinks to actual files) |
| `/proc/[pid]/stat` | Raw process stats (used by `ps`, `top`) |
| `/proc/[pid]/cgroup` | Cgroup membership (relevant for [[Docker]] containers) |
| `/proc/[pid]/exe` | Symlink to the running executable binary |

### System-Wide Info

| Path | Description |
|------|-------------|
| `/proc/cpuinfo` | CPU model, cores, flags, cache sizes |
| `/proc/meminfo` | Total/free/available RAM, swap, buffers |
| `/proc/loadavg` | 1/5/15-minute load averages |
| `/proc/uptime` | System uptime in seconds |
| `/proc/interrupts` | IRQ counts per CPU |
| `/proc/mounts` | Currently mounted filesystems |
| `/proc/version` | Kernel version string |
| `/proc/partitions` | Block device partition table |

### Kernel Tunables (`/proc/sys/`)

| Path | Description | Example |
|------|-------------|---------|
| `/proc/sys/fs/inotify/max_user_watches` | Max [[inotify]] watches per user | `524288` |
| `/proc/sys/fs/file-max` | System-wide file descriptor limit | `9223372036854775807` |
| `/proc/sys/net/ipv4/ip_forward` | Enable IP forwarding (routing) | `0` or `1` |
| `/proc/sys/net/core/somaxconn` | Max socket listen backlog | `4096` |
| `/proc/sys/vm/swappiness` | How aggressively to use swap | `60` |
| `/proc/sys/kernel/pid_max` | Maximum PID value | `4194304` |

---

## 🔬 `/proc/sys/` and sysctl

[[sysctl]] is just a convenience wrapper around `/proc/sys/`. The dot-notation maps directly to directory paths:

```
sysctl parameter:      fs.inotify.max_user_watches
procfs path:           /proc/sys/fs/inotify/max_user_watches
                       ──────────┬──────────────────────────
                        dots become slashes
```

These two commands are equivalent:

```bash
# Using sysctl
sudo sysctl fs.inotify.max_user_watches=524288

# Writing directly to procfs
echo 524288 | sudo tee /proc/sys/fs/inotify/max_user_watches
```

The direct write approach is lower-level but works without `sysctl` installed. Both are temporary — lost on reboot unless persisted to `/etc/sysctl.conf`.

---

## 📊 Comparison Chart

| Filesystem | Mount Point | Purpose | Writable | Persistent |
|------------|-------------|---------|----------|------------|
| **procfs** | `/proc` | Process info + kernel tunables | Partially (`/proc/sys/`) | No (virtual) |
| **sysfs** | `/sys` | Device/driver model, hardware topology | Some files | No (virtual) |
| **devfs / udev** | `/dev` | Device nodes for hardware access | N/A (device files) | No (virtual) |
| **debugfs** | `/sys/kernel/debug` | Kernel debugging info (ftrace, etc.) | Some files | No (virtual) |
| **configfs** | `/sys/kernel/config` | User-space driven kernel object config | Yes | No (virtual) |
| **cgroupfs** | `/sys/fs/cgroup` | Resource limits for process groups | Yes | No (virtual) |
| **tmpfs** | `/tmp`, `/run` | RAM-backed temporary files | Yes | No (RAM) |

---

## 🔧 Practical Examples

```bash
# List all open file descriptors for a process
ls -la /proc/$(pgrep obsidian | head -1)/fd/

# Check inotify watch limits (the driving example)
cat /proc/sys/fs/inotify/max_user_watches

# See which inotify instances a process has
ls /proc/$(pgrep code | head -1)/fdinfo/ | head

# Count how many processes are running
ls -d /proc/[0-9]* | wc -l

# Read your own process info
cat /proc/self/status

# Check if IP forwarding is enabled
cat /proc/sys/net/ipv4/ip_forward

# Find total and available memory
grep -E 'MemTotal|MemAvailable' /proc/meminfo
```

---

## 🌟 Strengths

- Universal inspection tool — works on every Linux system without extra packages
- Zero overhead — no daemon, no disk I/O, just kernel queries
- Real-time — always reflects current kernel state
- Scriptable — standard file operations (`cat`, `echo`, `grep`) work on it
- Foundation for many tools (`ps`, `top`, `htop`, `sysctl`, `lsof`)

---

## ⚠️ Weaknesses

- Linux-only — not available on macOS, Windows, or BSD (they have equivalents)
- Unstable ABI — file formats can change between kernel versions
- No standardized machine-readable format — some files are space-separated, some are colon-separated, some are custom
- Security exposure — process info visible to other users (mitigated by `hidepid` mount option)
- `/proc/sys/` writes are temporary — need [[sysctl]] config files for persistence

---

## 🔗 Related Notes

- [[Linux]]
- [[sysctl]]
- [[inotify]]
- [[epoll]]
- [[shmget]]
- [[Docker]]
- [[Bash]]
- [[udevadm]]

---

## 🌐 External Resources

- Linux man page: `https://man7.org/linux/man-pages/man5/proc.5.html`
- Kernel docs on procfs: `https://docs.kernel.org/filesystems/proc.html`
- Red Hat guide to `/proc`: `https://docs.redhat.com/en/documentation/red_hat_enterprise_linux/`

---

## 📝 Summary

procfs is the virtual filesystem at `/proc` that exposes kernel internals as regular files. It's the mechanism behind [[sysctl]] — when you run `sysctl fs.inotify.max_user_watches=524288`, you're really writing to `/proc/sys/fs/inotify/max_user_watches`. Every process monitoring tool on Linux reads from procfs, and it's the first place to look when debugging system behavior.
