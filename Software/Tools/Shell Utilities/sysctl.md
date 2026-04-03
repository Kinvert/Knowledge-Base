# sysctl

**sysctl** is a Linux utility for viewing and modifying kernel parameters at runtime. It reads from and writes to [[procfs]] (`/proc/sys/`), providing a friendly interface for tuning filesystem limits, networking behavior, memory management, and more. Changes are temporary by default — persisting them requires writing to `/etc/sysctl.conf` or a drop-in file in `/etc/sysctl.d/`.

---

## ⚙️ Overview

```bash
sysctl [options] [variable[=value] ...]
```

- Dot-notation maps to [[procfs]] paths: `fs.inotify.max_user_watches` → `/proc/sys/fs/inotify/max_user_watches`
- Read with `sysctl variable`, write with `sysctl variable=value`
- Writing requires `sudo` (modifying kernel parameters)
- Temporary by default — reboot resets everything
- Permanent via config files loaded at boot by `systemd-sysctl.service`

---

## 🔬 Command Breakdown — The Driving Example

These three commands fix the [[inotify]] watch limit for tools like VS Code, Obsidian, and webpack:

### Command 1: Set the value temporarily

```bash
sudo sysctl fs.inotify.max_user_watches=524288
```

**`sudo`** — Required because you're modifying a kernel parameter.

**`sysctl`** — The tool itself.

**`fs.inotify.max_user_watches`** — The kernel parameter. Dots map to path separators:
```
fs.inotify.max_user_watches
 ↓    ↓          ↓
/proc/sys/fs/inotify/max_user_watches
```

**`=524288`** — The new value. This is 512 × 1024 — a common recommendation. Each [[inotify]] watch consumes ~1 KB of kernel memory, so 524,288 watches ≈ 540 MB worst case. See [[inotify]] for why the default (8,192) is too low.

**Result:** Immediate effect, but lost on reboot.

### Command 2: Persist to config file

```bash
echo fs.inotify.max_user_watches=524288 | sudo tee -a /etc/sysctl.conf
```

Why not `sudo echo ... > /etc/sysctl.conf`? Because the shell redirect (`>`) runs as *your* user, not as root. `sudo` only elevates the `echo` command — the redirect still fails with "Permission denied." The [[tee]] command solves this: it runs as root (via `sudo`) and writes to the file.

**`echo ...`** — Produces the config line on stdout.

**`|`** — [[pipe]] sends stdout to the next command's stdin.

**`sudo tee -a`** — [[tee]] reads stdin and writes it to the file. `-a` means append (without it, tee overwrites). `sudo` gives tee root permission to write to `/etc/sysctl.conf`.

**`/etc/sysctl.conf`** — The main sysctl config file, read at boot. Alternatively, use a drop-in file like `/etc/sysctl.d/99-inotify.conf` for cleaner organization.

### Command 3: Reload config

```bash
sudo sysctl -p
```

**`-p`** — Reload `/etc/sysctl.conf` (or specify a path: `sysctl -p /etc/sysctl.d/99-inotify.conf`). Applies all settings from the file immediately without rebooting.

After this, `max_user_watches` is set to 524,288 both now and on every future boot.

---

## 🚩 Common Options

| Option | Description | Example |
|--------|-------------|---------|
| *(none)* | Read a parameter | `sysctl fs.inotify.max_user_watches` |
| `=value` | Set a parameter | `sysctl fs.inotify.max_user_watches=524288` |
| `-w` | Explicit write (same as `=`) | `sysctl -w net.ipv4.ip_forward=1` |
| `-p [file]` | Load settings from file | `sysctl -p /etc/sysctl.d/99-custom.conf` |
| `-a` | List all parameters | `sysctl -a` |
| `-n` | Print value only (no name) | `sysctl -n fs.file-max` → `9223372036854775807` |
| `-q` | Quiet — suppress output on set | `sysctl -q -w vm.swappiness=10` |
| `--pattern` | Filter by regex | `sysctl -a --pattern inotify` |
| `--system` | Load all config files in order | `sysctl --system` |

---

## 🗂️ Common Parameters

### Filesystem

| Parameter | Default | Description |
|-----------|---------|-------------|
| `fs.inotify.max_user_watches` | 8192 | Max [[inotify]] watches per user |
| `fs.inotify.max_user_instances` | 128 | Max inotify instances per user |
| `fs.file-max` | ~huge | System-wide max open file descriptors |
| `fs.nr_open` | 1048576 | Per-process max open file descriptors |

### Networking

| Parameter | Default | Description |
|-----------|---------|-------------|
| `net.ipv4.ip_forward` | 0 | Enable IP forwarding (routing between interfaces) |
| `net.core.somaxconn` | 4096 | Max socket listen queue backlog |
| `net.ipv4.tcp_syncookies` | 1 | SYN flood protection |
| `net.ipv4.tcp_max_syn_backlog` | 512 | Max queued SYN requests |
| `net.ipv4.conf.all.rp_filter` | 1 | Reverse path filtering (anti-spoofing) |

### Memory

| Parameter | Default | Description |
|-----------|---------|-------------|
| `vm.swappiness` | 60 | How aggressively to swap (0=avoid, 100=aggressive) |
| `vm.overcommit_memory` | 0 | Memory overcommit policy (0=heuristic, 1=always, 2=never) |
| `vm.dirty_ratio` | 20 | % of RAM for dirty pages before sync |
| `vm.dirty_background_ratio` | 10 | % of RAM for dirty pages before background flush |

### Kernel

| Parameter | Default | Description |
|-----------|---------|-------------|
| `kernel.panic` | 0 | Seconds before reboot on panic (0=hang) |
| `kernel.shmmax` | varies | Max shared memory segment size |
| `kernel.pid_max` | 4194304 | Maximum PID value |
| `kernel.core_pattern` | varies | Core dump filename pattern |

---

## 🔧 Practical Examples

```bash
# Read a value
sysctl fs.inotify.max_user_watches

# Read value only (no parameter name)
sysctl -n vm.swappiness

# Set temporarily
sudo sysctl vm.swappiness=10

# Search all parameters by pattern
sysctl -a --pattern inotify
sysctl -a --pattern forward

# Load from specific config file
sudo sysctl -p /etc/sysctl.d/99-inotify.conf

# Load all system configs in proper order
sudo sysctl --system

# Create a clean drop-in file (better than editing sysctl.conf)
echo "fs.inotify.max_user_watches=524288" | sudo tee /etc/sysctl.d/99-inotify.conf
sudo sysctl --system
```

---

## ⏱️ Temporary vs Permanent

| Method | Survives Reboot | Scope | Command |
|--------|----------------|-------|---------|
| `sysctl variable=value` | No | Immediate | `sudo sysctl fs.inotify.max_user_watches=524288` |
| Write directly to [[procfs]] | No | Immediate | `echo 524288 \| sudo tee /proc/sys/fs/inotify/max_user_watches` |
| `/etc/sysctl.conf` | Yes | All params in file | `echo ... \| sudo tee -a /etc/sysctl.conf && sudo sysctl -p` |
| `/etc/sysctl.d/*.conf` | Yes | Per-file (recommended) | Drop-in files loaded alphabetically |

Drop-in files in `/etc/sysctl.d/` are preferred over editing `/etc/sysctl.conf` directly. They're easier to manage, won't conflict with package updates, and can be removed cleanly.

---

## 📊 Comparison Chart

| Tool | Scope | What It Tunes | Persistent | Requires Root |
|------|-------|---------------|------------|---------------|
| **sysctl** | Kernel parameters (`/proc/sys/`) | FS limits, networking, memory, kernel | Config file | Yes |
| **ulimit** | Per-process resource limits | Open files, stack size, CPU time | `/etc/security/limits.conf` | No (soft), Yes (hard) |
| **limits.conf** | Per-user/group resource limits | Same as ulimit, but persistent | Yes (PAM) | Yes |
| **modprobe** | Kernel module parameters | Driver-specific settings | `/etc/modprobe.d/` | Yes |
| **ethtool** | Network interface settings | Ring buffers, offload, speed | Network scripts | Yes |
| **tuned** | System-wide performance profiles | Bundles sysctl + other tuning | Yes (daemon) | Yes |
| **systemd-sysctl** | Loads sysctl configs at boot | Same as sysctl | Yes (service) | Yes |

---

## 🌟 Strengths

- Standard on every Linux system — no packages to install
- Immediate effect — no restart needed for temporary changes
- Covers all kernel tunables in one tool
- Simple syntax: `parameter=value`
- Drop-in config files for clean management
- Well-documented (`man sysctl`, `man sysctl.conf`)

---

## ⚠️ Weaknesses

- Temporary by default — easy to forget persistence step
- Requires root for all writes
- No validation — can set invalid values that break things
- Parameter names are not always intuitive (`vm.dirty_background_bytes` vs `vm.dirty_background_ratio`)
- Some parameters require specific kernel config or modules to be loaded
- Changes don't propagate to [[Docker]] containers (they have their own `/proc/sys` view unless using `--privileged` or `--sysctl`)

---

## 🔗 Related Notes

- [[Linux]]
- [[procfs]]
- [[inotify]]
- [[epoll]]
- [[tee]]
- [[pipe]]
- [[Docker]]
- [[Bash]]
- [[udevadm]]

---

## 🌐 External Resources

- Linux man page: `https://man7.org/linux/man-pages/man8/sysctl.8.html`
- sysctl.conf man page: `https://man7.org/linux/man-pages/man5/sysctl.conf.5.html`
- Kernel sysctl docs: `https://docs.kernel.org/admin-guide/sysctl/`
- Arch Wiki on sysctl: `https://wiki.archlinux.org/title/Sysctl`

---

## 📝 Summary

sysctl reads and writes kernel parameters via [[procfs]] (`/proc/sys/`). The driving example — `sudo sysctl fs.inotify.max_user_watches=524288`, persisted with `echo | sudo tee -a /etc/sysctl.conf`, then reloaded with `sysctl -p` — is a three-step pattern you'll use whenever a kernel tunable needs changing: set it now, write it to config, reload. The `sudo tee` trick exists because shell redirects don't inherit `sudo` privileges.
