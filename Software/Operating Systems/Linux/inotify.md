# inotify

**inotify** is a Linux kernel subsystem for monitoring filesystem events — file creation, modification, deletion, moves, and more. It's how applications like [[Obsidian]], VS Code, webpack, and `tail -f` detect file changes without polling. Each monitored file or directory consumes a "watch," and the system-wide watch limit is a common pain point for developers with large projects.

---

## ⚙️ Overview

- Kernel-level file monitoring — no polling, no CPU waste
- Event-driven: your application blocks until something happens
- Each watched file/directory = one "watch" consuming ~1 KB of kernel memory
- System-wide limit on watches per user (default: 8192 — often too low)
- API: `inotify_init()`, `inotify_add_watch()`, `read()` for events
- Available since Linux 2.6.13 (2005), replaced the older `dnotify`

```bash
# Check current watch limit
cat /proc/sys/fs/inotify/max_user_watches

# Monitor a directory for changes (requires inotify-tools)
inotifywait -m -r ~/projects/
```

---

## 🧠 Core Concepts

- **Watches**
  A watch monitors a single file or directory for specified events. Each watch is identified by a "watch descriptor" (integer). Directories watch their immediate children — not recursive by default.

- **Instances**
  Created via `inotify_init()`. Each instance is a file descriptor that receives events for all its watches. A process can have multiple instances.

- **Events**
  Delivered as structs when you `read()` from the inotify file descriptor.

| Event | Triggered When |
|-------|---------------|
| `IN_CREATE` | File/dir created in watched directory |
| `IN_DELETE` | File/dir deleted from watched directory |
| `IN_MODIFY` | File contents modified |
| `IN_MOVED_FROM` | File moved out of watched directory |
| `IN_MOVED_TO` | File moved into watched directory |
| `IN_ATTRIB` | Metadata changed (permissions, timestamps) |
| `IN_CLOSE_WRITE` | File opened for writing was closed |
| `IN_CLOSE_NOWRITE` | File opened read-only was closed |
| `IN_OPEN` | File was opened |
| `IN_ACCESS` | File was read |
| `IN_DELETE_SELF` | Watched file/dir itself was deleted |
| `IN_MOVE_SELF` | Watched file/dir itself was moved |

- **Recursive Watching**
  inotify does NOT natively support recursive directory monitoring. User-space tools like `inotifywait -r` simulate it by adding a watch on every subdirectory — which is exactly why watch limits matter.

---

## 🔬 The Watch Limit Problem

This is the most common inotify issue developers encounter. The error message:

```
Error: ENOSPC: System limit for number of file watchers reached
```

Or in some tools: `"No space left on device"` (misleading — it's watches, not disk).

### Why it happens

The default `max_user_watches` is **8192**. A single project can easily exceed this:

| What | Typical Watch Count |
|------|-------------------|
| Moderate Node.js project (`node_modules/`) | 10,000–50,000 |
| Large Obsidian vault (800+ notes) | 1,000–5,000 |
| VS Code workspace (multi-folder) | 5,000–20,000 |
| Webpack/Vite watching `src/` + deps | 5,000–30,000 |
| Multiple projects open simultaneously | Adds up fast |

### Memory cost

Each watch consumes approximately **1,080 bytes** on 64-bit systems (a `struct inotify_inode_mark` plus overhead). So:

| Watches | Memory |
|---------|--------|
| 8,192 (default) | ~8 MB |
| 65,536 | ~67 MB |
| 524,288 | ~540 MB |
| 1,048,576 | ~1.1 GB |

524,288 is the most common recommendation — meaningful memory cost (~540 MB worst case) but manageable on modern systems with 16+ GB RAM.

### The fix

```bash
# Temporary (lost on reboot)
sudo sysctl fs.inotify.max_user_watches=524288

# Permanent
echo fs.inotify.max_user_watches=524288 | sudo tee -a /etc/sysctl.conf
sudo sysctl -p
```

See [[sysctl]] for a full breakdown of these commands.

---

## 🔧 API Overview

The C API is three system calls plus standard `read()`:

```c
#include <sys/inotify.h>

// 1. Create an inotify instance (returns a file descriptor)
int fd = inotify_init1(IN_NONBLOCK);

// 2. Add a watch on a path for specific events
int wd = inotify_add_watch(fd, "/tmp/mydir",
                           IN_CREATE | IN_DELETE | IN_MODIFY);

// 3. Read events (blocks unless IN_NONBLOCK was set)
char buf[4096];
ssize_t len = read(fd, buf, sizeof(buf));

// 4. Parse events from the buffer
struct inotify_event *event = (struct inotify_event *)buf;
printf("Event on: %s (mask: 0x%x)\n", event->name, event->mask);

// 5. Remove a watch
inotify_rm_watch(fd, wd);
close(fd);
```

Most languages wrap this: Python has `inotify` and `watchdog`, Node.js has `fs.watch()` (backed by inotify on Linux), Rust has `notify`.

---

## 💡 User-Space Tools

The `inotify-tools` package provides two CLI utilities:

```bash
# Install
sudo apt install inotify-tools

# Watch a directory recursively, print events as they happen
inotifywait -m -r -e modify,create,delete ~/projects/

# Count events over a period (statistical summary)
inotifywatch -r -t 60 ~/projects/

# Trigger a build on file change
inotifywait -m -r -e modify src/ | while read path action file; do
    echo "Rebuilding due to $action on $file..."
    make
done
```

---

## 📊 Comparison Chart

| Tool / API | Platform | Level | Recursive | Strengths | Weaknesses |
|------------|----------|-------|-----------|-----------|------------|
| **inotify** | Linux | Kernel | No (manual) | Efficient, fine-grained events | Linux-only, watch limits |
| **fanotify** | Linux | Kernel | Yes (mount-wide) | Filesystem-wide, access control | Less granular, needs `CAP_SYS_ADMIN` |
| **[[epoll]]** | Linux | Kernel | N/A | Scalable I/O multiplexing | Monitors FDs, not filesystem events |
| **kqueue** | BSD/macOS | Kernel | No | General event notification | Not on Linux, per-file only |
| **FSEvents** | macOS | Framework | Yes | Recursive, directory-level | macOS-only, coarse granularity |
| **ReadDirectoryChangesW** | Windows | Win32 API | Optional | Native Windows support | Windows-only, complex API |
| **watchman** (Meta) | Cross-platform | User-space | Yes | Scalable, query language | External daemon required |
| **watchdog** (Python) | Cross-platform | User-space | Yes | Python-native, easy API | Wrapper overhead |

---

## 🌟 Strengths

- Efficient kernel-level event delivery — no polling overhead
- Fine-grained: per-file event types (create, modify, delete, move, etc.)
- Low latency — events delivered nearly instantly
- Well-supported in every major language and framework
- Battle-tested — powers file watching in VS Code, webpack, systemd, and more
- Works with [[epoll]] for multiplexed event handling

---

## ⚠️ Weaknesses

- Linux-only — need kqueue (BSD/macOS), FSEvents (macOS), or ReadDirectoryChangesW (Windows) elsewhere
- No native recursive watching — must manually add watches on all subdirectories
- Watch limits can be hit easily with large projects (the `ENOSPC` error)
- Each watch consumes ~1 KB kernel memory — can add up
- Cannot watch files on network filesystems (NFS, CIFS, FUSE)
- Rename detection requires correlating `IN_MOVED_FROM` + `IN_MOVED_TO` via cookie

---

## 🔗 Related Notes

- [[epoll]]
- [[sysctl]]
- [[procfs]]
- [[Linux]]
- [[Docker]]
- [[Bash]]

---

## 🌐 External Resources

- Linux man page: `https://man7.org/linux/man-pages/man7/inotify.7.html`
- inotify-tools: `https://github.com/inotify-tools/inotify-tools`
- LWN article on inotify: `https://lwn.net/Articles/604686/`
- Python watchdog: `https://github.com/gorakhargosh/watchdog`

---

## 📝 Summary

inotify is how Linux applications detect file changes without polling. The most common issue is the watch limit — with a default of 8,192 watches and modern projects easily consuming tens of thousands, the `ENOSPC` error is almost a rite of passage. The fix is a one-liner with [[sysctl]]: bump `fs.inotify.max_user_watches` to 524,288 and persist it in `/etc/sysctl.conf`.
