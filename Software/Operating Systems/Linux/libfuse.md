# libfuse
libfuse is a userspace library for implementing filesystems in user space on Linux and other Unix-like systems. It allows developers to write a fully functional filesystem without modifying kernel code, enabling experimentation, custom virtual filesystems, and integration with networked or remote storage.

---

## 🔍 Overview
FUSE (Filesystem in Userspace) is a kernel module plus a userspace library that allows non-privileged users to create their own filesystem without editing kernel code. `libfuse` provides the userspace API and helper functions, while the kernel module handles communication between the userspace program and the virtual filesystem interface. Popular use cases include SSHFS, encfs, and cloud storage mounts.

---

## ⚙️ Core Concepts
- User-space filesystem development
- Kernel-user communication via FUSE device
- Filesystem operations: read, write, getattr, readdir, etc.
- Mounting virtual filesystems
- Integration with standard system calls
- Event-driven model for filesystem requests

---

## 🧩 How It Works
1. The kernel module registers `/dev/fuse` and provides a virtual filesystem interface.
2. The userspace process linked against `libfuse` opens `/dev/fuse`.
3. Kernel forwards filesystem requests (open, read, write) to the userspace process.
4. Userspace responds with appropriate data or error codes.
5. The kernel returns the response to the calling process.

`libfuse` abstracts much of the low-level IPC, making it easier to implement custom filesystem logic.

---

## 🛠️ Installation
On Linux, libfuse can be installed via standard package managers:

- Debian/Ubuntu:
`sudo apt install libfuse2 libfuse-dev`  
`sudo apt install libfuse2-t64` (if you need 64-bit compatibility library)

- Fedora/RedHat:
`sudo dnf install fuse-libs fuse-devel`

- Arch Linux:
`sudo pacman -S fuse2`

- From source:
1. Clone: `git clone https://github.com/libfuse/libfuse.git`
2. Build: `cmake -S . -B build && cmake --build build`
3. Install: `sudo cmake --install build`

---

## 📊 Comparison Chart

| Library / Variant | Architecture | 64-bit Support | Kernel Dependency | Primary Use Case | Typical Users |
|------------------|--------------|----------------|-----------------|-----------------|---------------|
| libfuse2 | 32/64-bit | Optional | Yes | User-space filesystem development | Developers on Linux |
| libfuse2-t64 | 64-bit | Yes | Yes | Compatibility for 64-bit applications | Developers on modern 64-bit systems |
| libfuse3 | 32/64-bit | Yes | Yes | Modern FUSE API, async operations | New projects, cloud storage |
| WinFsp | Windows | Yes | Yes | FUSE-like support on Windows | Cross-platform projects needing FUSE API |
| MacFUSE / OSXFUSE | macOS | Yes | Yes | FUSE for macOS | macOS filesystem utilities |

---

## 🎯 Use Cases
- SSHFS: mount remote servers as local directories
- EncFS: encrypted virtual filesystems
- Cloud storage integration: Google Drive, Dropbox, S3
- Custom virtual filesystems for applications or games
- Container filesystems in Kubernetes or Docker

---

## AppImages

AppImage needs libfuse because it relies on FUSE (Filesystem in Userspace) to mount itself as a virtual filesystem. Libfuse is a user-space library that allows the AppImage to appear and behave like a mounted filesystem without requiring root privileges, enabling the app to run directly without installation. This filesystem mounting by FUSE is essential for executing the packaged software inside the AppImage format.

In practice, when you run an AppImage, FUSE mounts the image so the system can access its files and run the contained application transparently. Without libfuse installed, the AppImage cannot mount properly and will fail to run unless it is extracted manually. Most modern Linux distributions come with FUSE enabled by default, but some, such as newer Ubuntu versions, require explicit installation of libfuse2 to use AppImage files.

Thus, libfuse provides the crucial interface that bridges the AppImage container and the Linux kernel, allowing apps packaged in AppImage format to run easily without installation while maintaining system security and user-space operation. If libfuse is missing, users can still extract an AppImage manually and run the app from the extracted contents, but this is less convenient.

---

## ✅ Strengths
- Enables rapid filesystem development without kernel programming
- Works for non-privileged users
- Supports networked and encrypted filesystems
- Cross-platform variants exist
- Mature ecosystem with many production users

---

## ❌ Weaknesses
- Slightly lower performance compared to kernel-native filesystems
- Dependent on kernel module; requires installation
- Debugging complex virtual filesystem logic can be tricky
- Version differences (libfuse2 vs libfuse3) may affect API compatibility

---

## 🧠 Capabilities
- Filesystem read/write and metadata operations
- Directory listing and inode handling
- File attribute manipulation
- Event-driven callbacks for filesystem actions
- Network filesystem implementation
- Encrypted or virtual overlays

---

## 🧰 Compatible Items
- Linux kernel with FUSE module
- SSHFS, EncFS, CloudFS tools
- Filesystem utilities like `fusermount`
- Development environments using GCC/Clang
- Docker/Containerized environments supporting FUSE

---

## 🔗 Related Concepts/Notes
- [[FUSE]] (Filesystem in Userspace)
- [[SSHFS]] (Remote filesystem mounting)
- [[EncFS]] (Encrypted filesystem)
- [[libfuse3]] (Modern FUSE library)
- [[Filesystem Drivers]]
- [[Virtual Filesystems]]
- [[Container Filesystems]]

---

## 🌐 External Resources
- libfuse official repository: https://github.com/libfuse/libfuse
- FUSE wiki: https://github.com/libfuse/libfuse/wiki
- SSHFS documentation
- EncFS documentation
- Linux Kernel FUSE module documentation

---

## 🧾 Documentation and Support
- Man pages: `man fuse`, `man fusermount`
- Community forums: Linux kernel mailing list, StackOverflow
- GitHub issues and discussions for libfuse
- Tutorials for virtual filesystem development in C or Zig

---

## 🏁 Summary
libfuse provides a robust mechanism to implement user-space filesystems, enabling both experimentation and production-quality virtual filesystems. With variants like libfuse2 and libfuse2-t64, it supports both legacy and modern 64-bit applications, making it widely used in networking, cloud storage, and containerized environments.
