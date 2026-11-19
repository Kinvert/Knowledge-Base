# Linux

**Linux** is a family of open-source, Unix-like operating systems based on the Linux kernel. It is widely used in servers, embedded systems, robotics, cloud computing, and personal devices. Linux distributions (distros) combine the kernel with system libraries, package managers, and desktop environments to provide a complete operating system.

---

## ⚙️ Overview

- **Kernel-based**: Built around the Linux kernel, originally released by Linus Torvalds in 1991  
- **Distributions**: Examples include Ubuntu, Debian, Fedora, Arch, and CentOS  
- **Open Source**: Licensed under the GNU General Public License (GPL)  
- **Ubiquity**: Powers most servers, supercomputers, IoT devices, and is core to Android  

---

## 🧠 Core Concepts

- **Kernel**: The core that manages processes, memory, drivers, and hardware  
- **Shell**: Command-line interface for interacting with the system (e.g., `bash`, `zsh`)  
- **Package Managers**: Tools to install/manage software (e.g., `apt`, `dnf`, `pacman`)  
- **File System Hierarchy**: `/etc`, `/home`, `/var`, `/usr`, `/bin`, etc.  
- **Permissions**: User/group/other with read/write/execute flags  

---

## 📊 Comparison Chart (Linux vs. Other OS)

| Feature             | Linux                  | Windows               | macOS                   |
|---------------------|-----------------------|-----------------------|--------------------------|
| License             | Open source (GPL)     | Proprietary           | Proprietary (BSD-based) |
| Customization       | Very high             | Limited               | Moderate                |
| Security            | Strong (multi-user)   | Often target for malware | Unix permissions, strong |
| Popular Use Cases   | Servers, robotics, embedded | Desktop, gaming  | Creative, development   |
| Package Management  | `apt`, `dnf`, `pacman`| EXE/MSI installers    | `brew`, App Store       |

---

## 🛠️ Common Linux Commands

- `ls` → List files in a directory  
- `cd` → Change directory  
- `pwd` → Print current working directory  
- `cp source dest` → Copy files or directories  
- `mv source dest` → Move/rename files  
- `rm file` → Remove files  
- `chmod 755 file` → Change permissions  
- `chown user:group file` → Change file ownership  
- `ps aux` → Show running processes  
- `top` → Interactive process viewer  
- `df -h` → Show disk usage  
- `du -sh folder` → Show folder size  
- `systemctl start service` → Start a systemd service  
- `journalctl -xe` → View system logs  

---

## 🔧 Configuration and Important Files

- `/etc/passwd` → User account information  
- `/etc/fstab` → Disk mounting configuration  
- `/etc/hosts` → Local hostname resolution  
- `/etc/network/interfaces` or `/etc/netplan/` → Network configuration  
- `/var/log/` → System logs  
- `/home/username/` → User files and settings  

---

## ✅ Strengths

- Stability and reliability (favored for servers)  
- Highly customizable (kernel, shell, desktop environments)  
- Strong community and ecosystem  
- Runs on nearly any hardware (from embedded boards to supercomputers)  
- Preferred for robotics, AI, and cloud computing  

---

## ❌ Weaknesses

- Learning curve for beginners  
- Hardware driver compatibility issues in some cases  
- Fragmentation across distributions  
- Gaming support less extensive than Windows (though improving with Proton/Steam Deck)  

---

## 🔗 Related Concepts/Notes

- [[Operating Systems]]
- [[GRUB]] (Bootloader)
- [[UEFI]] (Firmware interface)
- [[Shell]]
- [[Pip]]
- [[Conda]]
- [[SSH]]
- [[ROS2]] (Robot Operating System)
- [[Docker]]
- [[MMU]]
- [[Linux Requirements]]

---

## 📚 Further Reading

- [Linux Kernel Official Site](https://www.kernel.org/)  
- [GNU Project](https://www.gnu.org/)  
- [Arch Wiki](https://wiki.archlinux.org/)  
- [Linux Foundation](https://www.linuxfoundation.org/)  
- [Ubuntu Documentation](https://help.ubuntu.com/)  
