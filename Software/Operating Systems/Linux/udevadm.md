# udevadm

`udevadm` is the command-line utility for controlling and interacting with the `udev` device manager on Linux systems. It allows developers and system administrators to monitor, query, and manage device events and rules in real time. In robotics and embedded systems, `udevadm` is especially useful for managing USB devices, serial ports, and other dynamically detected hardware.

---

## ⚙️ Overview

`udev` is the device manager for the Linux kernel, responsible for dynamically creating and removing device nodes in `/dev`. `udevadm` provides userspace control over this process, making it possible to test and reload rules, trigger device events, and debug hardware detection.

**What you can do with it (at a glance):**
- Monitor hotplug events (`add`, `remove`, `change`) as they happen.
- Inspect device attributes and environment variables to write precise rules.
- Reload rules without rebooting and re-trigger matches across the system.
- Create persistent symlinks (e.g., `/dev/robot-lidar`) for flaky device orderings.
- Adjust permissions, groups, and tags (`uaccess`, `systemd`) for user access.

---

## 🧠 Core Concepts

- **Device Nodes**: Special files in `/dev` representing hardware devices.
- **Rules**: Matching expressions + actions (e.g., `SYMLINK+=`, `MODE=`, `GROUP=`, `TAG+=`, `RUN+=`).
- **Events**: Hardware changes (`add`, `remove`, `change`) that trigger rule evaluation.
- **Subsystems**: Device categories (e.g., `usb`, `tty`, `block`, `net`, `input`).
- **Attributes**: Metadata exposed via `/sys` (sysfs), usable in rules (`ATTR{}` / `ATTRS{}`).
- **Environment**: Key-value pairs set during event processing (`ENV{ID_SERIAL}`, etc.).
- **Unit Interaction**: `TAG+="systemd"` can cause systemd units to start/stop with devices.

---

## 📊 Comparison Chart

| Tool / Feature       | `udevadm` | [[lsusb]] | [[lspci]] | [[dmesg]] | [[hwinfo]] | [[udev]] | [[systemd]] |
|----------------------|-----------|-----------|-----------|-----------|------------|----------|-------------|
| **Manages udev Rules** | ✅       | ❌         | ❌         | ❌         | ❌          | ✅       | ⚠️ via tags |
| **Real-time Monitoring** | ✅     | ❌         | ❌         | Limited    | Limited     | ✅       | ❌          |
| **Kernel Device Info**   | ✅     | ✅         | ✅         | ✅         | ✅          | ✅       | ✅          |
| **Trigger/Simulate Events** | ✅  | ❌         | ❌         | ❌         | ❌          | ✅       | ❌          |
| **User Access Control**  | ✅     | ❌         | ❌         | ❌         | ❌          | ✅       | ✅ (units)  |

---

## 🛠 Use Cases

- Stable names for robot peripherals (LiDARs, IMUs, cameras) regardless of USB order.
- Automatic permissions for `/dev/ttyUSB*` so non-root processes can access sensors.
- Auto-start services when specific devices appear (camera node, CAN interface).
- Debugging intermittent device enumeration and flaky cables/hubs.
- Auditing attributes for writing robust, portable rules.

---

## ✅ Strengths

- Deep visibility into Linux device management.
- Essential for robotics where device order and permissions matter.
- Transport-agnostic across subsystems (`tty`, `net`, `input`, `block`, etc.).
- Scriptable; integrates well with containers and CI-on-hardware.

---

## ❌ Weaknesses

- Requires root privileges for most operations.
- Complex matching semantics (`ATTR` vs `ATTRS`, parent walking) can confuse.
- Misconfigured rules can block devices or break permissions.
- Debugging `RUN+=` side-effects can be tricky (ordering, environment, timing).

---

## 🧩 How It Works (High-Level Flow)

1. Kernel detects device → emits uevent with attributes and subsystem info.  
2. `systemd-udevd` receives event, builds an environment, and evaluates rules.  
3. Matching rules set node permissions, create symlinks, add tags, queue `RUN+=` programs.  
4. Device node(s) appear in `/dev`; optional systemd units may be started.  
5. Subsequent `change`/`remove` events update or remove nodes and symlinks.

---

## 🧪 Debug Workflow (Step-by-Step)

- Identify device node: `ls -l /dev/ttyUSB*`
- Inspect full info: `udevadm info --query=all --name=/dev/ttyUSB0`
- Follow live events: `udevadm monitor --kernel --udev --property`
- Check attributes in sysfs: `udevadm info --attribute-walk --name=/dev/ttyUSB0`
- Edit rule file (e.g., `/etc/udev/rules.d/99-robot.rules`) and reload: `udevadm control --reload-rules`
- Re-apply rules to existing devices: `udevadm trigger --subsystem-match=tty`
- Validate a single device path: `udevadm test /sys/class/tty/ttyUSB0`

---

## 🧰 Rule Writing Guide (One-Line Examples)

- Minimal stable symlink by VID/PID:  
  `SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="lidar"`

- Set permissions and group for serial devices:  
  `SUBSYSTEM=="tty", MODE="0660", GROUP="dialout"`

- Match by USB serial number (most stable):  
  `SUBSYSTEM=="tty", ENV{ID_SERIAL_SHORT}=="A1B2C3D4", SYMLINK+="imu"`

- Differentiate identical devices by physical port:  
  `SUBSYSTEM=="tty", KERNELS=="1-1.2:1.0", SYMLINK+="left-lidar"`

- Add user seat access (desktop/non-root plug-and-play):  
  `SUBSYSTEM=="tty", TAG+="uaccess"`

- Trigger a script when a device appears:  
  `SUBSYSTEM=="video4linux", ACTION=="add", RUN+="/usr/local/bin/camera-up %E{DEVNAME}"`

- Tie a systemd unit to a device:  
  `SUBSYSTEM=="net", KERNEL=="can0", TAG+="systemd"`

- Disambiguate FTDI devices using interface number:  
  `SUBSYSTEM=="tty", ATTRS{bInterfaceNumber}=="00", SYMLINK+="gps"`

- Ignore noisy devices completely:  
  `SUBSYSTEM=="usb", ATTR{idVendor}=="1d6b", OPTIONS+="ignore_device"`

---

## 🔍 Attributes, Match Keys, and Semantics

- **`KERNEL` / `KERNELS`**: Match device name vs. walk through parents.  
  Example: `KERNEL=="ttyUSB*"` vs. `KERNELS=="1-1.4"` (USB tree path)

- **`SUBSYSTEM` / `DRIVER`**: Narrow by functional class/driver.  
  Example: `SUBSYSTEM=="video4linux", DRIVERS=="uvcvideo"`

- **`ATTR{}` vs `ATTRS{}`**: Current device attribute vs parent device attribute.  
  Example: `ATTR{bInterfaceNumber}=="00"` vs `ATTRS{idVendor}=="2341"`

- **`ENV{}`**: Match environment keys udev exports.  
  Example: `ENV{ID_MODEL}=="Arduino_Uno"`

- **Actions**: `MODE=`, `GROUP=`, `OWNER=`, `SYMLINK+=`, `TAG+=`, `RUN+=`, `OPTIONS+=`.

---

## 🧱 Persistent Device Naming Patterns (Robotics)

- Stable symlink by serial number:  
  `SUBSYSTEM=="tty", ENV{ID_SERIAL_SHORT}=="FT4S3R1AL", SYMLINK+="base-imu"`

- Name cameras via `ID_V4L_PRODUCT`:  
  `SUBSYSTEM=="video4linux", ENV{ID_V4L_PRODUCT}=="ZED", SYMLINK+="zed-left"`

- CAN interfaces by interface index:  
  `SUBSYSTEM=="net", NAME=="can*", ENV{ID_NET_DRIVER}=="mcp251x", NAME="can0"`

- USB-to-serial per-port naming:  
  `SUBSYSTEM=="tty", KERNELS=="1-1.3:1.0", SYMLINK+="arm-teensy"`

---

## 🧪 Testing & Monitoring One-Liners (Cheat Sheet)

- Show everything about a device: `udevadm info --query=all --name=/dev/ttyUSB0`
- Walk parent attributes: `udevadm info --attribute-walk --name=/dev/ttyUSB0`
- Live event stream (with env): `udevadm monitor --udev --kernel --property`
- Reload rules daemon: `udevadm control --reload-rules`
- Re-trigger for a class: `udevadm trigger --subsystem-match=tty`
- Test a path (no real action): `udevadm test /sys/class/tty/ttyUSB0`
- Filter to adds only: `udevadm monitor --udev --property | grep -E "^ACTION=add|^DEVNAME="`
- Confirm rule file load order: `udevadm test-builtin rules /sys/class/tty/ttyUSB0`
- Check running daemon status: `systemctl status systemd-udevd`
- Show current tags for a node: `udevadm info --query=property --name=/dev/video0 | grep TAGS`
- Find device by symlink: `udevadm info --query=path --name=/dev/robot-lidar`
- Map devnode → sysfs path: `udevadm info --path=/sys/class/tty/ttyUSB0`
- Find all serials quickly: `udevadm info -e | grep -E "DEVNAME=|ID_SERIAL_SHORT="`

---

## ⚠️ Common Pitfalls (and Fixes)

- **Rule not applied** → Forgot to reload: `udevadm control --reload-rules` then `udevadm trigger`.  
- **Wrong attribute scope** → Use `ATTRS{}` for parent USB attributes (VID/PID), `ATTR{}` for the device itself.  
- **Symlink not created** → Check `ACTION=="add"`, spelling of keys, and whether rule file has `.rules` extension.  
- **Permissions still wrong** → Conflicts with distro defaults; increase rule priority (`10-*.rules` vs `99-*.rules`).  
- **Timing race** with `RUN+=` → External services may start before node is ready; consider systemd device units.

---

## 🔒 Security & Access Notes

- Prefer group-based access: `GROUP="dialout", MODE="0660"`.  
- For desktop logins, `TAG+="uaccess"` grants device ACLs to active seat.  
- Avoid world-writable devices unless necessary; audit with `udevadm info --query=property`.

---

## 🔁 systemd Integration Tips

- Start a unit when a device appears by tagging: `TAG+="systemd"`.  
- Bind a unit to a device with a device unit name (e.g., `dev-ttyUSB0.device`).  
- Use `ENV{SYSTEMD_WANTS}="my@%E{DEVNAME}.service"` to queue services.

---

## 🔧 Common Commands

- Monitor device events: `udevadm monitor`  
- Query device attributes: `udevadm info --query=all --name=/dev/ttyUSB0`  
- Trigger udev events: `udevadm trigger`  
- Reload rules: `udevadm control --reload-rules`  
- Test rules: `udevadm test /sys/class/tty/ttyUSB0`

---

## 🤖 Robotics Patterns (Quick Recipes)

- Give LiDAR a stable name and access:  
  `SUBSYSTEM=="tty", ENV{ID_SERIAL_SHORT}=="LDR123", SYMLINK+="lidar", GROUP="dialout", MODE="0660"`

- Auto-start a ROS camera node when camera connects:  
  `SUBSYSTEM=="video4linux", ACTION=="add", TAG+="systemd", ENV{SYSTEMD_WANTS}="ros-camera@%E{DEVNAME}.service"`

- Mark a USB joystick for user access:  
  `SUBSYSTEM=="input", KERNEL=="js*", TAG+="uaccess"`

---

## 📚 Related Concepts

- [[udev]]
- [[lsusb]]
- [[dmesg]]
- [[lspci]]
- [[Serial]]
- [[Linux]]
- [[systemd]]
- [[ROS2]] (Robot Operating System)

---

## 🌐 External Resources

- man page: `man udevadm`
- Kernel.org Documentation: `https://www.kernel.org/doc/html/latest/admin-guide/udev.html`
- Arch Wiki (reference-quality guide): `https://wiki.archlinux.org/title/udev`
- Spaceball Driver https://dev.to/denladeside/reviving-legacy-hardware-with-web-hid-3447
