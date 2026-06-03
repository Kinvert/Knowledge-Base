# 🔌 usbipd

**usbipd** is a command-line tool and Windows service from the `usbipd-win` project that shares locally connected USB devices with USB/IP clients, especially [[WSL]] 2 distributions. It is useful for embedded development because Linux tools inside WSL can access USB programmers, serial adapters, debug probes, smartcard readers, and microcontroller boards that are physically plugged into Windows.

---

## 📚 Overview

WSL 2 runs inside a lightweight virtual machine and does not automatically receive direct access to arbitrary USB devices. `usbipd-win` fills that gap by exporting a Windows USB device over the USB/IP protocol, then attaching it to the WSL 2 Linux kernel as if it were plugged into the Linux VM.

The usual flow is:

1. Plug a USB device into Windows.
2. Run `usbipd list` to find its bus ID.
3. Run `usbipd bind --busid <BUSID>` once as Administrator to share it.
4. Run `usbipd attach --wsl --busid <BUSID>` from a normal Windows shell.
5. Optionally use `usbipd attach --wsl --busid <BUSID> --auto-attach` when you want a long-running process that reattaches the device after unplug/replug events.
6. Use `lsusb`, `dmesg`, `udev`, `openocd`, `avrdude`, `dfu-util`, or serial tools inside WSL.

---

## 🧠 Core Concepts

- **USB/IP**: Protocol for exporting USB requests over TCP/IP.
- **usbipd-win**: Windows host implementation that exports local USB devices.
- **Bus ID**: Windows-side USB topology identifier used by `usbipd`, such as `4-4`.
- **Bind**: Marks a Windows USB device as shared. This is persistent and requires Administrator privileges.
- **Attach**: Connects a shared USB device to a client, such as WSL 2. This is not persistent.
- **Detach**: Disconnects the USB device from WSL so Windows can use it again.
- **WSL 2 Kernel**: Linux kernel that must support USB/IP and the target USB class driver.
- **[[vhci_hcd]]**: Linux virtual host controller driver that receives USB/IP devices.

---

## ⚙️ Typical WSL Workflow

Run from an elevated PowerShell or Windows Terminal the first time:

```powershell
usbipd list
usbipd bind --busid 4-4
usbipd list
```

Then run from a normal Windows shell:

```powershell
usbipd attach --wsl --busid 4-4
usbipd list
```

For a long-running auto-reattach session:

```powershell
usbipd attach --wsl --busid 4-4 --auto-attach
```

`--auto-attach` keeps the command running and watches for the device to disconnect/reconnect. It is useful for boards that reset during flashing and briefly disappear from USB. Newer usbipd-win releases also support auto-attaching currently unplugged devices with `--unplugged`.

```powershell
usbipd attach --wsl --busid 4-4 --auto-attach --unplugged
```

Verify inside WSL:

```bash
lsusb
dmesg | tail
```

Detach from Windows when finished:

```powershell
usbipd detach --busid 4-4
```

If a device should no longer be shared at all:

```powershell
usbipd unbind --busid 4-4
```

---

## 🧰 Command Reference

| Command | Where | Admin? | Purpose |
|---|---|---|---|
| `usbipd --help` | Windows | No | Show installed command syntax |
| `usbipd list` | Windows | No | List USB devices and sharing state |
| `usbipd bind --busid <BUSID>` | Windows | Yes | Share a USB device for later attach |
| `usbipd attach --wsl --busid <BUSID>` | Windows | No | Attach shared device to WSL 2 |
| `usbipd attach --wsl --busid <BUSID> --auto-attach` | Windows | No | Keep running and reattach after disconnect/reconnect |
| `usbipd attach --wsl --busid <BUSID> --auto-attach --unplugged` | Windows | No | Wait for a currently unplugged shared device and attach when it appears |
| `usbipd detach --busid <BUSID>` | Windows | No | Detach device from WSL |
| `usbipd unbind --busid <BUSID>` | Windows | Yes | Stop sharing the device |
| `lsusb` | WSL | No | Verify Linux sees the attached USB device |
| `udevadm info` | WSL | No/root | Inspect Linux device metadata |

---

## 📊 Comparison Chart

| Method | Best For | Strength | Weakness | Typical Use |
|---|---|---|---|---|
| **usbipd-win + WSL 2** | USB devices in WSL | Uses Linux tools from Windows host | Requires attach flow | Embedded dev on Windows |
| Native Linux USB | Direct hardware access | Simplest Linux path | Requires Linux host | Daily firmware work |
| Windows native tools | Vendor IDEs | No VM bridge | Linux tools unavailable | ST, Arduino, vendor stacks |
| Hyper-V USB/IP | VM USB sharing | General VM support | More setup | Linux VM development |
| Serial-over-COM only | UART consoles | Simple for serial | Not real USB passthrough | Logs, REPLs |
| PCIe USB controller passthrough | Full controller access | Strong isolation | Hardware/VM complexity | Dedicated VMs |

---

## ✅ Pros

- Enables Linux USB tools inside WSL 2.
- Good for embedded boards, serial adapters, debug probes, and firmware flashing.
- `bind` is persistent, so devices only need to be shared once.
- `attach --wsl` does not require Administrator privileges after binding.
- `--auto-attach` helps boards that reset or reconnect during flashing.
- Keeps Windows as the host OS while using Linux hardware tooling.

---

## ❌ Cons

- Attached devices cannot be used by Windows at the same time.
- Attach state is not persistent across reboot, WSL restart, device reset, or unplug/replug.
- `--auto-attach` is long-running and does not create a permanent background policy by itself.
- Some devices need Linux kernel drivers not present in the default WSL kernel.
- Some workflows require [[udevadm]] rules inside WSL.
- USB timing, isochronous devices, and special vendor drivers can be fragile.
- Firewall or third-party security software can interfere with USB/IP traffic.

---

## 🛠️ Installation Notes

Install on Windows:

```powershell
winget install --interactive --exact dorssel.usbipd-win
```

The installer provides:

- the `usbipd` Windows service
- the `usbipd` command-line tool
- a firewall rule for the USB/IP service

For WSL, keep the kernel current:

```powershell
wsl --update
wsl --shutdown
```

Inside WSL, install inspection tools:

```bash
sudo apt update
sudo apt install usbutils
```

---

## 🔍 Troubleshooting

| Symptom | Likely Cause | Check |
|---|---|---|
| Device not in `usbipd list` | Windows did not enumerate it | Device Manager, cable, port |
| `bind` fails | Shell not elevated or device blocked | Run Administrator shell |
| `attach --wsl` fails | WSL not running or device not shared | Open WSL shell, rerun `usbipd list` |
| `lsusb` shows nothing | Attach failed or kernel support missing | `dmesg`, `wsl --update` |
| Device appears but app needs root | Missing udev rule | `/etc/udev/rules.d`, `udevadm` |
| Windows app loses device | Device attached to WSL | `usbipd detach --busid <BUSID>` |
| Device disappears after reset | Attach is non-persistent | Use `usbipd attach --wsl --busid <BUSID> --auto-attach` |

---

## 🔧 Embedded Development Examples

- Flashing Arduino-compatible boards from WSL.
- Using [[OpenOCD]] with ST-Link, J-Link, CMSIS-DAP, or FTDI probes.
- Accessing USB serial adapters through Linux tools.
- Running `dfu-util` or `avrdude` inside WSL.
- Talking to [[FT232H]] or libusb devices from Python.
- Building and loading Parallax Propeller code with [[propgcc]] and serial tools.

---

## 🔗 Related Notes

- [[vhci_hcd]]
- [[WSL]]
- [[Linux]]
- [[udevadm]]
- [[OpenOCD]]
- [[FT232H]]
- [[USB Storage Failures in Linux]]
- [[propgcc]]

---

## 🌐 External Resources

- usbipd-win GitHub: https://github.com/dorssel/usbipd-win
- usbipd-win WSL Support: https://github.com/dorssel/usbipd-win/wiki/WSL-support
- Microsoft WSL USB guide: https://learn.microsoft.com/en-us/windows/wsl/connect-usb
- Linux USB/IP protocol: https://docs.kernel.org/usb/usbip_protocol.html

---

## 📝 Summary

`usbipd` is the practical Windows-to-WSL USB bridge. For embedded development, the commands to remember are `usbipd list`, `usbipd bind --busid <BUSID>`, `usbipd attach --wsl --busid <BUSID>`, and `usbipd detach --busid <BUSID>`.
