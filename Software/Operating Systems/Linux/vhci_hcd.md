# 🧩 vhci_hcd

**vhci_hcd** is the Linux USB/IP virtual host controller driver. It lets a Linux machine import a USB device exported by another host and expose that remote device to the local Linux USB stack as if it were attached to a local USB controller.

---

## 📚 Overview

In USB/IP, one machine exports a physical USB device and another machine imports it. The importing side needs a virtual USB host controller. On Linux, that role is handled by `vhci_hcd` / `vhci-hcd`.

This is why [[usbipd]] works with [[WSL]] 2: Windows exports the USB device, and the WSL Linux kernel imports it through the USB/IP client path. The device then appears to normal Linux tools such as `lsusb`, `dmesg`, libusb, serial drivers, or firmware flashing utilities.

---

## 🧠 Core Concepts

- **VHCI**: Virtual Host Controller Interface.
- **HCD**: Host Controller Driver, the Linux layer for USB host controllers.
- **USB/IP Client**: Machine importing the remote USB device.
- **USB/IP Server**: Machine exporting the physical USB device.
- **URB**: USB Request Block, the kernel object representing a USB transfer.
- **`vhci-hcd` Module**: Loadable kernel module name on many systems.
- **`CONFIG_USBIP_VHCI_HCD`**: Kernel configuration option for the driver.

---

## ⚙️ How It Works

1. A USB/IP server exports a physical USB device.
2. The client asks for the exported device list.
3. The client imports one bus ID.
4. `vhci_hcd` creates a virtual USB host controller port.
5. USB request traffic is sent over TCP/IP to the exporting host.
6. The local Linux USB stack binds normal device drivers to the imported device.

---

## 🧰 Commands

Check whether the module is loaded:

```bash
lsmod | grep vhci
```

Try loading it on a normal Linux system:

```bash
sudo modprobe vhci-hcd
```

Check kernel config:

```bash
zgrep CONFIG_USBIP_VHCI_HCD /proc/config.gz
```

Inspect USB devices after attaching:

```bash
lsusb
dmesg | tail
```

In WSL, the driver may be built into the kernel or managed by the WSL kernel package rather than loaded manually.

---

## 📊 Comparison Chart

| Driver / Layer | Role | Physical Hardware? | Used For | Notes |
|---|---|---|---|---|
| **vhci_hcd** | Virtual USB host controller | No | USB/IP client/import side | Used by WSL USB attach |
| `usbip-host` | Export physical USB devices | Yes | USB/IP server side on Linux | Host-side driver |
| `xhci_hcd` | USB 3 host controller | Yes | Modern physical USB ports | Common PC USB controller |
| `ehci_hcd` | USB 2 host controller | Yes | Older physical USB ports | Legacy high-speed USB |
| `uhci_hcd` / `ohci_hcd` | USB 1.x host controllers | Yes | Legacy USB | Mostly historical |
| USB gadget drivers | Device-side USB | Usually yes | Linux acting as USB device | Opposite direction |

---

## ✅ Pros

- Lets remote USB devices appear as normal local Linux USB devices.
- Works with standard Linux USB drivers after attach.
- Useful for WSL, VMs, remote labs, and embedded development.
- Keeps device-level tools like `lsusb`, libusb, and serial drivers usable.
- Separates USB transport from the application using the device.

---

## ❌ Cons

- Depends on kernel support and correct USB class drivers.
- Network latency can affect timing-sensitive USB devices.
- Some USB transfer types or vendor-specific behavior may be fragile.
- Debugging spans kernel modules, USB/IP tools, network transport, and device drivers.
- Security matters because USB devices can be dangerous attack surfaces.

---

## 🔍 WSL Notes

For WSL 2 and `usbipd-win`:

- Keep WSL updated with `wsl --update`.
- Use `usbipd attach --wsl --busid <BUSID>` from Windows.
- Verify inside WSL with `lsusb`.
- Add [[udevadm]] rules when non-root tools need device access.
- If the device class driver is missing, a custom WSL kernel may be required.

---

## 🔗 Related Notes

- [[usbipd]]
- [[WSL]]
- [[Linux]]
- [[USB Storage Failures in Linux]]
- [[udevadm]]
- [[OpenOCD]]
- [[FT232H]]

---

## 🌐 External Resources

- Linux USB/IP protocol: https://docs.kernel.org/usb/usbip_protocol.html
- usbipd-win WSL Support: https://github.com/dorssel/usbipd-win/wiki/WSL-support
- Linux USB Host Side API: https://www.kernel.org/doc/html/latest/driver-api/usb/usb.html

---

## 📝 Summary

`vhci_hcd` is the Linux virtual USB host controller used by USB/IP clients. If `usbipd` is the Windows-side sharing tool, `vhci_hcd` is the Linux-side kernel mechanism that makes the attached USB device appear usable inside WSL or another Linux client.
