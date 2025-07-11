# Flatpak

**Flatpak** is a universal Linux application packaging and distribution system designed to work across different distributions. It provides sandboxed, self-contained application bundles that include their dependencies, ensuring consistent behavior regardless of the host system.

---

## 📚 Overview

Flatpak packages applications with their runtimes and dependencies, isolating them from the host environment using sandboxing technologies (like Bubblewrap). This approach helps avoid "dependency hell" and allows developers to distribute desktop and graphical applications that run consistently across various Linux distributions. It is an alternative to package managers like Snap and AppImage.

---

## 🧠 Core Concepts

- **Flatpak Runtime**: A common set of libraries and frameworks shared by multiple Flatpak apps to reduce duplication  
- **Sandboxing**: Uses Linux namespaces and cgroups to isolate apps from the host system  
- **Portals**: Secure interfaces that allow sandboxed apps controlled access to resources like files, network, or hardware  
- **Repositories (Remotes)**: Sources from which Flatpaks are installed, such as Flathub (default public repo)  
- **Manifest Files**: Describe how to build and bundle Flatpak applications  

---

## 🧰 Use Cases

- Distributing graphical Linux apps with consistent dependencies  
- Running newer versions of software on older or diverse Linux distros  
- Ensuring security by limiting app access to system resources  
- Testing multiple versions of an app without conflicts  
- Delivering desktop tools in robotics or simulation environments  

---

## ✅ Pros

- Distribution-agnostic, runs on most major Linux distros  
- Strong sandboxing improves security  
- Efficient sharing of runtimes reduces disk space usage  
- Flexible permission controls via portals  
- Easy installation and update mechanisms  

---

## ❌ Cons

- Sandboxing can limit hardware or system integration  
- Larger disk usage compared to native packages due to runtimes  
- Some apps require manual permission adjustments  
- Not as widely adopted for command-line or server tools  
- Startup time can be slower than native apps  

---

## 📊 Comparison Chart: Flatpak vs Snap vs AppImage

| Feature              | Flatpak           | [[Snap]]          | [[AppImage]]       |
|----------------------|-------------------|-------------------|--------------------|
| Distribution Support | Most Linux distros | Most Linux distros | Most Linux distros  |
| Sandboxing           | Yes (Bubblewrap)  | Yes (AppArmor)    | No                 |
| Dependency Handling  | Shared runtimes   | Bundled dependencies | Bundled dependencies |
| Auto Updates         | Yes               | Yes               | No (manual updates) |
| Package Size         | Moderate          | Larger            | Small to moderate   |
| GUI Apps Support     | Excellent         | Excellent         | Good               |
| CLI/Server Tools     | Less common       | Supported         | Supported          |

---

## 🔧 Useful Commands (One-Liners)

- `flatpak install flathub org.example.App` – Install app from Flathub  
- `flatpak run org.example.App` – Run a Flatpak app  
- `flatpak update` – Update all installed Flatpaks  
- `flatpak uninstall org.example.App` – Remove a Flatpak app  
- `flatpak list` – List installed Flatpak apps and runtimes  
- `flatpak info org.example.App` – Show info about an installed Flatpak  

---

## 🔧 Compatible Items

- Linux desktop applications for robotics visualization or simulation tools  
- [[Flatpak Repositories]] like Flathub  
- [[Bubblewrap]] sandboxing technology  
- Desktop environments such as GNOME, KDE that integrate with Flatpak apps  

---

## 🔗 Related Concepts

- [[Snap]] (Canonical’s universal Linux packaging alternative)  
- [[AppImage]] (Portable Linux application format)  
- [[Bubblewrap]] (Sandboxing utility used by Flatpak)  
- [[Linux Namespaces]] (Kernel feature enabling sandboxing)  
- [[Flatpak Repositories]] (Sources for Flatpak apps)  

---

## 📚 Further Reading

- [Flatpak Official Website](https://flatpak.org/)  
- [Flatpak Documentation](https://docs.flatpak.org/en/latest/)  
- [Flathub – The Flatpak App Store](https://flathub.org/)  
- [Understanding Flatpak Sandboxing](https://docs.flatpak.org/en/latest/sandbox-permissions.html)  
- [Comparison of Linux Packaging Formats](https://itsfoss.com/snap-flatpak-appimage/)  

---
