# Neofetch

**Neofetch** is a command-line system information tool that displays an overview of your operating system, hardware, and software in a visually appealing format. It is highly customizable and often used in screenshots, status bars, or just for personal reference. While not directly related to Reinforcement Learning, it can be useful for documenting the environment on which experiments are run.

---

## 🧭 Overview

Neofetch collects system information such as:

- OS and version
- Kernel version
- CPU and GPU details
- Memory usage
- Shell and terminal info
- Uptime
- Disk usage

It displays this information alongside an ASCII logo of your OS or custom logo. Written in **Bash**, Neofetch runs on Linux, macOS, Windows (via WSL or Git Bash), and BSD systems.

---

## 🧠 Core Concepts

- **ASCII Art Display** – Uses text-based logos for aesthetic presentation  
- **System Probing** – Reads system files and executes commands to detect hardware/software  
- **Highly Configurable** – Users can select which modules to display, colors, and logo  
- **Scriptable** – Can be run in scripts or automated to document system info  
- **Cross-Platform Support** – Linux, macOS, Windows (WSL, Git Bash), BSD  
- **Lightweight** – Minimal dependencies, runs fast in the terminal

---

## ⚙️ How It Works

1. Detects the operating system and version  
2. Reads system info from `/proc`, system commands, or APIs depending on platform  
3. Formats output according to configuration file (~/.config/neofetch/config.conf)  
4. Prints ASCII logo and system information to terminal  

Command example:  
`neofetch` – runs with default config  
`neofetch --off logo` – hides the logo  
`neofetch --cpu_temp C` – shows CPU temperature in Celsius

---

## 📊 Comparison Chart

| Tool | Platform | Display Style | Customizability | Use Case |
|------|---------|---------------|----------------|----------|
| **Neofetch** | Linux, macOS, WSL, BSD | ASCII / text | High | System info for screenshots, logging |
| [[Screenfetch]] | Linux, macOS | ASCII | Moderate | Similar to Neofetch, less maintained |
| [[Archey]] | Linux | ASCII | Low | Lightweight, minimal info |
| [[Conky]] | Linux | Graphical + text | Very High | Desktop system monitoring |
| [[Bashtop]] / [[Bpytop]] | Linux | Graphical in terminal | High | Resource monitoring in terminal |

---

## 🚀 Use Cases

- Display system information in terminal screenshots  
- Quickly document machine environment for reproducibility in ML/RL experiments  
- Add system info to startup scripts or status bars  
- Lightweight monitoring of system resources in terminal  

---

## ⭐ Strengths

- Extremely configurable  
- Cross-platform support  
- Lightweight and fast  
- Aesthetically pleasing  
- Supports custom logos and ASCII art  
- Easy to install via package managers (`apt`, `brew`, `pacman`)

---

## ⚠️ Weaknesses

- Primarily for display; does not actively monitor system resources  
- Limited interactivity  
- Some features dependent on platform support (e.g., CPU temperature detection)  
- Not suitable for detailed system diagnostics  

---

## 🔧 Variants

- **Neofetch Classic** – default maintained version  
- **Screenfetch** – predecessor with similar functionality  
- **Archey** – minimal, lightweight ASCII system info  
- **Conky** – graphical terminal/desktop display with more detailed info  

---

## 🧩 Compatible Items

- Linux distributions (Ubuntu, Arch, Fedora, Debian, etc.)  
- macOS terminals  
- Windows via WSL or Git Bash  
- Terminal emulators supporting ANSI colors  
- Scripts for system logging or RL experiment reproducibility  

---

## 🧷 Related Concepts / Notes

- [[Terminal]]
- [[CLI]]
- [[System Info]] (General hardware and OS information)  
- [[Bash Scripts]] (Configuration and automation)  
- [[CLI Tools]] (Command-line utilities)  
- [[Monitoring]] (System resource tracking)  
- [[Reproducibility]] (Tracking system environment for ML/RL experiments)

---

## 📚 External Resources

- [Official Neofetch GitHub](https://github.com/dylanaraps/neofetch)  
- Documentation via `man neofetch`  
- Community examples for custom ASCII logos  
- Tutorials for integrating Neofetch in `.bashrc` or `.zshrc`

---

## 🏗️ Developer Tools

- Git for installation and updates  
- Terminal emulator with ANSI color support  
- Package managers: `apt`, `yum`, `brew`, `pacman`  

---

## 📝 Documentation & Support

- GitHub Issues for bug reports  
- Wiki on GitHub for advanced configuration  
- Community forums and Reddit posts for custom themes and integration tips  

---

## 🏁 Summary

Neofetch is a versatile, lightweight, and visually appealing tool for displaying system information in the terminal. While not a monitoring tool, its customizability and cross-platform support make it ideal for documenting environments, especially in RL or ML research where reproducibility is important.

---

**Additional Suggestion:**  
If you later document other terminal utilities like `htop`, `bpytop`, or `screenfetch`, consider a subfolder `Software/CLI Tools/System Info` for grouping visually similar utilities.
