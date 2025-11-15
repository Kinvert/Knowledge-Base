# TUI (Text-Based User Interface)

A **Text-Based User Interface (TUI)** is an interactive interface rendered in a terminal or console environment, offering richer interactivity than traditional command-line tools while remaining lightweight, scriptable, and highly portable. In robotics engineering, TUIs are frequently used for diagnostics, visualization of sensor data, debugging robot states, configuring embedded systems, and interacting with networked robotic resources where GUI access is limited.

---

## 🧭 Overview

TUIs bridge the gap between simple CLIs and full graphical interfaces. They offer structured layouts, keyboard navigation, color, panes, and widgets, while preserving CLI advantages such as low resource usage, SSH-friendliness, and ease of automation.

---

## 🧠 Core Concepts

- **Terminal Rendering**  
  TUIs draw characters, colors, and box elements in a controlled buffer instead of standard stdout text streams.

- **Event Loops**  
  TUIs generally use non-blocking input to handle keyboard events, timers, and updates.

- **Widgets & Layouts**  
  Libraries provide building blocks like tables, gauges, graphs, menus, dialogs, logs, and panes.

- **State Management**  
  User input updates application state, which re-renders portions of the screen.

- **Cross-Platform Considerations**  
  Handling differences between terminal emulators, escape sequences, and input models.

---

## 📊 Comparison Chart

| Technology | Type | Rich Interactivity | Common Language | Best For | Notes |
|-----------|------|--------------------|-----------------|----------|-------|
| **TUI** | Text-based | Medium–High | C++, Python, Go, Rust | Robotics tooling, diagnostics | SSH-friendly |
| **CLI** | Text-based | Low | Everything | Scripts, simple commands | No structured layout |
| **GUI** | Graphical | High | C++, Python, JS | Complex dashboards | Requires full desktop |
| **ncurses** | TUI Library | Medium | C/C++ | Foundational TUIs | Old but reliable |
| **Blessed/Blcurse** | TUI Library | Medium–High | Python/Node | Colorful, modern TUIs | Easier cross-platform |
| **fzf** | TUI Tool | Medium | Shell | Fuzzy searching | Popular standalone tool |

---

## 🧰 Use Cases

- Embedded device configuration over serial (`/dev/ttyUSB*`)
- Monitoring robotic arm joint positions or sensor streams
- Running headless robot diagnostics over SSH
- Displaying CAN bus data, planning trajectories, or IMU readings
- Text-based dashboards in CI pipelines for robotics
- TUI-based controllers or teleoperation panels for minimal environments

---

## ⭐ Strengths

- Lightweight, low RAM/CPU usage
- Works over SSH and serial links
- Fast startup, low latency
- Keyboard-driven high-efficiency workflows
- Excellent for log/stream monitoring
- More structured than plain CLI tools

---

## ⚠️ Weaknesses

- Limited visual fidelity compared to GUIs
- More complex to build than CLIs
- Terminal emulator inconsistencies
- No mouse/gesture support in many setups
- More constrained for spatial visualization than full graphics systems

---

## 🧩 Key Features

- Color and bold/underline support
- Panels, windows, and split layouts
- Interactive lists, tables, tree views
- Streaming data widgets (line charts, gauges)
- Hotkey systems
- Scrollback buffers
- Form inputs and dialogs
- Real-time updating dashboards

---

## 🔧 Developer Tools & Libraries

- **C/C++**: `ncurses`, `notcurses`, `termbox`
- **Python**: `urwid`, `textual`, `blessed`, `prompt_toolkit`
- **Go**: `tview`, `termui`, `bubbletea` (popular and modern)
- **Rust**: `ratatui`, `crossterm`
- **Node**: `blessed`, `neo-blessed`
- **Zig**: Uses wrappers over terminfo, minimal ecosystem but emerging

Use commands like `pip install textual` or `go get github.com/rivo/tview` depending on language.

---

## 🏗️ How It Works

1. **Initialize terminal** in raw or cbreak mode.
2. **Disable line buffering** and enable event polling.
3. **Create views/widgets** that define layout and state.
4. **Run an event loop** to process keyboard input.
5. **Render updates** by redrawing changed screen regions.
6. **Gracefully restore** terminal settings on exit.

---

## 🧮 Compatible Items

- [[CLI]] (Command Line Interfaces)
- [[Shell]] (Linux Shells)
- [[SSH]] (Secure Shell)
- [[Serial Protocols]] for embedded systems
- [[CAN Bus]] for diagnostics
- [[Microcontrollers]] (via UART debug interfaces)
- [[Dev Boards]] (accessible headless over SSH)

---

## 🔀 Variants

- **Full-Screen TUIs**: complex multi-pane dashboards
- **Inline TUIs**: small popups inside CLI tools
- **Menu-driven TUIs**: text menus for configuration
- **Streaming TUIs**: dashboards showing live logs or metrics
- **Wizard TUIs**: step-by-step configuration flows

---

## 📚 Related Concepts / Notes

- [[CLI]] (Command Line Interface)
- [[Shell]] (bash, zsh, fish)
- [[Terminal Emulators]] (alacritty, xterm)
- [[SSH]] (Secure Shell)
- [[Embedded System]] (for serial/TUI workflows)
- [[Linux Debugging Tools]] (for robot bring-up)
- [[Dev Boards]] (headless configuration)
- [[Visualization Tools]] (GUI alternatives)

---

## 🔗 External Resources

- `man 3 ncurses` for foundational TUI mechanics
- Textual (Python): textual.textualize.io  
- Bubbletea (Go): github.com/charmbracelet/bubbletea  
- Notcurses: github.com/dankamongmen/notcurses  

---

## 📝 Summary

Text-Based User Interfaces sit at the crossroads between CLI simplicity and GUI richness. For robotics engineers—who often work on remote systems, embedded devices, and headless compute modules—TUIs offer a valuable middle ground, enabling interactive visualization and control without the overhead of graphical stacks. Their flexibility, portability, and scripting-friendly nature make them an essential tool in complex robotics workflows.
