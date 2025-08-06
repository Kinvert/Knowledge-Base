# ncurses

`ncurses` (new curses) is a programming library that provides a terminal-independent API for building text-based user interfaces (TUIs). It extends the functionality of the original `curses` library, offering developers fine-grained control over character-cell displays, windows, input handling, color management, and keyboard interactions — all within a terminal.

While not exclusive to robotics, `ncurses` is often used in embedded and real-time systems for diagnostics, debugging, and runtime visualization when a full graphical interface is impractical.

---

## 🧠 Overview

`ncurses` abstracts terminal capabilities and allows the creation of complex interfaces with windows, colors, forms, and panels using standard ANSI terminals. It enables the development of responsive and interactive console applications using C or C++, with bindings also available in Python and other languages.

It is widely used in environments where GUIs are unavailable, like remote SSH sessions or low-level embedded systems.

---

## 📘 Core Concepts

- **Windowing System:** Create and manage multiple virtual windows inside the terminal.
- **Color Pairs:** Define and reuse foreground/background color combinations.
- **Input Modes:** Handle keyboard input (blocking/non-blocking), function keys, and special characters.
- **Forms and Panels:** Modular UI components for creating input forms and layered interfaces.
- **Terminal Abstraction:** Uses `terminfo`/`termcap` to support a wide range of terminal types.

---

## 🛠️ Key Features

- Efficient rendering in constrained environments
- Portable across Unix-like operating systems
- Supports mouse events (in compatible terminals)
- Can redraw only updated portions of the screen
- Resilient to screen resizing and multi-window usage

---

## 📊 Comparison Chart

| Library           | GUI/TUI | Language       | Terminal-Based | Common Use Cases            | Notes                            |
|-------------------|---------|----------------|----------------|-----------------------------|----------------------------------|
| ncurses           | TUI     | C/C++          | ✅              | Diagnostics, Config, CLI UI | Lightweight and efficient        |
| [[Qt]]            | GUI     | C++            | ❌              | Full GUI apps               | Not terminal-based               |
| [[Dear ImGui]]    | GUI     | C++            | ❌              | Embedded debug UIs          | Great for real-time rendering    |
| [[urwid]]         | TUI     | Python         | ✅              | Python CLI tools            | Less performant than ncurses     |
| [[Blessed]]       | TUI     | Python         | ✅              | Terminal UIs with color     | High-level ncurses-like wrapper  |

---

## ✅ Strengths

- Lightweight and fast
- Ideal for headless and embedded environments
- Wide adoption and mature documentation
- Integrates well with low-level system diagnostics

---

## ⚠️ Weaknesses

- Steeper learning curve for layout and input handling
- Limited to terminal display (no graphics, audio, etc.)
- C-based API can feel low-level and verbose

---

## 🧪 Use Cases

- Text-based robot diagnostics tools
- Real-time sensor data visualization over SSH
- On-device configuration menus for headless robots
- Interactive CLI dashboards and status boards
- Embedded systems with no GUI support

---

## 🔗 Related Concepts

- [[TTY]] (Teletype Terminal)
- [[Shell Tools]]
- [[CLI]]
- [[Embedded Linux]]
- [[Cross Compilation]]
- [[Debugging Tools]]
- [[Real-Time Systems]]

---

## 🔧 Compatible Items

- POSIX-compliant systems (Linux, macOS, BSD)
- [[tmux]] or [[screen]] for session persistence
- [[GCC]] and [[Clang]] for compilation
- [[Python]] (via `curses` or `blessed`)
- [[CMake]] build system integration

---

## 🌐 External Resources

- [GNU ncurses homepage](https://invisible-island.net/ncurses/)
- [Linux Journal ncurses Primer](https://www.linuxjournal.com/article/11233)
- [tldp.org ncurses HOWTO](https://tldp.org/HOWTO/NCURSES-Programming-HOWTO/)
- [Python curses documentation](https://docs.python.org/3/library/curses.html)

---

## 📚 Further Reading

- *Programming with Curses* by John Strang
- *The Linux Programming Interface* by Michael Kerrisk
- ncurses source code (available via most distro package managers)

---
