# WezTerm

**WezTerm** is a GPU-accelerated, cross-platform terminal emulator and multiplexer written in [[Rust]]. Created by Wez Furlong, it combines the functionality of a modern terminal with built-in multiplexing (similar to [[tmux]]), extensive Lua-based configuration, and support for advanced features like ligatures, image protocols, and remote session management.

---

## ⚙️ Overview

WezTerm runs on Linux, macOS, Windows, FreeBSD, and NetBSD with a consistent experience across all platforms. Unlike minimal terminals like Alacritty, WezTerm prioritizes features - native tabs, splits, workspaces, SSH client, serial port connections, and a full multiplexer that works across local and remote hosts.

Key highlights:
- GPU-accelerated rendering (WebGPU: Metal/Vulkan/DX12)
- Built-in multiplexer with panes, tabs, and workspaces
- Lua scripting for deep customization
- Hot-reloadable configuration
- Supports Sixel, Kitty graphics, and iTerm2 image protocols

---

## 🧠 Core Concepts

- **Workspaces:** Named session groups containing windows, tabs, and panes
- **Multiplexer:** Single-process mux that persists across disconnects (like tmux)
- **Domains:** Local, SSH, serial, or Unix socket connections
- **wezterm.lua:** Hot-reloadable Lua config file
- **Key Tables:** Modal keybinding system for complex shortcuts
- **Quick Select:** Mouse-free text selection and copying

---

## 📊 Comparison Chart

| Terminal | Language | GPU | Multiplexer | Ligatures | Image Protocols | Platforms | Config |
|----------|----------|-----|-------------|-----------|-----------------|-----------|--------|
| **WezTerm** | Rust | WebGPU | Built-in | Yes | Sixel, Kitty, iTerm2 | Linux, macOS, Windows, BSD | Lua |
| Alacritty | Rust | OpenGL | No | No | No | Linux, macOS, Windows, BSD | TOML |
| Kitty | C/Python | OpenGL | Partial | Yes | Kitty, Sixel | Linux, macOS | Plain text |
| iTerm2 | Obj-C | Yes | tmux integration | Yes | iTerm2, Sixel | macOS only | GUI/JSON |
| Ghostty | Zig | Metal/OpenGL | No | Yes | Kitty | Linux, macOS | Plain text |
| Hyper | Electron | WebGL | No | Plugin | No | Linux, macOS, Windows | JavaScript |
| Windows Terminal | C++ | DirectX | No | Yes | No | Windows only | JSON |

---

## ⚡ Performance

WezTerm uses WebGPU for GPU acceleration, supporting Metal (macOS), Vulkan (Linux), and DirectX 12 (Windows). Input latency benchmarks show WezTerm competitive with Kitty and slightly ahead of iTerm2. Memory usage is moderate - heavier than Alacritty (~30MB) but lighter than Electron-based terminals.

---

## 🔧 Key Features

- **Multiplexing:** Native panes, tabs, windows without tmux
- **SSH Integration:** Built-in SSH client with native tabs per connection
- **Serial Ports:** Direct connection to Arduino/embedded devices
- **Font Rendering:** Ligatures, color emoji, font fallback chains
- **Hyperlinks:** Clickable URLs in terminal output
- **Searchable Scrollback:** Regex search through history
- **Copy Mode:** Vim-like keyboard-driven text selection
- **Status Bar:** Customizable bar with Lua scripting

---

## 🚀 Use Cases

| Use Case | Why WezTerm |
|----------|-------------|
| Cross-platform dev | Same config/behavior on Linux, macOS, Windows |
| Remote development | Built-in mux persists sessions like mosh+tmux |
| Embedded/Arduino | Native serial port support |
| Power users | Lua scripting for complex workflows |
| tmux refugees | Built-in multiplexer, no external dependency |

---

## 🌟 Strengths

- True cross-platform with identical config
- Built-in multiplexer eliminates tmux dependency
- Lua scripting enables dynamic configuration
- Hot reload - no restart needed for config changes
- All major image protocols supported
- Active development and responsive maintainer

---

## ⚠️ Weaknesses

- Heavier resource usage than minimal terminals
- Lua config has learning curve vs simple TOML/JSON
- Less mature than iTerm2 on macOS
- No native tmux integration (unlike iTerm2)
- Documentation can be overwhelming

---

## 🔩 Configuration Example

```lua
-- ~/.config/wezterm/wezterm.lua
local wezterm = require 'wezterm'
local config = {}

config.font = wezterm.font 'JetBrains Mono'
config.font_size = 12.0
config.color_scheme = 'Catppuccin Mocha'
config.enable_tab_bar = true
config.window_padding = { left = 10, right = 10, top = 10, bottom = 10 }

return config
```

---

## 🔗 Related Notes

- [[tmux]]
- [[Rust]]
- [[SSH]]
- [[Linux]]
- [[Lua]]
- [[GPU]]

---

## 🌐 External Resources

- [WezTerm Official Site](https://wezterm.org/)
- [GitHub Repository](https://github.com/wezterm/wezterm)
- [Features Documentation](https://wezterm.org/features.html)
- [Configuration Reference](https://wezterm.org/config/files.html)

---

## 📝 Summary

WezTerm is a feature-rich, GPU-accelerated terminal emulator that combines modern rendering with built-in multiplexing and deep Lua customization. Its cross-platform consistency and native SSH/serial support make it ideal for developers working across multiple systems or managing remote sessions without external tools like tmux.
