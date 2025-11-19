# tmux

**tmux** (Terminal Multiplexer) is a command-line tool that allows users to manage multiple terminal sessions inside a single window. It enables detachable sessions, split panes, persistent workflows, remote development stability, and more. In Reinforcement Learning and engineering workflows, tmux is invaluable for managing multiple experiment processes, logs, servers, data pipelines, and background training sessions on local or remote machines.

---

## 🧩 Overview

tmux provides a persistent shell environment where sessions continue running even when the terminal disconnects. This is especially critical for long-running RL training jobs, robotics simulations, continuous logging, and remote development via SSH. It offers windows, panes, session management, and extensive customization via the `.tmux.conf` file.

---

## 🧠 Core Concepts

- **Session:** A collection of windows that can be detached/attached  
- **Window:** A tab-like workspace within a session  
- **Pane:** Subdivision of a window (horizontal or vertical)  
- **Detach/Attach:** Keep processes running even when disconnected  
- **Prefix Key:** Default `Ctrl+b`, used to trigger tmux commands  
- **tmux.conf:** Configuration file for custom bindings, colors, behaviors  
- **Status Bar:** Customizable bottom bar with system and session info  

---

## 📊 Comparison Chart

| Tool | Type | Detach/Attach | Pane/Window System | Remote Stability | Ideal Use Case |
|------|------|----------------|---------------------|------------------|----------------|
| tmux | Terminal multiplexer | Yes | Yes | Excellent | RL training, server workflows |
| GNU Screen | Terminal multiplexer | Yes | Limited | Good | Minimalistic legacy workflows |
| iTerm2 splits | Terminal emulator | No | Yes | None | Local desktop workflows |
| VSCode SSH | IDE/remote dev | Partial | Panels | Good | Dev-focused remote editing |
| Byobu | Wrapper around tmux/screen | Yes | Yes | Good | User-friendly terminal UI |

---

## ⚙️ How It Works

tmux sits between the terminal emulator and the shell. When you launch tmux:

1. You create a **server** process  
2. Inside the server, you create **sessions**  
3. Sessions contain **windows**, each of which can have multiple **panes**  
4. These persist on the server until explicitly killed  
5. You interact via the **prefix key** followed by commands  

This makes tmux ideal for retaining processes even across network failures, SSH disconnects, or local terminal closures.

---

## 🛠️ Use Cases

- Running long RL training processes and monitoring logs  
- Splitting screens for simultaneous monitoring of metrics, logs, and model outputs  
- Remote SSH development with persistent sessions  
- Robotics simulation pipelines (Gazebo, ROS logs, RL loops)  
- Multiple service orchestration (database, API, training script)  
- Live tailing of datasets, rollout stats, and system metrics  
- Quick switching between shell tools  

---

## 🌟 Strengths

- Persistent sessions  
- Lightweight and fast  
- Highly customizable  
- Ideal for SSH and remote RL experiments  
- Powerful pane/window management  
- Excellent stability under flaky network connections  

---

## ⚠️ Weaknesses

- Default keybindings can be unintuitive  
- Learning curve for complex workflows  
- Configuration syntax can feel arcane  
- Sharing sessions has security considerations  

---

## 🔧 Developer Tools

- `.tmux.conf` for custom setup  
- Plugins via **tmux plugin manager (TPM)**  
- Popular plugins:  
  - tmux-sensible  
  - tmux-resurrect  
  - tmux-continuum  
  - tmux-prefix-highlight  
  - tmux-yank  

---

## 🧬 Variants / Related Tools

- **Byobu** (wrapper for tmux/screen)  
- **GNU Screen** (older multiplexer)  
- **Zellij** (modern Rust-based alternative)  
- **WezTerm Multiplexer** (cross-host persistent sessions)  

---

## 🔍 Related Concepts / Notes

- [[SSH]]  
- [[Bash]]  
- [[Linux]]  
- [[Logging]]  
- [[Experiment Tracking]]  
- [[RL Training Loop]]  
- [[System Monitoring]]  

---

## 🔌 Compatible Items

- Linux, macOS, BSD  
- Remote servers and cloud instances  
- GPU machines for RL training  
- ROS systems, simulation servers  
- Any terminal-based workflow  

---

## 📚 External Resources

- tmux GitHub repository  
- The official tmux manual (`man tmux`)  
- “tmux: Productive Mouse-Free Development” by Brian Hogan  
- TPM (tmux plugin manager) documentation  
- Community dotfiles and sample configs  

---

## 📝 Summary

tmux is a powerful, lightweight terminal multiplexer that allows engineers to run persistent, pane-rich, multi-window terminal sessions. It's an essential tool for reinforcement learning operations, long-running training processes, robotics simulations, and remote computing. With tmux, workflows become more resilient, organized, and efficient.
