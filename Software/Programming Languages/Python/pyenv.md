# pyenv

`pyenv` is a popular and flexible tool for managing multiple versions of Python on Unix-like systems. It allows developers to easily switch between different Python environments on a per-project basis, making it ideal for robotics, embedded systems, and other engineering contexts where compatibility and isolation are key. In robotics workflows, particularly where multiple toolchains or simulators rely on specific Python versions, `pyenv` streamlines the development process and reduces system conflicts.

---

## 🧰 Overview

`pyenv` provides a simple mechanism for installing and managing multiple Python versions alongside system Python. It modifies the `PATH` to prioritize your selected version and integrates with other tools like `pyenv-virtualenv` for even more fine-grained control over environments.

---

## 🧠 Core Concepts

- **Shims:** `pyenv` uses lightweight wrapper binaries ("shims") to intercept calls to `python`, `pip`, etc., and redirect them to the appropriate version.
- **Local/Global/Shell Settings:** You can set a global Python version, override it per-project (`.python-version` file), or even just for a single shell session.
- **Plugins:** Core functionality can be extended with community plugins like `pyenv-virtualenv` and `pyenv-update`.

---

## 📊 Comparison Chart

| Feature                    | pyenv         | conda           | virtualenv        | venv (builtin)     | Docker              |
|---------------------------|---------------|------------------|-------------------|--------------------|---------------------|
| Multiple Python versions  | ✅             | ✅                | ❌                | ❌                 | ✅                  |
| Environment isolation     | ✅ (via plugin)| ✅                | ✅                 | ✅                 | ✅                  |
| Package management        | ⛔️            | ✅ (conda/pip)    | ✅ (pip)           | ✅ (pip)            | ✅ (apt/pip/other)   |
| System independence       | ✅             | ✅                | ❌                | ❌                 | ✅                  |
| Easy version switching    | ✅             | ✅                | ❌                | ❌                 | ✅ (via image tag)  |
| GUI installers available  | ❌             | ✅                | ❌                | ❌                 | ✅ (Docker Desktop) |
| Good for embedded use     | ✅             | ⚠️ (bloat)        | ✅                 | ✅                 | ⚠️ (overhead)       |

---

## 🛠️ Use Cases

- Managing different Python versions across projects with ROS1 and ROS2 (which often require specific Python versions)
- Ensuring compatibility when compiling third-party C++ libraries with Python bindings (e.g., `pybind11`)
- Isolating Python interpreters on embedded systems like Raspberry Pi for deployment

---

## ✅ Strengths

- Lightweight and Unix-native (shell-based)
- Doesn't require root permissions
- Excellent for version pinning per project
- Broad community support and plugin ecosystem
- Simple installation and usage

---

## ❌ Weaknesses

- Not cross-platform (limited Windows support)
- No built-in virtual environment support (requires `pyenv-virtualenv`)
- May be confusing when mixed with `system` or `conda` Python setups
- Can cause path issues if shell configuration is incorrect

---

## 🧩 Compatible Items

- `pyenv-virtualenv` for environment management
- `pipx` for global CLI tools
- Robotics environments like [[ROS]] (Robot Operating System)
- [[Poetry]] (Python dependency management)
- [[pip]] (Python package installer)

---

## 📚 Related Notes

- [[Python]] (General purpose programming language)
- [[pip]] (Python package installer)
- [[Poetry]] (Python dependency management)
- [[Docker]] (Containerization platform)
- [[venv]]
- [[UV]]
- [[Conda]]
- [[Anaconda]]

---

## 🔗 External Resources

- Official site: https://github.com/pyenv/pyenv
- Installation script: https://github.com/pyenv/pyenv-installer
- Plugin list: https://github.com/pyenv/pyenv/wiki#available-plugins
- Setup guide: https://realpython.com/intro-to-pyenv/

---

## 📦 Documentation and Support

- GitHub Issues and Discussions
- Community guides (Real Python, Dev.to)
- Compatible with shell environments like bash, zsh, and fish
- Commonly used alongside `direnv` for project auto-loading

---
