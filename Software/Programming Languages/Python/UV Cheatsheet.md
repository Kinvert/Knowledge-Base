# uv Cheatsheet ⚡

A blazing-fast Python package manager built in Rust. Designed as a drop-in replacement for `pip`, `pip-tools`, `virtualenv`, and `venv`. Perfect for Python devs who want speed and reproducibility in robotics and automation workflows.

---

## 🚀 Install `uv`

Recommended via `pipx`:

`pipx install uv`

Or via `cargo`:

`cargo install --locked uv`

---

## 🧱 Create Virtual Environment

Create a venv and install from `pyproject.toml` (like Poetry, but faster):

`uv venv`

You can activate it with:

`source .venv/bin/activate`

Set the Python version explicitly:

`uv venv --python 3.11`

---

## 📦 Install Packages

Install and update dependencies:

`uv pip install requests`  
`uv pip install -r requirements.txt`

Install with `--upgrade`:

`uv pip install --upgrade numpy`

---

## 📄 Export Requirements

Lock and export dependencies to a requirements file:

`uv pip freeze > requirements.txt`

---

## 🔍 Check Environment

Check installed packages and Python version:

`uv pip list`  
`uv pip show scipy`  
`uv pip --version`  
`uv --version`

---

## 🔒 Sync Dependencies from pyproject.toml

Install only what’s declared:

`uv pip install --no-deps -r requirements.txt`

Or use `uv`'s planned future lock support (still evolving):

---

## 🧼 Clean Up

Remove unused or unpinned packages manually from your venv:

`uv pip uninstall some_package`

---

## ⚙️ Common One-Liners

Create + install from `pyproject.toml`:

`uv venv && source .venv/bin/activate && uv pip install -r requirements.txt`

Activate + freeze:

`source .venv/bin/activate && uv pip freeze > requirements.txt`

---

## ⚡ Key Strengths

- Fastest Python package installer (Rust-powered)
- Combines multiple tools: pip + venv + pip-tools
- Compatible with existing `requirements.txt` and `pyproject.toml`
- Great for CI and robotics where speed matters

---

## 🧩 Related Notes

- [[UV]]
- [[venv]] (Python virtual environments)
- [[pyenv]] (Python version management)
- [[pip]] (Python package manager)
- [[Poetry]] (Dependency + env management)
- [[Python]] (General language info)

---
