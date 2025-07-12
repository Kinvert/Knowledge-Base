# UV

**UV** is a next-generation Python package manager developed by Astral (the team behind `pdm`). It is designed to be a drop-in replacement for tools like [[pip]], [[virtualenv]], and [[pip-tools]], while being significantly faster, more secure, and reproducible.

UV is written in Rust for performance and safety. It handles everything from dependency resolution to environment management in a single binary.

---

## 🧠 Overview

UV aims to provide:
- Fast dependency resolution and installation
- A unified CLI for managing Python packages and environments
- Compatibility with `requirements.txt` and `pyproject.toml`
- Lockfile generation with strict reproducibility

It is positioned as a **Rust-powered alternative** to the legacy Python tooling ecosystem, consolidating multiple tools into one.

---

## 🧪 Features

- ✅ Fast: Written in Rust, it outperforms `pip` by a wide margin  
- ✅ Reproducible: Lockfile ensures deterministic builds  
- ✅ Secure: Verifies package hashes and metadata  
- ✅ Compatible: Works with PEP 517/518/660 (modern Python packaging)  
- ✅ Unified: Combines the jobs of pip, virtualenv, pip-tools, and more  
- ✅ Offline support: Uses a local cache for installs

---

## ⚙️ Example Commands

- `uv pip install numpy` — Installs a package like `pip`  
- `uv venv create .venv` — Creates a virtual environment  
- `uv pip sync requirements.txt` — Syncs from a `requirements.txt` file  
- `uv pip freeze > requirements.txt` — Exports installed packages  
- `uv lock` — Generates a `uv.lock` file (similar to `poetry.lock`)  
- `uv pip install --editable .` — Installs in editable mode

---

## 📊 Comparison Table

| Tool         | Language | Speed  | Lockfiles | Virtualenvs | Offline Support | Notes                       |
|--------------|----------|--------|-----------|-------------|------------------|-----------------------------|
| UV           | Rust     | 🚀 Fast | ✅ Yes     | ✅ Yes       | ✅ Yes            | Modern all-in-one manager   |
| [[pip]]      | Python   | 🐢 Slow | ❌ No      | ❌ No        | 🟡 Partial        | Legacy standard tool        |
| [[Poetry]]   | Python   | 🟡 Med  | ✅ Yes     | ✅ Yes       | 🟡 Partial        | Higher-level but slower     |
| [[Conda]]    | C++/Py   | 🟡 Med  | ✅ Yes     | ✅ Yes       | ✅ Yes            | Binary-based env manager    |
| [[pipenv]]   | Python   | 🐢 Slow | ✅ Yes     | ✅ Yes       | ❌ No             | Complex dependency tree     |
| [[PDM]]      | Python   | 🟡 Fast | ✅ Yes     | ✅ Yes       | 🟡 Partial        | UV’s spiritual predecessor  |

---

## 🔗 Related Concepts

- [[pip]]  
- [[venv]]  
- [[PDM]]  
- [[Conda]]  
- [[Poetry]]  
- [[Python]]  
- [[Package Managers]]  
- [[Lockfile]]  
- [[Virtual Environments]]  
- [[Rust]]

---

## 📚 Further Reading

- https://github.com/astral-sh/uv  
- UV documentation (README and usage examples)  
- Blog posts comparing UV vs pip + virtualenv  
- Benchmarks showing UV performance

---
