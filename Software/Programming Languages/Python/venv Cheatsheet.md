# venv Cheatsheet 🐍

Quick reference for Python’s built-in `venv` module. Ideal for robotics engineers switching between environments quickly and cleanly.

---

## 🔧 Create Virtual Environment

Create a new virtual environment in a folder named `.venv` (common convention):

`python3 -m venv .venv`

Use any name you want (e.g. `myenv`) but `.venv` keeps it hidden and standard:

`python3 -m venv myenv`

---

## 🚀 Activate Environment

- **Bash / Zsh (Linux/macOS):**  
  `source .venv/bin/activate`

- **Fish:**  
  `source .venv/bin/activate.fish`

- **C Shell (csh):**  
  `source .venv/bin/activate.csh`

- **Windows (PowerShell):**  
  `.venv\Scripts\Activate.ps1`

- **Windows (CMD):**  
  `.venv\Scripts\activate.bat`

---

## ❌ Deactivate Environment

Leave the virtual environment and return to system Python:

`deactivate`

---

## 📦 Install Packages

Install packages into the active venv:

`pip install some_package`

Freeze installed packages:

`pip freeze > requirements.txt`

Reinstall from freeze file:

`pip install -r requirements.txt`

---

## 🧪 Check Python Version

Ensure you're using the venv’s Python:

`which python`  
`which pip`

Should show paths inside `.venv`

---

## 🔁 Remove Environment

Simply delete the venv folder:

`rm -rf .venv`  
(on Windows: `rmdir /s /q .venv`)

---

## 📂 Common Patterns

Initialize venv in new repo:

`python3 -m venv .venv && source .venv/bin/activate && pip install -r requirements.txt`

Add to `.gitignore`:

`.venv/`

---

## 🧪 ROS-Specific Example

ROS 2 Humble:

`python3 -m venv ros2_env && source ros2_env/bin/activate && pip install -U colcon-common-extensions`

---

## ✅ Good Practices

- Use `.venv` for per-project isolation
- Add `.venv` to `.gitignore`
- Use `source .venv/bin/activate` in project README/dev docs
- Use `python3 -m venv` instead of relying on `virtualenv` unless needed

---

## 🧩 Related Notes

- [[pyenv]] (Python version management)
- [[pip]] (Python package manager)
- [[Poetry]] (Python dependency and venv manager)
- [[venv]]
- [[UV]]

---
