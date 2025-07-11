# Chocolatey

**Chocolatey** is a package manager for Windows, designed to automate the installation and configuration of software via the command line. It's widely used for DevOps, system provisioning, and automating robotics or embedded development environments on Windows machines.

---

## 📚 Overview

Chocolatey leverages **PowerShell** and **NuGet** infrastructure to install `.exe`, `.msi`, and `.zip` files. It can manage system software, SDKs, CLI tools, and libraries that are essential for engineering workflows—making it a Windows analog to tools like `apt`, `brew`, or `winget`.

---

## 🧠 Core Concepts

- **Packages**: `.nupkg` files that wrap installers or scripts
- **Sources/Repos**: Community repository (chocolatey.org) or private/internal repos
- **PowerShell Automation**: Installations run via scripted PowerShell commands
- **Package Cache**: Stores previously downloaded packages for offline reuse
- **Custom Packages**: Organizations can create internal `.nuspec` files for private deployment

---

## 🧰 Use Cases

- Set up robotics development environments on Windows (Python, CMake, Git, etc.)
- Install drivers, SDKs, and dependencies across teams in a reproducible way
- Automate CI/CD pipelines or provisioning scripts for engineering workstations
- Manage security updates for CLI tools like Git, OpenSSL, or Docker CLI

---

## ✅ Pros

- Windows-first: best option for automating installs on Windows
- Works with EXEs, ZIPs, and MSIs—not just language-specific libraries
- Custom internal repositories supported for enterprise use
- Easy to integrate into DevOps and IT workflows
- CLI-first interface allows scripting and automation

---

## ❌ Cons

- Slower package update cadence in the public repo
- GUI-based installers can cause failures if not scripted properly
- Not as commonly used in academic or Linux-first robotics projects
- Some packages lag behind official vendor releases

---

## 📊 Comparison Chart

| Feature                 | Chocolatey         | winget             | apt                | pip                | choco + scoop       |
|-------------------------|--------------------|---------------------|--------------------|--------------------|----------------------|
| OS Support              | ❌ Windows-only     | ❌ Windows-only     | ✅ Linux-only      | ✅ All OS           | ❌ Windows-only       |
| Binary Support          | ✅ EXE/MSI/ZIP      | ✅ MS Store/EXE     | ✅ DEB binaries     | ❌ Python-only      | ✅ (Scoop focuses on ZIP) |
| Language Packages       | ⚠️ Not primary       | ❌ No               | ✅ Some (via DEBs)  | ✅ Python           | ❌ (system only)       |
| GUI Integration         | ❌ CLI only         | ✅ Built-in GUI     | ❌ CLI              | ❌ CLI              | ❌ CLI only            |
| Scriptable              | ✅ PowerShell       | ⚠️ Basic            | ✅ Bash/Shell       | ✅ Python CLI       | ✅ PowerShell          |

---

## 🤖 In a Robotics Context

| Scenario                              | Chocolatey Benefit                              |
|---------------------------------------|--------------------------------------------------|
| Install CMake, Git, VSCode            | Automate setup of a Windows robotics dev env     |
| Manage Python and pip installs        | Consistent setup across engineers' workstations  |
| Prepare a Windows CI machine          | Ensure reproducible toolchains for builds        |
| Setup Docker CLI on Windows           | Avoid manual download of Windows Docker tools    |
| Embedded dev toolchains (e.g., ARM)   | Install toolchains via Chocolatey                |

---

## 🔧 Useful Commands (One-Liners)

- `choco install git` – Install Git  
- `choco install cmake --installargs '"ADD_CMAKE_TO_PATH=System"'` – Install CMake and add to PATH  
- `choco upgrade all` – Upgrade all packages  
- `choco list -lo` – List installed packages  
- `choco uninstall nodejs` – Uninstall Node.js  
- `choco install vscode python3` – Install multiple packages  

---

## 🔧 Compatible Items

- [[CMake]] – Install via Chocolatey
- [[Python]] – Windows Python installs managed easily
- [[Docker]] – Chocolatey provides CLI-only Docker installations
- [[CI-CD Pipelines]] – Can be used in GitHub Actions or Jenkins on Windows runners
- [[Windows Terminal]] – Install via Chocolatey

---

## 🔗 Related Concepts

- [[apt]] (Linux equivalent)  
- [[Homebrew]] (macOS alternative)  
- [[winget]] (Modern Windows package manager)  
- [[pip]] (Language-specific, often installed alongside)  
- [[Dockerfile]] (Chocolatey can be used in Windows containers)  
- [[CI-CD Pipelines]] (Used to provision runners with needed tools)

---

## 📚 Further Reading

- [Chocolatey Official Site](https://chocolatey.org/)
- [Installing Chocolatey](https://chocolatey.org/install)
- [Creating a Chocolatey Package](https://docs.chocolatey.org/en-us/create/create-packages/)
- [choco CLI Commands](https://docs.chocolatey.org/en-us/choco/commands/)
- [Comparison: Chocolatey vs winget](https://chocolatey.org/blog/windows-package-manager-comparison)

---
