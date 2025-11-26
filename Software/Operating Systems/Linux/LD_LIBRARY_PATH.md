# LD_LIBRARY_PATH 🛠️

`LD_LIBRARY_PATH` is an environment variable used in Unix-like systems to specify a list of directories where the dynamic linker should look for shared libraries (`.so` files) before searching the system default paths. It is commonly used for development, testing, or running applications with non-standard library locations.

---

## 🧠 Overview

- Affects how dynamically linked executables locate shared libraries at runtime.
- Useful when multiple versions of a library exist or libraries are installed in non-standard directories.
- Applies only to the process environment where it is set.
- Overrides `/etc/ld.so.conf` and default system paths like `/lib` and `/usr/lib`.

---

## ⚙️ Core Concepts

- **Dynamic Linking:** Executables can depend on shared libraries, loaded at runtime instead of being statically compiled.
- **Linker Search Path:** When an executable runs, the dynamic linker (`ld.so` or `ld-linux.so`) searches directories in:
  1. `LD_LIBRARY_PATH`
  2. `/etc/ld.so.cache` (updated via `ldconfig`)
  3. Default system paths (`/lib`, `/usr/lib`, `/usr/local/lib`)
- **Environment Variable Scope:** Only affects processes spawned with the variable set.
- **Precedence:** `LD_LIBRARY_PATH` takes priority over default system paths.

---

## 🔍 How It Works

1. Set the variable in the shell, e.g., `export LD_LIBRARY_PATH=/opt/mylibs:$LD_LIBRARY_PATH`
2. Launch a dynamically linked application.
3. The dynamic linker searches the directories in `LD_LIBRARY_PATH` in order.
4. If the required `.so` files are found, they are loaded into memory.
5. If not found, the linker falls back to default paths and may report a runtime error (`cannot open shared object file`).

---

## 📊 Comparison Chart

| Feature / Tool                | LD_LIBRARY_PATH         | /etc/[[ld.so.conf]] + [[ldconfig]] | rpath / RUNPATH in ELF |
|-------------------------------|----------------------|---------------------------|-----------------------|
| Scope                        | Per-process           | System-wide               | Binary-specific       |
| Overrides system libraries     | Yes                  | No                        | Yes                   |
| Persistent                     | No (until unset)     | Yes                       | Yes                   |
| Ease of use                     | Simple, temporary    | Requires root and reload  | Needs recompilation   |
| Common Use Case                | Testing libraries    | Installing new shared libs| Shipping portable binaries |

---

## ✅ Use Cases

- Running software with custom or newer library versions.
- Testing development builds without installing them system-wide.
- Avoiding conflicts with system libraries.
- Running portable binaries on multiple systems.

---

## ⚡ Strengths

- Easy to set and temporary, requiring no root access.
- Flexible for development and testing.
- Allows overriding system libraries safely for specific processes.

---

## ❌ Weaknesses

- Can lead to version conflicts or unexpected behavior if set incorrectly.
- May be ignored if the executable uses a fixed `rpath` or is statically linked.
- Not ideal for long-term library management.

---

## 🛠 Developer Tools

- `ldd`: Inspect which shared libraries an executable will load.
- `readelf -d`: Check ELF dynamic section for `RPATH` or `RUNPATH`.
- `ldconfig`: Update system-wide cache for shared libraries.
- Shell commands: `export LD_LIBRARY_PATH=/path/to/libs:$LD_LIBRARY_PATH`

---

## 📚 Related Concepts / Notes

- [[Dynamic Linking]] (Runtime linking of shared libraries)
- [[RPATH]] (ELF binaries library path)
- [[PATH]]
- [[Shared Libraries]] (Files with `.so` extensions)
- [[ldconfig]] (System-wide library cache management)
- [[Environment Variables]] (Process-level settings)

---

## 🔗 External Resources

- `man ld.so`: Linux dynamic linker documentation
- `man ldconfig`: Cache and system library management
- GNU Libc documentation: `https://www.gnu.org/software/libc/manual/html_node/`

---

## ⚙️ Variants

- `DYLD_LIBRARY_PATH` (macOS equivalent)
- `LD_PRELOAD` (Force loading of specific libraries before others)
- `LIBRARY_PATH` (Used at compile-time instead of runtime)

---

## 🌐 Compatible Items

- ELF binaries on Linux/Unix
- Shared object libraries (`.so`)
- Shell environments: bash, zsh, sh

---

## 🏷 Key Highlights

- Per-process override for shared library paths
- Temporary and flexible for development/testing
- Must be carefully managed to avoid runtime conflicts

---

## 📖 Further Reading

- Linux From Scratch: Managing shared libraries
- "Advanced Linux Programming" chapters on dynamic linking
- Online tutorials on LD_LIBRARY_PATH and RPATH interplay
