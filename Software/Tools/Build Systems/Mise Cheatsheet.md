# mise — Cheatsheet (Command + Description Format)

This is a detailed **mise** cheat sheet listing commands, one-liner usage, descriptions, and common options. It covers core functionality, per-project workflows, and support for Zig, Elixir, Ash, PostgreSQL, C, Python, and more.

---

## 🧭 Core Commands

- `mise install <tool@version>` — Installs a specific tool version without activating it; can use `-v` for verbose output.  
- `mise use <tool@version>` — Activates a tool for the current project, writes/updates `mise.toml`, optional `-v` for verbose.  
- `mise exec <tool@version> -- <command>` — Runs a one-off command with the specified tool active; `--dry-run` shows the command without executing.  
- `mise ls` — Lists installed tools for the current machine; add `-v` for full paths and metadata.  
- `mise ls-remote <tool>` — Shows available remote versions; `--all` lists full historical versions.  
- `mise latest <tool>` — Displays the latest available version of a tool.  
- `mise install-into <tool@version> <path>` — Installs a tool into a custom path instead of the default mise directory.  
- `mise reshim` — Refreshes shims and executable links after installing tools or escripts.  
- `mise link <tool@version> <path>` — Links a pre-installed tool from a local path into mise’s management.  
- `mise lock` — Locks current project tools to exact versions in a lockfile for reproducibility.  
- `mise implode` — Removes project-level mise configuration; `-n` runs a dry-run without deletion.  
- `mise generate <type>` — Generates CI, devcontainer, or shell scripts based on project tools; e.g., `mise generate github-action`.  
- `mise info <tool>` — Shows detailed info about a specific tool including installed versions, paths, and backends.  
- `mise uninstall <tool@version>` — Removes a specific tool version.  
- `mise --help` — Displays general help; subcommands support `mise help <command>` for detailed usage.
- `mise doctor`
- `mise help activate`
- `mise setup`

---

## 🧩 Stack-Specific Usage

### Zig
- `mise install zig@0.11` — Install Zig version 0.11.  
- `mise use zig@0.11` — Activate Zig 0.11 for project; updates `mise.toml`.  
- `mise exec zig@0.11 -- zig build` — Build project using the specified Zig version.  
- Options: `-v` for verbose download/build messages.

### Elixir / Mix / Ash
- `mise install elixir@1.15 otp@26` — Install Elixir 1.15 with OTP 26 combo.  
- `mise use elixir@1.15` — Activate project with Elixir; updates `mise.toml`.  
- `mise exec elixir@1.15 -- mix deps.get` — Fetch dependencies inside mise environment.  
- `mise reshim elixir` — Refresh escript shims after installation.  
- Options: `--dry-run` to see commands without execution.
- `mise use erlang@28.1.1`

### PostgreSQL
- `mise install postgres@17` — Install PostgreSQL version 17 locally.  
- `mise use postgres@17` — Activate project database environment.  
- `mise exec postgres@17 -- psql -U user dbname` — Connect to database via active tunnel.  
- Options: `-v` verbose install output.

### Python
- `mise install python@3.12` — Install Python 3.12.  
- `mise use python@3.12` — Activate Python 3.12 for project; optionally create venv automatically.  
- `mise exec python@3.12 -- python -m pip install -r requirements.txt` — Run pip commands in project environment.  
- Options: `--venv` to create isolated virtual environment.

### C / Toolchains
- `mise install gcc@12` — Install GCC 12 compiler.  
- `mise use gcc@12 cmake@3.25` — Activate specific compiler and CMake version for project.  
- `mise exec gcc@12 -- gcc main.c -o main` — Compile source using active GCC.  
- Options: `-v` shows detailed compile/link messages.

---

## ⚡ Common Options / Flags

- `-v` — Verbose output, shows full install or command logs.  
- `--dry-run` — Show what the command would do without executing.  
- `--all` — Used with `ls-remote` to show all historical versions.  
- `--venv` — For Python, create a virtual environment automatically.  
- `-n` — Dry-run for destructive commands like `implode`.  

---

## 🔀 Comparison Chart

| Feature | mise | asdf | Nix / Docker |
|---|---:|---:|---:|
| Per-project TOML + tasks | ✔ | ✖ | ✖ |
| Multiple language/tool support | ✔ | ✔ | ✔ |
| Task runner | ✔ | ✖ | ✖ |
| CI-ready | ✔ | ✔ | ✔ |
| Easy one-off command | ✔ | Medium | Low |

---

## 🧮 Related Concepts / Notes

- [[Tool Versioning]]  
- [[Docker]]  
- [[CI-CD]]  
- [[Python]] (venv management)  
- [[Zig]] (toolchains & cross-compilation)  
- [[Elixir]] (Mix tasks, escripts, Ash ops)  
- [[Postgres]] (server & CLI tools)  
- [[Aqua]] (alternative backend)  
- [[asdf]] (legacy compatibility)
- [[Mise]]
- [[Erlang]]

---

## 🔗 External Resources

- Official docs: `https://mise.sh/docs`  
- GitHub plugins & community registry  
- CLI reference: `mise help <command>`  

---

## 📝 Summary

`mise` unifies multi-language version management, environment activation, and task execution under one tool. Its CLI commands allow installing, activating, running, and locking tools with per-project reproducibility. Using options like `-v`, `--dry-run`, and `--venv`, developers can tailor workflow verbosity, safety, and isolation for Python, Zig, Elixir, Ash, PostgreSQL, C, and more.
