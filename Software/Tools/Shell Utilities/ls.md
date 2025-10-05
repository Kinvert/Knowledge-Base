# ls — List directory contents (Comprehensive Cheatsheet)

`ls` is the classic UNIX/Linux command for listing directory contents. This note is a deep, practical cheatsheet focused on real-world one-liners, pipelines, and portability tips that engineers (especially robotics / embedded folks) actually use. Wherever `ls` can't do something reliably (searching by date, recursive ranking, folder-size summaries, content searches) you'll see the recommended tool (`find`, `du`, `stat`, `grep`) and exactly how to combine them.

---

## 📖 Overview

`ls` shows filenames and (with flags) metadata such as permissions, owners, sizes, and timestamps. It's fast and ubiquitous, but `ls` is *not* a replacement for `find`, `du`, or other tools when you need filtering, recursion, or robust scripting. Use `ls` for quick human-friendly views; use `find`/`du`/`stat` for programmatic searches.

---

## 🧠 Core concepts & gotchas

- `ls` shows *names* and formats output for humans. It does not filter by time or size (use `find` for that).
- Common sort keys: `-t` (modification time), `-S` (size), `-X` (extension), `-v` (natural/version).
- `-r` reverses sort order. Combine with `-t` or `-S` for newest/oldest or largest/smallest.
- Hidden files begin with `.`; `-a` shows them, `-A` shows them but not `.` and `..`.
- `ls` output is meant for humans; **don't parse `ls` output in scripts**. Use `find -print0` / `xargs -0` or null-separated output for scripting.
- GNU vs BSD differences: things like `--group-directories-first`, `-printf` (find), `stat -c` vs `stat -f`, and `--time-style` are GNU-specific. I call out portable alternatives where needed.

---

## 🧾 Common flags quick reference

| Flag | Meaning |
|---|---|
| `-a` | show all files including `.` and `..` |
| `-A` | show all except `.` and `..` |
| `-l` | long format (perms, links, owner, size, date, name) |
| `-h` | human-readable sizes for `-l` (e.g. `1.2K`) |
| `-t` | sort by mtime (newest first) |
| `-r` | reverse sort order |
| `-S` | sort by file size (largest first) |
| `-X` | sort alphabetically by extension |
| `-F` | append indicator (`/` dir, `*` exec, `@` link) |
| `-i` | show inode number |
| `-d` | list directories themselves, not their contents |
| `-R` | recursive (walks directories) |
| `-1` | one entry per line (useful in scripts/humans) |
| `--color=auto` | colorize output (GNU) |
| `--group-directories-first` | show directories before files (GNU) |

---

## 🧰 Practical one-liners (organized by task)

> Notes: All commands are single-line `inline code` examples. If a command uses `find -printf` or `stat -c` it's GNU-prefixed; a portable alternative is provided below when useful.

### Basics & human-friendly listing
- `ls` — default, columns (human view).
- `ls -1` — one filename per line (good for piping).
- `ls -la` — long listing, all files including dotfiles.
- `ls -lah` — long listing with human-readable sizes (KB/MB).
- `ls -l --color=auto` — long listing with color (if your `ls` supports it).
- `ls -F` — classify entries (adds `/`, `*`, `@`).

### Sorting & top/oldest
- `ls -lt | head -n 10` — **10 newest files** in current directory (sorted newest→oldest).
- `ls -ltr | head -n 10` — **10 oldest files** in current directory (oldest→newest thanks to `-r`).
- `ls -lS | head -n 10` — **10 largest files** in current directory.
- `ls -lSr | head -n 10` — **10 smallest files** (reverse of `-S`).

### Extension / pattern filtering (current directory)
- `ls *.pt` — list `.pt` files in current directory (globbing).
- `ls -lt *.pt | head -n 1` — **newest `.pt` file** in current directory.

### Search-by-name recursively (use `find`)
- `find . -type f -name '*.pt'` — recursive search for `.pt` files.
- `find . -type f -name '*1759*.pt'` — **all `.pt` files whose filename contains `1759`**.

### Newest/oldest across an entire tree (recursive)
- `find . -type f -printf '%T@ %p\n' | sort -nr | head -n 10 | cut -d' ' -f2-`  
  → **10 newest files in tree** (GNU `find` with `-printf`). `%T@` prints epoch mtime.
- Portable alternative (if `-printf` not available):  
  `find . -type f -exec stat -f '%m %N' {} + | sort -nr | head -n10 | cut -d' ' -f2-` (BSD `stat`)  
  or use `stat -c` on GNU.

- `find . -type f -printf '%T@ %p\n' | sort -n | head -n 10 | cut -d' ' -f2-` — **10 oldest files** (recursive).

### Find files modified on a specific date (e.g., March 7, 2025)
- GNU `find` quick: `find . -type f -newermt '2025-03-07' ! -newermt '2025-03-08'`  
  → files modified on 2025-03-07 (midnight-to-midnight).
- Portable touch-based method (POSIX-friendly):
  - `touch -t 202503070000 /tmp/start && touch -t 202503080000 /tmp/end`
  - `find . -type f -newer /tmp/start ! -newer /tmp/end`
  - Remove the temp files afterwards.
- More precise time range: `find . -newermt '2025-03-07 08:00' ! -newermt '2025-03-07 12:00'`

### Files modified within last N days / minutes
- `find . -type f -mtime -7` — modified within last 7 days.
- `find . -type f -mmin -60` — modified within last 60 minutes.
- `find . -type f -mtime -7 -exec ls -lh {} +` — show long listings for those files.

### Content search inside files (filename vs content)
- Filename contains `1759`: `find . -type f -name '*1759*.pt'`
- Content contains `1759` (text files only): `grep -R --line-number -F '1759' --include='*.pt' .`  
  → Note: `.pt` model files are usually binary; `grep` may not be meaningful. For binary, use `strings file.pt | grep 1759`.

### Which folder has the most GB (top directories)
- `du -sh * | sort -h | tail -n 5` — sizes of immediate children, largest last (GNU & common).  
- `du -sh ./* | sort -h | tail -n 10` — similar, explicit glob.
- `du -ah . | sort -hr | head -n 20` — top 20 biggest files and directories recursively (GNU `du`).
- macOS/BSD variant: `du -d 1 -h | sort -hr | head -n 20` or `du -h -d 1` (flags differ across BSD).
- Interactive alternative: `ncdu .` (great for exploring big folders).

### Combining `ls` with other tools safely (for scripts / spaces in names)
- Avoid `ls` for parsing. Use null separators:
  - `find . -type f -print0 | xargs -0 -I{} ls -ld -- "{}"`  
  - Or list basenames: `find . -maxdepth 1 -type f -print0 | xargs -0 -n1 basename`

### Quick permission / owner filters
- Files owned by user `alice`: `find . -type f -user alice -ls`
- Files writable by group: `find . -type f -perm /g=w -ls`

### Nice `ls`-style alternatives / enhancements
- `exa` — modern `ls` replacement: `exa -la --git` (shows git status, colors).
- `fd` — faster alternative to `find` for name-based searches (`fd '*.pt'`).
- `bat` — `cat` replacement with syntax highlighting (useful when inspecting text files found by `ls`/`find`).

---

## 🔧 Answered example questions (explicit)

- **Find the 10 newest files (recursive)**:  
  `find . -type f -printf '%T@ %p\n' | sort -nr | head -n 10 | cut -d' ' -f2-`  
  *(GNU `find`)*

- **Find the 10 oldest files (recursive)**:  
  `find . -type f -printf '%T@ %p\n' | sort -n | head -n 10 | cut -d' ' -f2-`

- **Newest `.pt` file in current directory**:  
  `ls -t *.pt | head -n 1`

- **Newest `.pt` file in an entire project tree (recursive)**:  
  `find . -type f -name '*.pt' -printf '%T@ %p\n' | sort -nr | head -n1 | cut -d' ' -f2-`

- **All `.pt` files whose filename contains `1759` (recursive)**:  
  `find . -type f -name '*1759*.pt'`

- **All `.pt` files whose *contents* contain `1759`**:  
  `grep -aR --line-number -F '1759' --include='*.pt' .`  
  *(Note: `.pt` may be binary; use `strings` if necessary: `find . -name '*.pt' -exec sh -c 'strings "$1" | grep -q "1759" && echo "$1"' _ {} \;` )*

- **Which folder has the most GB of stuff (top-level children)**:  
  `du -sh * | sort -h | tail -n 1` — shows largest child.  
  For a top-10: `du -sh * | sort -h | tail -n 10`

- **Find all files modified on March 7, 2025**:  
  `find . -type f -newermt '2025-03-07' ! -newermt '2025-03-08'`  
  *(portable alternative using `touch -t` shown above)*

---

## ⚠️ Portability notes & gotchas

- `find -printf` and `--time-style` are GNU extensions; on macOS/BSD, use `stat -f` or the `touch`-based approach.
- `ls` does not support filtering by time or size in a robust way — use `find` and `du`.
- `ls` output is locale- and color-dependent; do not parse it in scripts.
- Binary `.pt` files: `grep` may flag them as binary. Use `strings` or specialized tools to inspect.

---

## 🔁 When to use `ls` vs `find` vs `du` (short guidance)

- Use `ls` when you want a **quick human listing** of a directory (single level).  
- Use `find` when you need **recursive filtering** (by name, mtime, size, owner) or to run actions.  
- Use `du` when you care about **disk usage / directory sizes**.  
- Use `stat` when you need **precise timestamps** or file metadata in scripts.  
- Use `exa` / `fd` for **nicer UX / performance** if available.

---

## 📊 Comparison chart — commands similar to `ls`

| Tool | Primary purpose | Recursive? | Filter by date/size? | Human-friendly output | When to prefer |
|---|---:|:---:|:---:|:---:|---|
| `ls` | Show directory contents | Limited (`-R`) | No (use `find`) | Yes (default) | Quick human checks |
| `find` | Search & act on files | Yes (core feature) | Yes (mtime, mmin, size) | Minimal; script-friendly | Complex queries + scripting |
| `du` | Disk usage | Yes | Can show size buckets | Good with `-h` | Find large folders/files |
| `stat` | Detailed metadata | Per-file | Yes (timestamps) | Numeric; script-friendly | Precise metadata for scripts |
| `tree` | Visual tree view | Yes | No filters built-in | Yes (tree ASCII) | Visual hierarchical view |
| `exa` | Modern `ls` replacement | Yes (`-R`) | Limited | Very nice (git + colors) | Interactive UX improvements |
| `fd` | Fast name-based search | Yes | No advanced mtime filters | Cleaner than `find` | Fast name searches; pair with `xargs` |

---

## ✅ Best practices (scripts / automation)

- **Do not parse `ls`**. Use `find -print0` and `xargs -0` or `while IFS= read -r -d '' file; do …; done < <(find . -print0)`.
- Use epoch timestamps (`%T@`) for numeric sorts and stable behavior.
- Prefer `du --apparent-size` vs `du` differences depending on whether you want allocated blocks vs apparent size (advanced).
- For reproducible tools across systems (Linux, macOS), include checks or fallbacks (e.g., check if `gfind` exists, or use `touch`-based date ranges).

---

## 🔗 Related Concepts / Notes

- [[find]] (powerful recursive file search)
- [[du]] (disk usage)
- [[stat]] (file metadata)
- [[grep]] (content search)
- [[tree]] (visual directory tree)
- [[exa]] (modern ls replacement)
- [[fd]] (fast search)
- [[ncdu]] (interactive disk usage)
- [[bash]] (shell idioms & safe looping)

---

## 🧾 Summary / TL;DR

- `ls` = quick human view. Great for glancing at a directory.
- For anything recursive, date- or size-based, or script-safe, combine `find`, `du`, and `stat` and show results with `sort`/`head`/`cut`.
- Use null-delimited outputs for safety with filenames that contain whitespace/newlines.
- Use `du`/`ncdu` to find where disk space is used.
- For cross-platform scripts, prefer portable `touch`-based date ranges and check `stat`/`find` flavor differences.

---

## ✨ Further reading & tools to try
- `man ls`, `man find`, `man du`, `man stat`
- Try `exa`, `fd`, `bat`, and `ncdu` for modern UX
- Consider small helper scripts in `~/bin` for repetitive searches (e.g., `topfiles`, `topdirs`) — keep them script-safe.
- https://explainshell.com/explain/1/ls

