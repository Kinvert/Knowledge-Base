# tee

**tee** reads from standard input and writes to both standard output AND one or more files simultaneously. Named after the T-shaped pipe fitting in plumbing — data flows in one end and splits two ways. Its most common use on Linux is the `sudo tee` pattern for writing to root-owned files, since shell redirects (`>`) don't inherit `sudo` privileges.

---

## ⚙️ Overview

```bash
command | tee [options] file [file2 ...]
```

- Reads stdin, writes to stdout AND the specified file(s)
- By default **overwrites** the file — use `-a` to **append**
- Accepts multiple file arguments — writes to all of them
- Commonly piped after other commands to capture output while still seeing it
- Part of GNU coreutils — available on every Linux system

---

## 🔬 Command Breakdown — The `sudo tee` Pattern

This command from the [[sysctl]] workflow demonstrates the most important tee pattern:

```bash
echo fs.inotify.max_user_watches=524288 | sudo tee -a /etc/sysctl.conf
```

### Why not `sudo echo ... > /etc/sysctl.conf`?

This is a critical concept. Let's compare:

```bash
# FAILS with "Permission denied"
sudo echo fs.inotify.max_user_watches=524288 > /etc/sysctl.conf
```

**Why it fails:** The shell processes the redirect (`>`) *before* executing the command. The redirect runs as your user, not as root. `sudo` only elevates `echo`, but `echo` doesn't need root — the file write does. By the time `echo` runs with root privileges, the shell has already failed to open the file.

```bash
# WORKS
echo fs.inotify.max_user_watches=524288 | sudo tee -a /etc/sysctl.conf
```

**Why it works:** `tee` is the command doing the file write, and `tee` runs as root (via `sudo`). The [[pipe]] (`|`) connects `echo`'s stdout to `tee`'s stdin, and `tee` opens and writes to the file with root privileges.

### Part-by-part

**`echo fs.inotify.max_user_watches=524288`** — Produces the config line on stdout. Runs as your user (doesn't need root).

**`|`** — [[pipe]]. Sends echo's stdout into tee's stdin.

**`sudo tee`** — tee runs as root. Reads from stdin, writes to the file AND stdout.

**`-a`** — Append mode. Without this, tee would **overwrite** `/etc/sysctl.conf`, destroying all existing settings.

**`/etc/sysctl.conf`** — The target file. Owned by root, so tee needs `sudo`.

### Suppressing the echo

tee also writes to stdout, so you'll see the line echoed back in your terminal. To suppress this:

```bash
echo fs.inotify.max_user_watches=524288 | sudo tee -a /etc/sysctl.conf > /dev/null
```

---

## 🚩 Options

| Option | Description | Example |
|--------|-------------|---------|
| `-a` | Append to file instead of overwriting | `echo "line" \| tee -a log.txt` |
| `-i` | Ignore SIGINT (interrupt signals) | `long_cmd \| tee -i output.log` |
| `-p` | Diagnose write errors to non-pipes | `cmd \| tee -p file.txt` (GNU coreutils 8.25+) |
| *(multiple files)* | Write to several files at once | `cmd \| tee file1.txt file2.txt file3.txt` |

---

## 💡 Common Patterns

### Write to root-owned file (the classic)

```bash
echo "nameserver 8.8.8.8" | sudo tee /etc/resolv.conf
echo "fs.inotify.max_user_watches=524288" | sudo tee -a /etc/sysctl.conf
```

### Log a pipeline while still seeing output

```bash
make 2>&1 | tee build.log
# Build output visible in terminal AND saved to build.log
```

### Write to multiple files simultaneously

```bash
echo "config line" | tee config_backup.txt config_live.txt
```

### Suppress stdout (silent write)

```bash
echo "secret" | sudo tee /etc/myconfig > /dev/null
```

### Capture intermediate pipeline results

```bash
cat data.csv | tee raw_backup.csv | sort | tee sorted.csv | head -20
# raw_backup.csv = unsorted, sorted.csv = sorted, terminal shows first 20
```

### Process substitution (write to commands instead of files)

```bash
echo "hello" | tee >(gzip > compressed.gz) >(wc -c > size.txt) > /dev/null
# Sends "hello" to gzip AND wc simultaneously
```

### Append timestamps to a log

```bash
some_service 2>&1 | while read line; do echo "$(date '+%H:%M:%S') $line"; done | tee -a service.log
```

---

## 📊 Comparison Chart

| Tool / Method | Writes to File | Shows on Screen | Appends | Root Write | Multiple Files |
|---------------|---------------|-----------------|---------|------------|----------------|
| **tee** | Yes | Yes (stdout) | `-a` flag | `sudo tee` | Yes |
| `>` (redirect) | Yes | No | No (overwrites) | Fails with sudo | No |
| `>>` (append redirect) | Yes | No | Yes | Fails with sudo | No |
| **script** | Yes (typescript) | Yes | `-a` flag | N/A | No |
| `cat > file` | Yes (from stdin) | No | No | N/A | No |
| **sponge** (moreutils) | Yes | No | No | N/A | No (but reads all first) |

The key differentiator: tee is the only standard tool that writes to a file AND passes through to stdout in a pipeline.

---

## 🌟 Strengths

- Solves the `sudo` redirect problem cleanly
- Pipeline-friendly — fits naturally into Unix command chains
- Can write to multiple files at once
- Available everywhere — part of GNU coreutils
- Simple, single-purpose tool (Unix philosophy)

---

## ⚠️ Weaknesses

- Overwrites by default — forgetting `-a` can destroy file contents
- No built-in file locking — concurrent writes can interleave
- Buffering can delay output in long pipelines (mitigate with `stdbuf -oL`)
- Prints to stdout even when you don't want it (redirect to `/dev/null`)
- No timestamp or formatting — just raw passthrough

---

## 🔗 Related Notes

- [[pipe]]
- [[sysctl]]
- [[Bash]]
- [[sed]]
- [[Linux]]

---

## 🌐 External Resources

- Linux man page: `https://man7.org/linux/man-pages/man1/tee.1.html`
- GNU coreutils docs: `https://www.gnu.org/software/coreutils/manual/html_node/tee-invocation.html`

---

## 📝 Summary

tee splits a data stream — writing to a file while passing through to stdout. Its killer use case is `sudo tee` for writing to root-owned files, since shell redirects (`>`) don't inherit `sudo` privileges. The [[sysctl]] example (`echo ... | sudo tee -a /etc/sysctl.conf`) is the canonical case: append a kernel parameter to a root-owned config file without overwriting it.
