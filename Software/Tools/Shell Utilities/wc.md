# wc (Word Count)

**wc** is a Unix command-line utility that counts lines, words, and bytes in files or stdin. Despite its name, line counting (`wc -l`) is its most common use - often piped after other commands to count results or verify file lengths.

---

## ⚙️ Overview

wc reads input and outputs counts. By default it shows all three metrics (lines, words, bytes). Flags select specific counts. Multiple files show per-file counts plus a total.

**Basic syntax:**
```bash
wc [options] [file...]
cat file | wc -l
```

---

## 🔧 Options

| Option | Counts | Example Output |
|--------|--------|----------------|
| `-l` | Lines (newlines) | `42 example.h` |
| `-w` | Words (whitespace-delimited) | `386 example.h` |
| `-c` | Bytes | `4521 example.h` |
| `-m` | Characters (different from bytes for UTF-8) | `4521 example.h` |
| `-L` | Length of longest line (GNU) | `120 example.h` |
| (none) | Lines, words, bytes | `42 386 4521 example.h` |

---

## 📋 Examples

```bash
# Count lines in a file
wc -l example.h

# Count lines after sed deletion (verify edit worked)
sed -i '983,1318d' example.h && wc -l example.h

# Count files in directory
ls | wc -l

# Count matching grep results
grep -r "TODO" src/ | wc -l

# Count words in document
wc -w essay.txt

# Multiple files (shows total)
wc -l *.c

# Longest line length
wc -L example.h
```

---

## 📊 Comparison Chart

| Tool | Purpose | Counts |
|------|---------|--------|
| **wc** | Count lines/words/bytes | Lines, words, bytes, chars |
| [[grep]] `-c` | Count matching lines | Matches only |
| `awk 'END{print NR}'` | Count lines | Lines (more overhead) |
| `stat` | File metadata | Bytes (plus permissions, dates) |

---

## 🌟 Strengths

- Dead simple, does one thing well
- Ubiquitous on all Unix systems
- Fast, even on large files
- Perfect for pipelines

---

## ⚠️ Weaknesses

- `-l` counts newlines, so file without trailing newline is off by one
- Word definition is simplistic (whitespace-delimited)
- No regex or pattern support (use grep -c instead)

---

## 🔗 Related Notes

- [[grep]]
- [[sed]]
- [[awk]]
- [[Bash]]
- [[pipe]]

---

## 📝 Summary

wc counts lines, words, and bytes. Most commonly used as `wc -l` to count lines - either directly on files or piped from other commands to count results. Simple, fast, everywhere.
