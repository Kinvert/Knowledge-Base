# sed (Stream Editor)

**sed** (stream editor) is a Unix command-line utility for parsing and transforming text. It processes input line-by-line, applying commands like substitution, deletion, and insertion. Unlike interactive editors, sed works non-interactively - perfect for automation, pipelines, and batch processing.

---

## ⚙️ Overview

sed reads input (file or stdin), applies editing commands to each line, and outputs the result. It maintains a "pattern space" (current line being processed) and optionally a "hold space" (temporary buffer for advanced operations).

**Basic syntax:**
```bash
sed [options] 'command' file
sed [options] -e 'cmd1' -e 'cmd2' file
cat file | sed 'command'
```

---

## 🧠 Core Concepts

- **Pattern space:** Buffer holding current line being processed
- **Address:** Line number or regex pattern specifying which lines to affect
- **Command:** Action to perform (substitute, delete, print, etc.)
- **Flags:** Modifiers like `g` (global), `i` (case-insensitive), `p` (print)
- **In-place editing:** `-i` flag modifies file directly (use with caution)

---

## 🔧 Common Options

| Option | Description |
|--------|-------------|
| `-i` | Edit file in-place (modifies original file) |
| `-i.bak` | Edit in-place, save backup with .bak extension |
| `-n` | Suppress automatic printing (use with `p` command) |
| `-e` | Add multiple commands |
| `-E` or `-r` | Use extended regex (ERE) instead of basic (BRE) |
| `-f script` | Read commands from file |

---

## 📍 Address Syntax

Addresses specify which lines a command affects:

| Address | Meaning | Example |
|---------|---------|---------|
| `5` | Line 5 only | `sed '5d' example.h` |
| `$` | Last line | `sed '$d' example.h` |
| `1,10` | Lines 1-10 | `sed '1,10d' example.h` |
| `5,$` | Line 5 to end | `sed '5,$d' example.h` |
| `/pattern/` | Lines matching regex | `sed '/TODO/d' example.h` |
| `/start/,/end/` | From pattern to pattern | `sed '/BEGIN/,/END/d' example.h` |
| `1~2` | Every 2nd line starting at 1 (GNU) | `sed '1~2d' example.h` |

**Example you might see Claude use:**
```bash
sed -i '983,1318d' example.h
```
This deletes lines 983 through 1318 from example.h in-place.

---

## 📊 Comparison Chart

| Tool | Type | Interactive | Primary Use | Strengths |
|------|------|-------------|-------------|-----------|
| **sed** | Stream editor | No | Line-by-line transforms | Fast, regex, pipelines |
| [[awk]] | Text processor | No | Field-based operations | Arithmetic, columns |
| [[grep]] | Search tool | No | Pattern matching | Fast search, no editing |
| [[vim]] | Text editor | Yes | Manual editing | Powerful, modal |
| [[Perl]] | Language | No | Complex text processing | More powerful than sed |

---

## 📋 Cheatsheet

### Substitution (s command)

```bash
# Basic substitution (first match per line)
sed 's/old/new/' example.h

# Global substitution (all matches per line)
sed 's/old/new/g' example.h

# Case-insensitive
sed 's/old/new/gi' example.h

# In-place edit (modifies file!)
sed -i 's/old/new/g' example.h

# In-place with backup
sed -i.bak 's/old/new/g' example.h

# Only on lines matching pattern
sed '/pattern/s/old/new/g' example.h

# Only on line 5
sed '5s/old/new/g' example.h

# Different delimiter (useful when / in pattern)
sed 's|/usr/local|/opt|g' example.h
sed 's#old#new#g' example.h
```

### Deletion (d command)

```bash
# Delete specific line
sed '5d' example.h

# Delete line range
sed '983,1318d' example.h

# Delete from line to end
sed '100,$d' example.h

# Delete lines matching pattern
sed '/DEBUG/d' example.h

# Delete empty lines
sed '/^$/d' example.h

# Delete lines containing "TODO"
sed '/TODO/d' example.h

# Delete from pattern to pattern
sed '/START/,/END/d' example.h
```

### Printing (p command)

```bash
# Print only matching lines (use -n to suppress default output)
sed -n '/pattern/p' example.h

# Print line range
sed -n '10,20p' example.h

# Print first 10 lines (like head)
sed -n '1,10p' example.h

# Print last line
sed -n '$p' example.h
```

### Insertion & Appending

```bash
# Insert before line 5
sed '5i\New line here' example.h

# Append after line 5
sed '5a\New line here' example.h

# Insert before pattern match
sed '/pattern/i\Inserted line' example.h

# Add text to beginning of each line
sed 's/^/PREFIX: /' example.h

# Add text to end of each line
sed 's/$/ SUFFIX/' example.h
```

### Multiple Commands

```bash
# Semicolon separated
sed 's/foo/bar/; s/baz/qux/' example.h

# Multiple -e flags
sed -e 's/foo/bar/' -e 's/baz/qux/' example.h

# From script file
sed -f commands.sed example.h
```

### Trimming & Cleanup

```bash
# Remove leading whitespace
sed 's/^[ \t]*//' example.h

# Remove trailing whitespace
sed 's/[ \t]*$//' example.h

# Remove all whitespace
sed 's/[ \t]//g' example.h

# Remove blank lines
sed '/^$/d' example.h

# Remove C++ comments (// style)
sed 's|//.*||' example.h

# Remove shell comments (# style)
sed 's/#.*//' example.h
```

### Capture Groups

```bash
# Swap two fields (basic regex uses \( \))
sed 's/\(.*\):\(.*\)/\2:\1/' example.h

# With extended regex (-E), cleaner syntax
sed -E 's/(.*):(.*):\2:\1/' example.h

# Reference whole match with &
sed 's/[0-9]*/(&)/' example.h
```

---

## ⚠️ Gotchas

- **In-place on macOS:** `sed -i '' 's/old/new/' file` (empty string required)
- **Newlines:** sed processes one line at a time; multi-line matches need special handling
- **Escaping:** Special chars `.*[]^$\` need escaping in patterns
- **No undo:** `-i` permanently modifies files - always test first without `-i`

---

## 🌟 Strengths

- Ubiquitous on Unix/Linux systems
- Extremely fast for large files
- Perfect for pipelines and automation
- Powerful regex support

---

## ⚠️ Weaknesses

- Cryptic syntax for beginners
- Multi-line operations are awkward
- Basic regex (BRE) by default, quirky escaping
- No built-in arithmetic (use [[awk]] instead)

---

## 🔗 Related Notes

- [[awk]]
- [[grep]]
- [[Bash]]
- [[Regex]]
- [[vim]]

---

## 🌐 External Resources

- [GNU sed Manual](https://www.gnu.org/software/sed/manual/sed.html)
- [sed One-Liners](https://www.pement.org/sed/sed1line.txt)
- [QuickRef Cheatsheet](https://quickref.me/sed.html)
- [Grymoire sed Tutorial](https://www.grymoire.com/Unix/Sed.html)

---

## 📝 Summary

sed is a stream editor for non-interactive text transformation. It excels at search-and-replace, line deletion, and text manipulation in pipelines. The command `sed -i '983,1318d' example.h` deletes lines 983-1318 in-place. While cryptic at first, sed's address + command syntax becomes powerful once learned. For field-based operations use [[awk]]; for interactive editing use [[vim]].
