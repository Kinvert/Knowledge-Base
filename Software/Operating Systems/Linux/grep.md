# grep

`grep` is a command-line utility used to search for patterns in text. It reads input from files, standard input, or streams and outputs lines that match the given pattern. For robotics engineers and developers, `grep` is an indispensable tool for filtering logs, debugging outputs, parsing configuration files, and chaining with other Unix utilities for automation.

---

## ⚙️ Overview

Originally named for the ed command `g/re/p` ("global / regular expression / print"), `grep` is one of the most widely used text-processing tools on Linux. It supports literal string search as well as powerful pattern matching through regular expressions. Variants like `egrep` and `fgrep` are now deprecated in favor of options (`-E` and `-F`).

**What it does best:**
- Quickly filter log files for keywords.
- Highlight matches in real time.
- Parse structured text and configuration files.
- Combine with pipelines for selective outputs.
- Debug regex patterns interactively.

---

## 🧠 Core Concepts

- **Pattern**: A string or regular expression to match.
- **Regular Expressions (Regex)**: Allow complex pattern definitions.
- **Input Source**: File(s) or standard input.
- **Match Modes**: Basic regex, extended regex, fixed strings, inverted matches.
- **Output Control**: Context lines, counts, filenames, line numbers.

---

## 📊 Comparison Chart

| Tool / Feature       | [[grep]] | [[awk]] | [[sed]] | [[cut]] | [[rg]] (ripgrep) | [[ag]] (The Silver Searcher) |
|----------------------|----------|---------|---------|---------|------------------|------------------------------|
| **Regex Support**    | ✅       | ✅      | ✅      | ❌      | ✅                | ✅                           |
| **Field Processing** | ❌       | ✅      | ❌      | ✅      | ❌                | ❌                           |
| **Search Speed**     | Medium   | Medium  | Medium  | Fast    | Very Fast        | Very Fast                    |
| **Recursive Search** | ✅ (`-r`) | Limited | ❌      | ❌      | ✅                | ✅                           |
| **Streaming Input**  | ✅       | ✅      | ✅      | ✅      | ✅                | ✅                           |
| **Regex Flavor**     | BRE/ERE  | ERE     | BRE     | N/A     | PCRE2-compatible | PCRE2-compatible             |

---

## 🛠 Use Cases

- Searching logs for errors or warnings: `grep "ERROR" robot.log`
- Filtering dmesg output for USB devices: `dmesg | grep usb`
- Extracting parameter values from config files.
- Chaining in pipelines to narrow down large datasets.
- Checking ROS node output for specific topics or messages.

---

## ✅ Strengths

- Extremely fast for plain-text searches.
- Universally available on Unix-like systems.
- Supports both simple and advanced regex.
- Flexible output options (counts, filenames, highlighting).
- Easy to combine with pipes and redirects.

---

## ❌ Weaknesses

- Regex syntax can be intimidating for beginners.
- Limited to line-based searches (no multi-line regex).
- Slower than modern alternatives (`ripgrep`, `ag`) for huge codebases.
- No built-in field extraction compared to `awk`.

---

## 🧪 Cheat Sheet (One-Liners)

- Basic search in file: `grep "pattern" file.txt`
- Case-insensitive search: `grep -i "pattern" file.txt`
- Recursive search in directory: `grep -r "pattern" ./src`
- Show line numbers: `grep -n "pattern" file.txt`
- Show only filenames with matches: `grep -l "pattern" *.log`
- Show only non-matching files: `grep -L "pattern" *.log`
- Count matches: `grep -c "pattern" file.txt`
- Show context (3 lines before/after): `grep -C 3 "pattern" file.txt`
- Show only matching text (not the whole line): `grep -o "pattern" file.txt`
- Highlight matches (default on many distros): `grep --color=auto "pattern"`
- Match whole words only: `grep -w "word" file.txt`
- Use extended regex: `grep -E "foo|bar" file.txt`
- Fixed-string search (faster, no regex): `grep -F "literal.string" file.txt`
- Invert match (exclude): `grep -v "DEBUG" file.txt`
- Multiple patterns: `grep -e "foo" -e "bar" file.txt`
- Read patterns from file: `grep -f patterns.txt input.log`
- Search compressed files: `zgrep "pattern" file.gz`

---

## 🔍 Regex Highlights for grep

- `.` → Any single character  
- `^pattern` → Match beginning of line  
- `pattern$` → Match end of line  
- `.*` → Any sequence of characters  
- `[abc]` → Match one of a set  
- `[^abc]` → Match anything but a set  
- `pattern1\|pattern2` → OR (use `-E`)  
- `pattern\{2,4\}` → Repeat count (use `-E`)  

---

## 🔧 Advanced Tricks

- Find lines not containing a keyword: `grep -v "ERROR" file.log`
- Show function definitions in C files: `grep -n "^[a-zA-Z_].*(.*)" *.c`
- Extract IP addresses: `grep -Eo "([0-9]{1,3}\.){3}[0-9]{1,3}" server.log`
- Filter `ps` for a process: `ps aux | grep "[r]oscore"`
- Track log in real time: `tail -f robot.log | grep --color=auto "WARN"`

---

## 📚 Related Concepts

- [[awk]]
- [[sed]]
- [[cut]]
- [[rg]]
- [[ag]]
- [[Regex]]
- [[Linux]]

---

## 🌐 External Resources

- man page: `man grep`
- GNU grep docs: [https://www.gnu.org/software/grep/manual/grep.html](https://www.gnu.org/software/grep/manual/grep.html)
- Regex101 (for testing patterns): [https://regex101.com](https://regex101.com)
