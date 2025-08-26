# sed (Stream Editor)

`sed` is a command-line utility for parsing and transforming text in a stream or file. It is most commonly used for search-and-replace operations, but it can also insert, delete, and manipulate text in highly efficient ways. For robotics engineers, `sed` is useful in automation pipelines, configuration file updates, and log preprocessing.

---

## ⚙️ Overview

`sed` processes text line-by-line and applies transformations based on scripts or one-liner commands. It excels at quick edits, batch substitutions, and automating repetitive text manipulations without opening a file in an editor.

---

## 🧠 Core Concepts

- **Stream editing:** Operates on input as a stream (stdin or file) without interactive editing.  
- **Patterns & regex:** Uses regular expressions to match text.  
- **Commands:** Includes substitution (`s`), deletion (`d`), printing (`p`), and insertion (`i`).  
- **Non-interactive:** Unlike editors (`vim`, `nano`), `sed` works in pipelines or scripts.  

---

## 📊 Comparison Chart

| Tool     | Type              | Interactive? | Primary Use | Strengths | Weaknesses |
|----------|-------------------|--------------|-------------|-----------|-------------|
| `sed`    | Stream editor     | No           | Batch text editing | Fast, regex support, scripting | Non-interactive, limited context editing |
| `awk`    | Text processing   | No           | Field-based operations | Arithmetic, formatting, reports | More complex syntax than `sed` |
| `grep`   | Search tool       | No           | Pattern matching | Fast search, regex support | No editing capabilities |
| `vi/vim` | Text editor       | Yes          | Manual editing | Powerful, modal editing, scripting | Steeper learning curve |
| `nano`   | Text editor       | Yes          | Simple editing | Easy to use | Limited features |
| `perl`   | Language          | No/Yes       | Advanced text processing | More powerful than `sed/awk` | Heavier runtime |
| `python` | Language          | No/Yes       | General-purpose scripting | Huge ecosystem, readability | Slower startup, more verbose |

---

## 🛠️ Common Use Cases

- Batch replacing configuration values in robotics log files.  
- Cleaning CSV/JSON sensor outputs before feeding into ROS. 
- Automating version string updates in source code.  
- Removing or filtering lines in datasets for ML preprocessing.  
- Inline editing inside CI/CD pipelines.  

---

## 📋 Cheatsheet (One-Liners)

- `sed 's/foo/bar/' file` → Replace first occurrence of `foo` with `bar` per line  
- `sed 's/foo/bar/g' file` → Replace all occurrences of `foo` with `bar`  
- `sed -i 's/foo/bar/g' file` → Replace all occurrences in-place  
- `sed '/pattern/d' file` → Delete lines matching `pattern`  
- `sed -n '/pattern/p' file` → Print only lines matching `pattern`  
- `sed '1,5d' file` → Delete lines 1 through 5  
- `sed -n '5,10p' file` → Print lines 5 through 10  
- `sed 's/[0-9]//g' file` → Remove all digits  
- `sed 's/\(.*\):\(.*\)/\2:\1/' file` → Swap fields around `:`  
- `sed 's/^/prefix_/' file` → Add prefix to each line  
- `sed 's/$/_suffix/' file` → Add suffix to each line  
- `sed 's/^[ \t]*//' file` → Trim leading spaces/tabs  
- `sed 's/[ \t]*$//' file` → Trim trailing spaces/tabs  
- `sed 's/\bword\b/NEWWORD/g' file` → Replace whole word only  
- `sed -i 's/old/new/; s/foo/bar/' file` → Multiple replacements at once  
- `sed -e 's/foo/bar/g' -e 's/baz/qux/g' file` → Chain replacements  
- `echo "Hello" | sed 'a\World'` → Append text after line  

---

## ✅ Strengths

- Lightweight and ubiquitous on Linux systems.  
- Perfect for inline replacements in automation.  
- Works in pipelines with other tools like [[grep]] and [[awk]].  
- Supports complex regex for advanced transformations.  

---

## ❌ Weaknesses

- Limited interactivity (not a text editor).  
- Syntax can be cryptic for beginners.  
- Complex multi-line manipulations are tricky compared to scripting languages.  

---

## 📚 Related Concepts

- [[awk]] (field-based text processing)  
- [[grep]] (pattern searching)  
- [[Perl]] (text manipulation language)  
- [[Bash]] (shell scripting)  
- [[Regex]] (regular expressions)  

---

## 🌍 External Resources

- GNU sed Manual: https://www.gnu.org/software/sed/manual/sed.html  
- One-Liner Sed Examples: https://www.pement.org/sed/sed1line.txt  
- "Sed & Awk" (book by Dale Dougherty, Arnold Robbins)  

---

## 🏁 Summary

`sed` is an indispensable tool in the Linux command-line ecosystem for robotics engineers and developers who deal with large datasets, logs, or configuration files. It is not an editor like `vim` or `nano`, but a stream-based editor that shines in automation and pipelines. For tasks requiring interactive or multi-line editing, editors or scripting languages are often better choices.
