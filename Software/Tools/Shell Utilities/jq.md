# jq

**jq** is a lightweight, command-line JSON processor — often described as "[[sed]] for [[JSON]]." It lets you slice, filter, reshape, and transform structured data using a concise expression language. Commonly piped from [[curl]], [[grep]], or API responses, jq is the standard Unix tool for working with JSON on the command line.

---

## ⚙️ Overview

```bash
jq [options] 'filter' [file...]
echo '{"a":1}' | jq '.a'
```

- Reads JSON from files or stdin, applies a filter expression, outputs the result
- Outputs valid JSON by default (quoted strings, colored in terminals)
- Use `-r` for raw (unquoted) string output
- Zero dependencies — single static C binary
- Streams one JSON value at a time (memory-efficient for arrays)

---

## 🔬 Command Breakdown — Real Example

This real command extracts a reward scale and reason from each suggestion in a [[Weights and Biases]] sweep override file:

```bash
jq '.suggestions | .[] | "\(.params["env/reward_aim_scale"]): \(.reason)"' df34_override.json
```

Output:

```
"0.0027700277: S2 clone of S1 lunar-valley-124 (AR=1270)"
"0.0025399492: S2 clone of S1 noble-sea-118 (AR=1231)"
"0.002900087: S2 clone of S1 lemon-wildflower-131 (AR=1193)"
```

### Concept-by-concept breakdown

**1. Single quotes around the filter** — `'...'`
Shell quoting prevents Bash from interpreting `$`, `()`, `"`, etc. Always single-quote jq filters.

**2. `.suggestions`** — dot notation field access
Extracts the `suggestions` key from the root object. Equivalent to `obj["suggestions"]` in [[Python]].

**3. `|`** — jq pipe operator
Passes the output of the left expression as input to the right. Works like Unix [[pipe]] but inside jq's expression language.

**4. `.[]`** — array iterator
Explodes an array into individual elements. Each element flows through the rest of the pipeline independently. Without the brackets (`.[]` not `.[0]`), it iterates *all* elements.

**5. `"\(...)"` — string interpolation**
Builds a string with embedded expressions. `\(expr)` evaluates `expr` and inserts the result. Similar to Python f-strings or shell `$()`.

**6. `.params["env/reward_aim_scale"]`** — bracket notation
Required when a key contains special characters like `/`. Dot notation (`.params.env/reward_aim_scale`) would fail because `/` isn't valid in an identifier. Bracket notation works for any key.

**7. `.reason`** — simple field access
Standard dot notation for a clean key name.

**8. `: `** — literal text in interpolated string
Any text outside `\(...)` in the string is literal. Here it formats output as `value: reason`.

**9. `df34_override.json`** — file path argument
jq reads from a file when provided, or stdin when piped. Both work identically.

### The `-r` flag difference

```bash
jq -r '.suggestions | .[] | "\(.params["env/reward_aim_scale"]): \(.reason)"' df34_override.json
```

```
0.0027700277: S2 clone of S1 lunar-valley-124 (AR=1270)
0.0025399492: S2 clone of S1 noble-sea-118 (AR=1231)
```

Without `-r`: output has surrounding `"quotes"`. With `-r` (raw): plain text, ready for piping to other tools.

---

## 🧱 Basic Filters

| Filter | Description | Example |
|--------|-------------|---------|
| `.` | Identity — pass input through | `jq '.' file.json` (pretty-print) |
| `.foo` | Object field access | `jq '.name'` → `"alice"` |
| `.["foo"]` | Bracket field access (any key) | `jq '.["my-key"]'` |
| `.[0]` | Array index (0-based) | `jq '.[0]'` → first element |
| `.[-1]` | Negative index | `jq '.[-1]'` → last element |
| `.[2:5]` | Array slice | `jq '.[2:5]'` → elements 2,3,4 |
| `.[]` | Iterate all elements | `jq '.[]'` → one output per element |
| `,` | Output multiple values | `jq '.name, .age'` |
| `\|` | Pipe (chain filters) | `jq '.items \| .[0]'` |
| `.foo?` | Suppress errors if missing | `jq '.foo?'` → no error if null |

---

## 🚩 Common CLI Flags

| Flag | Purpose | Example |
|------|---------|---------|
| `-r` | Raw string output (no quotes) | `jq -r '.name'` → `alice` |
| `-c` | Compact output (one line) | `jq -c '.'` → minify JSON |
| `-n` | Null input (don't read stdin) | `jq -n '{a: 1}'` → create JSON |
| `-s` | Slurp — read all inputs into array | `jq -s '.' *.json` |
| `-e` | Set exit code based on output | `jq -e '.ok'` → exits 1 if false/null |
| `-S` | Sort object keys | `jq -S '.' file.json` |
| `--arg k v` | Bind string variable `$k` | `jq --arg name "alice" '.[$name]'` |
| `--argjson k v` | Bind JSON variable `$k` | `jq --argjson n 42 '. + $n'` |
| `--slurpfile k f` | Bind file contents to `$k` | `jq --slurpfile d data.json '.'` |
| `--rawfile k f` | Bind raw file string to `$k` | `jq --rawfile t template.txt '.'` |
| `--jsonargs` | Treat remaining args as JSON | `jq -n '$ARGS.positional' --jsonargs 1 2 3` |

---

## 🔧 Common Functions

### Selection & Testing

```bash
# Select elements matching a condition
jq '.[] | select(.age > 30)' users.json

# Check if key exists
jq '.[] | select(has("email"))' users.json

# Filter by type
jq '.[] | select(type == "string")' mixed.json
```

### Transform

```bash
# Map over array
jq '[.[] | .name]' users.json        # or: jq 'map(.name)'

# Object to key-value pairs and back
jq 'to_entries' obj.json              # [{key, value}, ...]
jq 'from_entries' pairs.json          # {key: value, ...}

# Get keys or values
jq 'keys' obj.json                    # ["a", "b", "c"]
jq 'values' obj.json                  # [1, 2, 3]

# Flatten nested arrays
jq 'flatten' nested.json              # flatten(1) for one level

# Unique values
jq 'unique' array.json
jq 'unique_by(.name)' users.json
```

### Aggregation

```bash
jq 'length' array.json               # array length or string length
jq 'add' numbers.json                # sum an array of numbers
jq 'min, max' numbers.json           # min and max values
jq 'group_by(.dept)' employees.json  # group into sub-arrays
jq 'sort_by(.age)' users.json        # sort by field
```

### String Functions

```bash
jq '.name | split(" ")'              # ["first", "last"]
jq '["a","b"] | join(",")'           # "a,b"
jq '.name | test("^A")'              # regex match → true/false
jq '.name | ascii_downcase'          # lowercase
jq '.name | ascii_upcase'            # UPPERCASE
jq '.name | ltrimstr("Mr. ")'        # strip prefix
jq '.path | gsub("/"; "-")'          # regex replace
```

---

## 🏗️ Object Construction & String Interpolation

```bash
# Build new objects from existing data
jq '.[] | {name: .name, score: .points}' data.json

# Dynamic keys using parentheses
jq '.[] | {(.name): .value}' pairs.json

# String interpolation (always use -r for clean output)
jq -r '.[] | "Name: \(.name), Age: \(.age)"' users.json

# Construct arrays
jq '[.[] | .name]' users.json

# Add/override fields
jq '. + {status: "active"}' user.json

# Merge objects
jq '.[0] * .[1]' -s a.json b.json
```

---

## 🔀 Conditionals & Logic

```bash
# If-then-else
jq '.[] | if .score > 90 then "A" elif .score > 80 then "B" else "C" end' grades.json

# Alternative operator (null coalescing)
jq '.nickname // .name' user.json     # use nickname, fall back to name
jq '.value // empty' data.json        # skip nulls entirely

# Comparisons
jq '.[] | select(.age >= 18 and .age <= 65)' users.json
jq '.[] | select(.active == true or .role == "admin")' users.json
jq '.[] | select(.banned | not)' users.json
```

---

## 📌 Variables & Reduce

```bash
# Bind a value to a variable
jq '.max_score as $max | .[] | . / $max * 100' data.json

# Reduce — fold array into single value
jq 'reduce .[] as $x (0; . + $x)' numbers.json        # sum

# CLI variables with --arg and --argjson
jq --arg env "prod" '.[] | select(.environment == $env)' deploys.json
jq --argjson threshold 0.5 '.[] | select(.score > $threshold)' results.json

# Environment variables
jq -n 'env.HOME'                      # read $HOME
jq -n '$ENV.PATH'                     # same thing
```

---

## 🛡️ Error Handling

```bash
# Try-catch
jq '.[] | try .name catch "unknown"' data.json

# ? operator — suppress errors silently
jq '.items[]?.name' data.json         # no error if .items is null

# Optional object access
jq '.config.deep.nested? // "default"' config.json
```

---

## 🧪 Advanced Features

### Recursive Descent

```bash
# Find all values for key "id" anywhere in the tree
jq '.. | .id? // empty' nested.json
```

### Path Operations

```bash
jq 'path(.users[0].name)'                # ["users", 0, "name"]
jq 'getpath(["users", 0, "name"])'       # value at that path
jq 'setpath(["users", 0, "name"]; "Bob"' # set value
jq 'del(.users[0])'                       # delete element
```

### Format Strings

```bash
jq -r '.data | @csv'                  # CSV output
jq -r '.data | @tsv'                  # TSV output
jq -r '.value | @base64'             # Base64 encode
jq -r '.encoded | @base64d'          # Base64 decode
jq -r '.query | @uri'                # URL-encode
jq -r '.snippet | @html'             # HTML-escape
jq -r '.cmd | @sh'                   # Shell-escape (for eval)
```

### Custom Functions

```bash
jq 'def double: . * 2; [.[] | double]' numbers.json
jq 'def sigma(f): reduce .[] as $x (0; . + ($x | f)); sigma(. * .)' numbers.json
```

---

## 💡 Practical One-Liners

```bash
# Pretty-print JSON
jq '.' ugly.json

# Minify JSON
jq -c '.' pretty.json

# Extract field from array of objects
jq -r '.[].name' users.json

# Filter and reshape
jq '[.[] | select(.status == "active") | {name, email}]' users.json

# Count array elements
jq '.items | length' data.json

# Sum a field
jq '[.[] | .price] | add' orders.json

# Merge two JSON files
jq -s '.[0] * .[1]' defaults.json overrides.json

# Delete a field from all objects
jq '[.[] | del(.password)]' users.json

# JSON to CSV
jq -r '.[] | [.name, .age, .email] | @csv' users.json

# Curl + jq pipeline
curl -s https://api.example.com/data | jq '.results[] | {id, title}'

# Group and count
jq 'group_by(.status) | map({status: .[0].status, count: length})' items.json

# Flatten nested structure
jq '[.departments[].employees[].name]' org.json

# Sort by field, take top 5
jq '[sort_by(-.score) | limit(5; .[])]' leaderboard.json

# Convert object to key=value lines
jq -r 'to_entries[] | "\(.key)=\(.value)"' config.json
```

---

## 📊 Comparison Chart

| Tool | Purpose | Language | Interactive | Strengths | Weaknesses |
|------|---------|----------|-------------|-----------|------------|
| **jq** | JSON processor | C | No | Fast, zero deps, ubiquitous | Cryptic syntax, poor errors |
| **yq** | YAML/JSON/XML processor | Go | No | Multi-format, jq-like syntax | Slower, two competing forks |
| **gron** | Flatten JSON to grep-able lines | Go | No | Grep-friendly, simple | No transforms, just viewing |
| **fx** | Interactive JSON viewer | Go | Yes | TUI navigation, mouse support | Not for scripting |
| **jless** | JSON/YAML TUI viewer | Rust | Yes | Fast, vim-like keybindings | Read-only, no transforms |
| **miller** | CSV/JSON/TSV swiss army knife | C | No | Multi-format, table operations | Different query language |
| `python -m json.tool` | Pretty-print JSON | Python | No | Already installed | No filtering, slow startup |
| **dasel** | Query JSON/YAML/TOML/XML | Go | No | Unified syntax across formats | Less powerful than jq |

---

## 🌟 Strengths

- Fast — single C binary, no runtime dependencies
- Powerful expression language with pipes, variables, functions
- Shell-composable — reads stdin, writes stdout
- Ubiquitous — available on nearly every Linux/macOS system
- Handles streaming and large files efficiently
- Built-in format strings (@csv, @tsv, @base64, @uri, @sh)

---

## ⚠️ Weaknesses

- Syntax is cryptic for newcomers (especially string interpolation and reduce)
- Error messages are often unhelpful — wrong line numbers, vague descriptions
- No [[YAML]] support (use [[yq]] instead)
- Memory-bound on very large single JSON objects (arrays stream fine)
- No in-place file editing — must redirect to a temp file
- No built-in HTTP client — depends on [[curl]] or similar for fetching

---

## 🔗 Related Notes

- [[JSON]]
- [[pipe]]
- [[sed]]
- [[AWK]]
- [[curl]]
- [[API]]
- [[YAML]]
- [[CSV]]
- [[Bash]]
- [[grep]]

---

## 🌐 External Resources

- jq Manual: `https://jqlang.github.io/jq/manual/`
- jqplay (online playground): `https://jqplay.org`
- jq Cookbook: `https://github.com/stedolan/jq/wiki/Cookbook`
- jq GitHub: `https://github.com/jqlang/jq`

---

## 📝 Summary

jq is the standard tool for processing JSON on the command line. The driving example — extracting `reward_aim_scale` and `reason` from a nested sweep override file — touches most core concepts: dot notation, array iteration, bracket notation for special keys, string interpolation, and piping. Pair it with `-r` for clean output and `curl` for API work.
