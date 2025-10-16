# QuickBMS Cheatsheet

Quick reference for using **QuickBMS**, a scriptable tool for extracting, analyzing, and reimporting data from binary files. This cheatsheet covers command-line usage, core commands, script syntax, and workflow tips for reverse engineering proprietary file formats or data logs.

---

## ⚙️ Command-Line Basics

Run QuickBMS from the terminal or command prompt.  
Syntax: `quickbms [options] <script.bms> <input_file_or_folder> <output_folder>`

**Common Options:**
- `-o` → Overwrite existing files without asking  
- `-r` → Reimport mode (replace modified files back into the archive)  
- `-l` → List files without extracting  
- `-d` → Extract to current directory  
- `-k` → Keep existing output structure  
- `-Q` → Quiet mode (no interactive prompts)  
- `-w` → Write-only mode for specific operations  

**Examples:**
- `quickbms script.bms game.dat output_folder` → Extract files  
- `quickbms -r script.bms game.dat output_folder` → Reimport edited files  
- `quickbms -l script.bms firmware.bin` → List contents only  

---

## 🧠 Core BMS Script Syntax

Scripts describe binary layouts using a simple command syntax.

**Basic Commands**
- `get VAR SIZE` → Read SIZE bytes into variable VAR  
- `getdstring VAR SIZE` → Read null-terminated or fixed-length string  
- `goto OFFSET` → Move file pointer  
- `savepos VAR` → Save current position into VAR  
- `math VAR OP VALUE` → Perform math operation (e.g., `math OFFSET + 4`)  
- `log NAME OFFSET SIZE` → Extract block of SIZE bytes starting at OFFSET  
- `idstring "ABC"` → Verify file signature (header check)  
- `if VAR == VALUE ... endif` → Conditional logic  

**Variables**
- Can hold integers, strings, or file offsets  
- Predefined variables include `OFFSET`, `SIZE`, `FILESIZE`, `FILEPATH`  

**String Commands**
- `string VAR += TEXT` → Append text  
- `string VAR p TEXT` → Print text  
- `string VAR t` → Trim whitespace  

---

## 🔄 Workflow Example

1. **Identify file format**  
   Use a hex editor or `xxd` to inspect headers.
2. **Search for existing script**  
   Visit [QuickBMS Script Repository](https://aluigi.altervista.org/quickbms_scripts.htm).
3. **Test extraction**  
   Run `quickbms your_script.bms target_file output/`.
4. **Inspect output**  
   Check results; if wrong, modify script offsets or data types.
5. **Reimport if needed**  
   Edit files and use `quickbms -r your_script.bms target_file output/`.

---

## 🧩 Useful Flags

| Flag | Description |
|------|--------------|
| `-f` | Filter files to extract (wildcards allowed) |
| `-a` | Automatically extract multiple archives in folder |
| `-V` | Verbose logging |
| `-S` | Skip errors silently |
| `-M` | Use memory file for internal processing |
| `-L` | Log operations for debugging |
| `-t` | Test script without writing output |

---

## 🔒 Common Encryption/Compression Support

QuickBMS supports built-in algorithms for most binary formats:
- **Compression:** zlib, LZO, LZMA, gzip, bzip2, deflate, etc.  
- **Encryption:** XOR, AES, Blowfish, TEA, custom bit-level algorithms  
- **Hashing:** MD5, CRC32, SHA1  

Use commands like:
- `comtype zlib`
- `encryption xor "key"`
- `clog NAME OFFSET ZSIZE SIZE` (for compressed data extraction)

---

## 🧰 Debugging Tips

- Use `print` or `log` commands to visualize variable values.  
- Use `findloc OFFSET string "pattern"` to locate patterns.  
- Combine `math` and `savepos` for dynamic structures.  
- Use `goto` carefully — incorrect offsets can corrupt extraction.  
- Run `quickbms -V` to get verbose logs for troubleshooting.  

---

## 📁 File Naming and Paths

- Avoid spaces in paths; wrap in quotes if needed.  
- Output files keep directory structure by default.  
- For large archives, prefer extracting to SSDs to avoid slow I/O.  

---

## 🧩 Integration Notes

QuickBMS scripts can be invoked from:
- [[Python]] scripts (via `subprocess.run`)  
- [[Bash]] pipelines  
- [[CI-CD]] workflows for automated data extraction  

---

## 🔍 Related Concepts

- [[binwalk]] (Firmware and archive scanning)  
- [[Radare2]] (Binary reverse engineering)  
- [[Firmware Analysis]] (Embedded systems inspection)  
- [[Compression Algorithms]] (Data decompression methods)  
- [[Binary File Formats]] (Structured data formats)  
- [[Reverse Engineering]] (Understanding proprietary systems)  

---

## 🌐 External Resources

- **Official Site:** [https://aluigi.altervista.org/quickbms.htm](https://aluigi.altervista.org/quickbms.htm)  
- **Script Repository:** [https://aluigi.altervista.org/quickbms_scripts.htm](https://aluigi.altervista.org/quickbms_scripts.htm)  
- **Community Wiki:** Reverse engineering forums and GitHub gists  
- **Author:** Luigi Auriemma  

---

## 📚 Summary

QuickBMS is one of the most powerful, scriptable binary extraction tools available. Its strength lies in its flexibility — users can model almost any data format through scripting. Once mastered, it serves as a universal parser for binary archives in games, firmware, and robotics data systems.

---
