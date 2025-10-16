# QuickBMS

QuickBMS is a flexible and powerful tool for extracting and reimporting data from proprietary file formats used in video games and other binary archives. It is a script-driven extractor created by Luigi Auriemma, allowing engineers and modders to parse unknown binary formats by writing or reusing scripts that describe how data is structured. While primarily used in reverse engineering and game modding, its general-purpose design makes it useful for robotics and embedded systems work where proprietary or undocumented binary formats need to be analyzed.

---

## ⚙️ Overview

QuickBMS (Quick Binary Multi Script) interprets **BMS scripts** — human-readable instructions that describe how to unpack or repack a binary file. Its flexible scripting engine supports hundreds of commands for reading, writing, decrypting, and decompressing data. It can be used across multiple platforms including Windows, Linux, and macOS.

The tool can handle virtually any container, from common archive formats to obscure or proprietary binary logs, often found in simulation or robotics systems.

---

## 🧠 Core Concepts

- **BMS Script:** Defines how to interpret a specific binary format. Scripts can be reused across projects.
- **Endian Control:** Supports both big and little endian data.
- **Decompression Algorithms:** Built-in support for zlib, LZO, LZMA, and many others.
- **Decryption/Hashing:** Includes basic cryptographic operations to unpack encrypted data.
- **Reimport Mode:** Allows modified files to be reinserted into archives, useful for iterative debugging or modding.
- **Command-Line Interface:** Designed for automation and batch extraction in pipelines.

---

## 📊 Comparison Chart

| Tool / Feature                 | QuickBMS | [[binwalk]] | [[7zip]] | [[unzip]] | [[Radare2]] |
|--------------------------------|-----------|--------------|-----------|------------|--------------|
| Scriptable Extraction Logic     | ✅ Yes     | ⚙️ Limited   | ❌ No     | ❌ No      | ⚙️ Partial   |
| Binary Reverse Engineering Focus| ✅ High    | ✅ Moderate  | ❌ Low    | ❌ Low     | ✅ High      |
| Reimport Capability             | ✅ Yes     | ❌ No        | ❌ No     | ❌ No      | ⚙️ Limited   |
| Compression Algorithm Support   | ✅ Extensive | ⚙️ Moderate | ✅ Moderate | ✅ Basic | ⚙️ Custom   |
| GUI Support                     | ❌ No (CLI only) | ❌ No | ✅ Yes | ✅ Yes | ❌ No |
| Target Audience                 | Reverse engineers, game modders | Firmware analysts | General users | General users | Security researchers |

---

## 🧰 Use Cases

- **Game Asset Extraction:** Reading textures, meshes, and audio files from proprietary formats.
- **Log or Telemetry Data Parsing:** Reverse-engineering binary data dumps in robotics or automotive systems.
- **Firmware Analysis:** Extracting embedded archives in firmware images for diagnostics.
- **Data Conversion:** Converting unknown formats to open ones for visualization or AI training.

---

## ✅ Strengths

- Extremely flexible and script-driven
- Large public script library covering thousands of formats
- Cross-platform and lightweight
- Actively maintained and well-documented by its creator
- Supports reimporting, making it useful for testing data round-trips

---

## ❌ Weaknesses

- No graphical interface — command line only
- Steeper learning curve for non-programmers
- Dependent on available scripts — unsupported formats require manual reverse engineering
- Limited debugging features for complex binary layouts

---

## 🧩 Compatible Items

- [[binwalk]] (for embedded file detection)
- [[Radare2]] (for disassembly and reverse engineering)
- [[xxd]] (for raw hex inspection)
- [[Python]] (for automated script generation)
- [[7zip]] (for handling standard archives)
- [[Hex Editors]] (for exploring binary structures)

---

## 🔧 Developer Tools

- **BMS Script Templates:** Available in the QuickBMS package
- **Reimport and Test Modes:** Validate scripts iteratively
- **Command-Line Automation:** Can be invoked in CI or robotics pipelines
- **Integration with [[Python]] Scripts:** Automate analysis and extraction workflows

---

## 🔍 Related Concepts

- [[binwalk]] (Firmware extraction and reverse engineering)
- [[Radare2]] (Binary analysis toolkit)
- [[Reverse Engineering]] (General concept)
- [[Binary File Formats]] (Structured data storage)
- [[Compression Algorithms]] (Data reduction methods)
- [[Decompilation]] (Recovering source-like structure)
- [[Firmware Analysis]] (Embedded systems introspection)

---

## 🌐 External Resources

- **Official Website:** [https://aluigi.altervista.org/quickbms.htm](https://aluigi.altervista.org/quickbms.htm)
- **Script Repository:** [https://aluigi.altervista.org/quickbms_scripts.htm](https://aluigi.altervista.org/quickbms_scripts.htm)
- **Community Guides:** Various tutorials on reverse engineering forums
- **Author:** Luigi Auriemma (Security researcher and developer)

---

## 📚 Summary

QuickBMS is an indispensable tool for engineers dealing with proprietary or undocumented binary formats. Its modular script-driven nature and reimport functionality make it a versatile choice for robotics, embedded systems, and simulation engineers who need to access, modify, or inspect closed data formats. While its command-line interface requires some technical comfort, its capability to automate parsing at scale provides a major advantage over GUI-only extractors.

---
