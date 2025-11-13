# gunzip 🗜️

`gunzip` is a command-line utility used to decompress files that have been compressed using the GNU `gzip` format. It is the inverse of `gzip`, which is widely used in Unix-like systems for reducing file sizes of executables, text, logs, and archives. `gunzip` works on single files or streams and is commonly used in combination with other tools in pipelines.

---
## ⚙️ Overview
- `gunzip` decompresses `.gz` files (Gzip-compressed files) back to their original form.
- Supports single-file decompression, streams from stdin/stdout, and batch operations.
- Often used in combination with `tar` (`.tar.gz` archives) for packaging/unpacking files.
- Can preserve timestamps and file permissions.

Basic syntax:
- Decompress a file: `gunzip file.gz` → produces `file`
- Decompress to stdout: `gunzip -c file.gz > file`
- Keep original compressed file: `gunzip -k file.gz`
- Decompress multiple files: `gunzip *.gz`

---
## 🧠 Core Concepts
- **Compression Algorithm:** Uses DEFLATE (combination of LZ77 and Huffman coding).
- **File Structure:** Gzip files contain a header (with optional metadata), compressed data blocks, and a CRC32 checksum.
- **Streaming Support:** `gunzip` can read from standard input (`stdin`) and write to standard output (`stdout`) without intermediate files.
- **File Extension:** `.gz` is the conventional extension for Gzip-compressed files.
- **Integration:** Works seamlessly with `tar` for `.tar.gz` or `.tgz` archives.

---
## 🛠️ Developer Tools / Integration
- **Command-line:** `gunzip`, `gzip`, `zcat`, `zless`, `zmore`
- **Libraries & APIs:**
  - **C:** `zlib` provides `gzopen`, `gzread`, `gzwrite`, `gzclose`
  - **Python:** `gzip` module (`gzip.open`, `gzip.GzipFile`)
  - **Zig:** Zig standard library supports zlib/Gzip decompression via `std.compress`
  - **Elixir:** Uses Erlang’s `:zlib` module for handling Gzip streams (`:zlib.gunzip/1`)
- **Pipeline Usage:** `gunzip -c file.gz | tar xf -` for extracting compressed archives in one step.

---
## 🧮 Comparison Chart — gzip vs Other Compression Tools

| Tool | Compression Type | File Extension | Streaming Support | Typical Use Cases |
|---|---|---|---|---|
| gzip / gunzip | DEFLATE | `.gz` | Yes | Single-file compression, tarballs |
| bzip2 / bunzip2 | Burrows-Wheeler + Huffman | `.bz2` | Yes | Higher compression than gzip, slower |
| xz / unxz | LZMA2 | `.xz` | Yes | Maximum compression for large files, slower |
| zip / unzip | DEFLATE + container | `.zip` | Yes | Multi-file archives, cross-platform |
| 7z | LZMA / LZMA2 | `.7z` | Yes | High compression ratio, multi-platform |

---
## 🔍 Use Cases
- Decompress log files for analysis.
- Unpack software distributions (`.tar.gz` archives).
- Stream decompression in scripts/pipelines.
- Integrate with programming languages for runtime decompression:
  - Python: read compressed datasets (`gzip.open('data.csv.gz')`)
  - Zig: embedded firmware decompression
  - Elixir: decompress log files or network payloads on-the-fly

---
## 💪 Strengths
- Fast and widely supported.
- Streaming capable (low memory footprint).
- Cross-platform compatible format.
- Simple and scriptable.

---
## ⚠️ Weaknesses
- Moderate compression ratio compared to `xz` or `zstd`.
- No native support for multi-file archives without `tar`.
- Original Gzip format doesn’t include file hierarchy, permissions preserved only in tarballs.

---
## 🔧 Developer Examples
- **C (zlib):**
  - `gzopen`, `gzread` for programmatic decompression.
- **Python:**
  - `with gzip.open('file.gz', 'rt') as f: data = f.read()`
- **Zig:**
  - `std.compress.gzipDecompress` reads compressed buffer to original data.
- **Elixir:**
  - `:zlib.gunzip(compressed_binary)`

---
## 🧩 Related Concepts / Notes
- - [[gzip]] (compression utility)
- - [[tar]] (archiving for `.tar.gz` files)
- - [[zlib]] (C library for DEFLATE compression)
- - [[Python gzip module]] (read/write compressed files)
- - [[Zig]] (supports zlib/Gzip decompression)
- - [[Elixir]] (`:zlib` interface)
- - [[Stream Processing]] (useful for pipelines)
- - [[Compression Algorithms]] (DEFLATE, LZ77, Huffman)

---
## 📚 External Resources
- `man gunzip` and `man gzip`
- zlib official documentation: https://zlib.net
- Python `gzip` module documentation: https://docs.python.org/3/library/gzip.html
- Zig standard library: `std.compress`
- Erlang/Elixir `:zlib` docs
- Wikipedia: [Gzip](https://en.wikipedia.org/wiki/Gzip)

---
## 🧩 Summary
`gunzip` is a fundamental tool for decompressing Gzip files on Unix-like systems. Its combination with `tar` allows for portable software packaging, dataset distribution, and log management. Its programmatic interfaces in C, Python, Zig, and Elixir make it versatile for embedded systems, robotics pipelines, and cross-language applications.

---
## 💡 Quick Commands
- Decompress a single file: `gunzip file.gz`
- Decompress to stdout: `gunzip -c file.gz > file`
- Keep original file: `gunzip -k file.gz`
- Decompress multiple files: `gunzip *.gz`
- Combine with tar: `tar xzf archive.tar.gz`
- Inspect compressed content without extraction: `zcat file.gz | head`
