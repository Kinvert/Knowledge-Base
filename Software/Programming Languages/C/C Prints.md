# C Prints

C provides several functions for formatted and unformatted output, primarily through the `printf` family. These are used to print data to different destinations—such as the console, strings, or files—with fine-grained control over formatting. Understanding these is crucial for debugging, embedded logging, and low-level output formatting in robotics and embedded systems.

---

## ⚙️ Overview

C printing functions allow developers to write formatted text to standard output, memory buffers, or custom streams. They form the backbone of textual output in C programs, from debugging messages to file logging.

---

## 🧠 Core Concepts

- **Format specifiers** (`%d`, `%f`, `%s`, etc.) control how variables are converted to text.  
- **Destination** determines whether the output goes to the terminal, a string, or a file.  
- **Buffer safety** is a major concern—some functions (like `sprintf`) can overflow buffers.  
- **Return value** often represents the number of characters printed (excluding the null terminator).

---

## 📊 Comparison Chart

| Function | Destination | Safe? | Null Terminator | Typical Use | Notes |
|-----------|-------------|-------|-----------------|--------------|-------|
| `printf` | Standard output (`stdout`) | ✅ | N/A | Console printing | Most common |
| `fprintf` | `FILE*` stream | ✅ | N/A | Logging to files or stderr | Flexible |
| `sprintf` | String buffer | ❌ | ✅ | Writing to char array | Dangerous (no bounds check) |
| `snprintf` | String buffer (with size limit) | ✅ | ✅ | Safe string formatting | Preferred over `sprintf` |
| `vsprintf` | String buffer (va_list) | ❌ | ✅ | Internal vararg use | Used inside custom wrappers |
| `vsnprintf` | String buffer (va_list, bounded) | ✅ | ✅ | Safe custom formatters | Used in embedded logging |
| `asprintf` | Dynamically allocates string | ✅ | ✅ | Simplifies string handling | GNU extension |
| `dprintf` | File descriptor (int fd) | ✅ | N/A | Low-level output to sockets/files | POSIX, not ANSI C |
| `puts` | Standard output (`stdout`) | ✅ | Adds newline | Simple message printing | No format support |
| `fputs` | `FILE*` stream | ✅ | No newline | Writing raw strings | Good for file output |
| `putchar` | Single char to stdout | ✅ | N/A | Minimal IO | Often used in embedded systems |

---

## 🔧 Strengths

- **`printf`**: Easy and universal for quick debugging.  
- **`fprintf`**: Ideal for file-based or stream logging.  
- **`snprintf`**: Safe and recommended for buffer writing.  
- **`asprintf`**: Automatically handles dynamic allocation.  
- **`dprintf`**: Integrates with sockets and file descriptors.  
- **`vsnprintf`**: Enables flexible, reusable formatting functions.

---

## ❌ Weaknesses

- **`sprintf`** can easily cause buffer overflows.  
- **`printf`** is relatively slow for frequent embedded prints.  
- **`asprintf`** requires manual `free()`.  
- **`puts`** and **`putchar`** lack formatting control.  
- **`fprintf`** and **`dprintf`** depend on proper file management.

---

## 🧰 Use Cases

- **Robotics debugging**: `snprintf` or `vsnprintf` for safe UART or CAN message formatting.  
- **Logging systems**: `fprintf` for logs to file; `dprintf` for socket-based telemetry.  
- **Embedded systems**: `snprintf` to format messages into fixed buffers for transmission.  
- **String building**: `asprintf` when size is dynamic or unknown.

---

## 🏆 Best Practices

- Prefer `snprintf` to avoid buffer overflows.  
- Use `fprintf(stderr, ...)` for errors and `printf(...)` for standard messages.  
- Avoid `sprintf` in all safety-critical or embedded code.  
- Use `vsnprintf` to build wrapper functions like `log_printf`.  
- When dynamic memory is acceptable, `asprintf` simplifies code.

---

## 📘 Related Concepts / Notes

- [[C Strings]] (Null-terminated byte arrays)
- [[Format Specifiers in C]] (Details of `%d`, `%f`, `%s`, etc.)
- [[C File I-O]] (Working with `FILE*` and streams)
- [[Buffer Overflow]] (Memory safety issue)
- [[Embedded Logging]] (Output in constrained systems)

---

## 🌐 External Resources

- [GNU C Library printf manual](https://www.gnu.org/software/libc/manual/html_node/Formatted-Output-Functions.html)
- [ISO C Standard (printf family)](https://port70.net/~nsz/c/c11/n1570.html#7.21.6)
- [CERT Secure Coding Guidelines](https://wiki.sei.cmu.edu/confluence/display/c/STR02-C.+Sanitize+format+strings+in+printf+functions)

---
