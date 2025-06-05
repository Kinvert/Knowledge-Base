# MISRA (Motor Industry Software Reliability Association)

## 📝 Summary

**MISRA** refers to a set of **software development guidelines** for the C and C++ programming languages, originally developed for the **automotive industry**. It aims to **ensure code safety, portability, and reliability** in embedded systems, particularly in **safety-critical applications**.

Originally developed by a consortium led by the UK’s Motor Industry Research Association, MISRA guidelines are now widely adopted across industries beyond automotive, including **aerospace, defense, rail, and medical devices**.

---

## 📚 Versions & Scope

| Version       | Release Year | Language | Notes                               |
|---------------|--------------|----------|-------------------------------------|
| MISRA C       | 1998         | C        | First edition                       |
| MISRA C:2004  | 2004         | C        | Widely adopted                      |
| MISRA C:2012  | 2012         | C        | Modular, supports newer standards   |
| MISRA C++:2008| 2008         | C++      | For safety-critical C++ development|

> MISRA C:2012 is currently the most comprehensive and widely used version.

---

## 🎯 Goals of MISRA

- Prevent **undefined or implementation-defined behavior**
- Eliminate usage of **dangerous constructs** in C/C++
- Ensure **predictable and maintainable** code
- Improve **tool-based static analysis compatibility**
- Encourage **readability and portability**

---

## ✅ Typical Guidelines

- No dynamic memory allocation (`malloc`, `free`, etc.)
- Avoid recursion
- No use of `goto`
- Enforce initialization of variables
- Type-safe conversions and casts
- Avoid use of compiler-specific features

MISRA guidelines are broken down into:

- **Mandatory**: Must be complied with
- **Required**: Should be complied with unless a formal deviation is documented
- **Advisory**: Recommendations

---

## 🧪 Compliance & Tooling

### Tools Commonly Used
- **Static analyzers** like:
  - Polyspace
  - LDRA
  - PC-lint / Flexelint
  - Coverity
  - CodeSonar
  - Cppcheck (partial support)

### Deviation Process
MISRA allows **documented deviations** when justifiable. The deviation must:
- Be formally documented
- Be reviewed and approved
- Not compromise safety

---

## 📦 Use Cases

| Industry    | Use Case                            |
|-------------|-------------------------------------|
| Automotive  | ECU firmware, ADAS control systems  |
| Aerospace   | Flight control systems              |
| Medical     | Device firmware                     |
| Rail        | Train control and signaling         |
| Industrial  | Robotics and PLCs                   |

---

## 🧠 Pros and Cons

### ✅ Pros
- Ensures safety and reliability
- Widely accepted in regulated industries
- Supported by many tools
- Promotes better coding practices

### ❌ Cons
- Verbose and restrictive
- Learning curve
- Can slow development if not managed well
- Some rules may feel outdated in modern C++

---

## 🆚 Related Guidelines

| Standard       | Focus Area                    | Comparison to MISRA                    |
|----------------|-------------------------------|----------------------------------------|
| AUTOSAR C++14  | Modern C++ safety             | Focused on newer C++ standards         |
| CERT C/C++     | Secure coding                 | Emphasizes security over portability   |
| ISO 26262      | Functional safety (process)   | MISRA complements ISO 26262 coding     |
| JSF++          | Aviation software guidelines  | More stringent, often military-related |

---

## 🧭 See Also

- [[AUTOSAR]]
- [[ISO 26262]]
- [[Static Analysis]]
- [[Safety Critical Systems]]
