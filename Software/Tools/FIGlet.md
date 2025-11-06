# FIGlet (text banner software)

FIGlet is a classic text-to-ASCII art banner generator originally written in the early 90s. It’s still commonly found on Linux systems today, often used to generate stylized terminal titles, headers in documentation, or to add personality to command-line tools. In robotics and embedded/Linux ROS environments, FIGlet can be used to visually mark phases of scripts, CI build stages, test suite outputs, or robot device boot banners.

---

## 🧠 Overview

FIGlet transforms normal text into large ASCII banners using "fonts" (text layout templates). It can be used standalone (`figlet`) or via compatible tools like `toilet` or libraries in Python, Go, etc.

---

## 🧩 Core Concepts

- fonts (*.flf files) define how glyphs are drawn using ASCII patterns
- outputs are plain ASCII, so works everywhere, including over serial console
- commonly used inside scripts or startup services
- simple, stable, essentially zero-maintenance

---

## 📊 Comparison Chart

| Tool / Concept | Type | Output Style | Notes |
|---|---|---|---|
| FIGlet | CLI program | ASCII block text | Most widely installed baseline |
| Toilet | CLI program | ASCII block text with colors/filters | More modern filters, FIGlet compatible fonts |
| cowsay | novelty CLI | ASCII speech bubble w/ cow | Not a banner generator |
| pyfiglet | Python library | ASCII block text | Used inside programs/scripts |
| Rich (Python) | Python library | Unicode / styled terminal text | More modern; not ASCII font-based |

---

## 🧰 Use Cases

- marking sections in long CI logs
- printing banners at robot boot over UART
- printing headings in benchmarking scripts
- creating visually distinct startup messages in ROS launch bash scripts
- quick decorative titles in README or console demos

---

## ✅ Strengths

- tiny
- ubiquitous on Linux
- human-readable output anywhere (even dmesg logs if printed via init)
- great for quick visual structure without GUI

---

## ❌ Weaknesses

- ASCII-only art (not Unicode)
- can become noisy if over-used in logs
- fonts vary per system unless packaged with the app

---

## 📎 Related Notes / Connections

- [[Linux Service Units]] (e.g. systemd boot splash banners)
- [[Shell Scripting]] (bash integration)
- [[Terminal Emulators]]

---

## 🌐 Useful External Resources

- FIGlet Home: http://www.figlet.org/
- FIGlet Fonts Archive: http://www.figlet.org/fontdb.cgi
- pyfiglet PyPI: https://pypi.org/project/pyfiglet/
