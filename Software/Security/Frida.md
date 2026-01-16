# Frida

**Frida** is a dynamic instrumentation toolkit for injecting JavaScript into running applications. Often called "Greasemonkey for native apps," it enables security researchers and reverse engineers to hook functions, trace API calls, modify behavior, and inspect memory at runtime - without source code or recompilation. Created by Ole André Ravnås and supported by NowSecure.

---

## ⚙️ Overview

Frida injects a lightweight agent (GumJS, powered by QuickJS) into target processes. This agent executes JavaScript with full memory access, enabling function hooking, native calls, and bi-directional communication with a host script. Unlike static tools like [[IDA Pro]], Frida operates on live processes - ideal for runtime analysis, bypassing protections, and analyzing encrypted protocols.

Supported platforms: Windows, macOS, Linux, iOS, watchOS, tvOS, Android, FreeBSD, QNX.

---

## 🧠 Core Concepts

- **Agent:** JavaScript code injected into the target process
- **Interceptor:** Hook functions with onEnter/onLeave callbacks
- **NativeFunction:** Call native code from JavaScript
- **NativePointer:** Memory address wrapper with helper methods
- **Module:** Enumerate loaded libraries and exports
- **Memory:** Read/write/scan/allocate process memory
- **Stalker:** Instruction-level code tracing engine
- **Java/ObjC bridges:** High-level APIs for Android/iOS runtimes
- **Gadget:** Embeddable library for non-injection scenarios

---

## 🔧 Modes of Operation

| Mode | How It Works | Use Case |
|------|--------------|----------|
| **Injected** | frida-server injects into running/spawned process | Development, testing, rooted devices |
| **Embedded** | frida-gadget linked into target app | Jailed iOS/Android, non-rooted devices |
| **Preloaded** | frida-gadget via LD_PRELOAD/DYLD_INSERT_LIBRARIES | Autonomous execution, no external comms |

---

## 📊 Comparison Chart

| Tool | Type | Platforms | Hooks | Scripting | Persistence |
|------|------|-----------|-------|-----------|-------------|
| **Frida** | Dynamic instrumentation | Cross-platform | Native/Java/ObjC | JavaScript | Runtime |
| Xposed | Android framework | Android | Java only | Java | Persistent |
| Cydia Substrate | iOS hooking | iOS (jailbroken) | Native/ObjC | C/ObjC | Persistent |
| DynamoRIO | Binary instrumentation | Win/Linux | Native | C/C++ | Runtime |
| Intel Pin | Binary instrumentation | Win/Linux | Native | C++ | Runtime |
| [[GDB]] | Debugger | Cross-platform | Breakpoints | Python/CLI | Runtime |

---

## 🛠️ Quick Start

**Installation:**
```bash
pip install frida-tools   # CLI tools
pip install frida         # Python bindings
```

**Trace functions matching pattern:**
```bash
frida-trace -i "recv*" -i "read*" Twitter
```

Frida auto-generates JavaScript handlers in `__handlers__/` that you can customize. Changes reload automatically.

**Basic hook example:**
```javascript
Interceptor.attach(Module.getExportByName(null, 'open'), {
    onEnter(args) {
        console.log('open(' + args[0].readUtf8String() + ')');
    },
    onLeave(retval) {
        console.log('returned: ' + retval.toInt32());
    }
});
```

---

## 🛠️ Included Tools

| Tool | Purpose |
|------|---------|
| `frida` | Interactive REPL for live instrumentation |
| `frida-ps` | List running processes |
| `frida-trace` | Auto-generate hooks for functions |
| `frida-discover` | Discover internal functions |
| `frida-ls-devices` | List devices (USB, remote, local) |
| `frida-kill` | Kill process by name or PID |

---

## 🎯 Use Cases

| Domain | Application |
|--------|-------------|
| Mobile security | Bypass SSL pinning, root/jailbreak detection |
| Malware analysis | Observe runtime behavior |
| Vulnerability research | Fuzz inputs, trace attack surfaces |
| Protocol analysis | Intercept encrypted API calls |
| CTF competitions | Solve reverse engineering challenges |
| Game research | Analyze game logic and state |

---

## 📱 Mobile Platforms

**Android:**
- Rooted: run frida-server, full access
- Non-rooted: embed frida-gadget in debuggable APK
- Hooks Java (ART) and native code

**iOS:**
- Jailbroken: run frida-server, full access
- Non-jailbroken: frida-gadget in debuggable apps (iOS 13+)
- Hooks Objective-C, Swift, and native code

---

## 🌟 Strengths

- Truly cross-platform (desktop + mobile)
- Hooks native, Java, and Objective-C
- JavaScript scripting for rapid prototyping
- No persistent system modification
- Multiple bindings (Python, Node.js, Swift, .NET, Go)
- Active community and extensive documentation
- Free and open source

---

## ⚠️ Weaknesses

- Detectable by anti-tampering measures
- Requires root/jailbreak for full mobile functionality
- Can crash target if hooks are incorrect
- Performance overhead with heavy instrumentation
- Some apps implement Frida-specific detection

---

## 🔗 Related Notes

- [[IDA Pro]]
- [[GDB]]
- [[Kali Linux]]
- [[Ghidra]]
- [[Android]]
- [[iOS]]

---

## 🌐 External Resources

- [Frida Official Site](https://frida.re/)
- [Quick-Start Guide](https://frida.re/docs/quickstart/)
- [JavaScript API](https://frida.re/docs/javascript-api/)
- [GitHub Repository](https://github.com/frida/frida)
- [CodeShare](https://codeshare.frida.re/) (community scripts)
- [Frida Handbook](https://learnfrida.info/)

---

## 📝 Summary

Frida is the go-to dynamic instrumentation toolkit for security researchers and reverse engineers. By injecting JavaScript into running processes, it enables function hooking, memory inspection, and behavior modification across desktop and mobile platforms - all without source code. Its three operational modes (injected, embedded, preloaded) cover everything from development testing to production app analysis on jailed devices.
