---
title: JavaScript
aliases:
  - JS
  - ECMAScript
  - ECMAScript Language
tags:
  - programming-language
  - web
  - runtime
---

# JavaScript

**JavaScript** is a high-level, dynamically typed programming language standardized as ECMAScript. It is the default language for the browser and increasingly used in server-side services through runtimes like [[Node.js]].

For robotics and systems tooling, it is often used for:
- web dashboards and remote control panels
- websocket-based telemetry streaming
- rapid scripting around REST/gRPC APIs

---

## Core traits

- Prototype-based object model.
- Event-driven programming and non-blocking I/O in many runtimes.
- First-class functions and async/await syntax.
- Huge ecosystem under `npm`.

---

## Language vs runtime split

- **Language**: syntax and semantics (`.js` files, types are optional).
- **Runtime**: where code executes, like browsers, [[Node.js]], edge workers, or embedded JS engines.
- **Tooling**: transpilers, bundlers, and frameworks shape what you can ship.

---

## Use cases

- Browser UIs for robot teleoperation.
- Frontends for visualization and experiment dashboards.
- Backends exposing APIs to ROS2 bridges or simulation services.
- Desktop shells around command-line robotics tools.

---

## Comparison table

| Language | Typing | Concurrency Style | Primary Domain | Strength | Weakness |
|---|---|---|---|---|---|
| JavaScript | Dynamic | Event loop + async | Web + APIs + tooling | Massive ecosystem, fast iteration | Runtime surprises for CPU-heavy workloads |
| Python | Dynamic | Threads/processes | ML/automation/ROS tooling | Easy scientific integration | Lower raw performance for high-throughput servers |
| C++ | Static | Threads | Embedded + low-level controls | Deterministic performance | Slower development velocity |
| Rust | Static | Async/threads | Systems + high-reliability | Safety + speed | Steep learning curve |
| Go | Static | Goroutines | Backends/services | Simplicity + scaling | Smaller robotics ecosystem than JS/Python |

---

## Pros / Cons

### ✅ Pros
- Great for event-heavy robotics UIs and streaming telemetry.
- Massive library availability.
- Easy team onboarding for web-like teams.

### ⚠️ Cons
- Real-time hard-deadline logic usually belongs elsewhere.
- Package quality varies widely.
- Browser and server behavior differ in edge cases.

---

## Related notes

- [[TypeScript]]
- [[Node.js]]
- [[npm]]
- [[WebSockets]]
- [[ROS2 Web Bridge]]

