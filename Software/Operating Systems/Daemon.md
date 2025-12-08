# Daemon

A **daemon** is a long-running background process—typically started at boot—that performs tasks without direct user interaction. Daemons handle essential system services, scheduling, networking, logging, and automation across Unix-like operating systems. In modern engineering contexts, daemons also include service managers, distributed agents, and background components of larger systems such as ML pipelines and container orchestrators.

This page serves as the *parent index* for everything related to daemons, service lifecycle, system orchestration, and background execution models.

---

## 🔧 What Makes a Daemon a Daemon?

- **Runs detached** from any terminal or user session  
- **Persists** beyond user logins  
- **Supervised** by an init system or process manager  
- Often **logs to syslog**, journal, or custom logs  
- Typically **responds to signals** (e.g., SIGHUP for reload)

Daemons are foundational to OS behavior—many services users rely on are daemons behind the scenes (e.g., `sshd`, `cron`, `systemd-journald`).

---

## 🧩 Key Components of Daemon Behavior

### 1. **Initialization & Startup**
- Fork/detach from parent process
- Change working directory
- Close or redirect standard file descriptors
- Drop privileges (run as non-root when possible)
- Register with service supervisors

### 2. **Lifecycle Management**
- Start → Running → Reload → Stop → Restart → Crash
- Signal handling (`SIGTERM`, `SIGINT`, `SIGCHLD`, etc.)
- Health checks and watchdogs
- Self-restart or external supervision

### 3. **Logging & Monitoring**
- Syslog or journald integration  
- PID files  
- Metrics export (Prometheus, StatsD, custom)  
- Tracing (OpenTelemetry, LTTng, eBPF for advanced cases)

### 4. **Security Considerations**
- Dropping to unprivileged user/group
- Chroot jails or namespaces
- Mandatory Access Control (SELinux, AppArmor)
- Resource limits (`ulimit`, cgroups)

---

## 🏗️ Daemon Types

### **System Daemons**
Core OS functionality:
- init/system manager  
- network stack
- time sync
- device management  
Examples: `systemd`, `launchd`, `cron`, `dbus-daemon`

### **Service Daemons**
Provide network-accessible or application-level services:
- web servers  
- SSH  
- databases  
Examples: `nginx`, `sshd`, `postgres`

### **Agent Daemons**
Distributed or background agents:
- container runtimes  
- telemetry exporters  
- job runners  
Examples: `dockerd`, `node_exporter`, `celeryd`

### **ML / Data Pipeline Daemons**
Constantly running workers in AI/ML pipelines:
- feature stores  
- RL simulators  
- training job dispatchers  
Examples: Ray workers, Airflow workers, Kafka consumers

---

## 🏁 Daemon Management Systems

Use these to create, supervise, and manage daemons:

### **systemd**
Modern Linux standard. Units, timers, sockets, logging, cgroups.

### **Upstart / SysVinit**
Older systems; still relevant for legacy servers.

### **supervisord**
Python-based, simple service control. Common in container setups.

### **runit / s6**
Minimalist, reliable process supervision.

### **launchd** (macOS)
Apple’s service management framework.

---

## 🎛️ Creating a Daemon (Language Examples)

Your subpages may include specific sections such as:

- Daemonizing a **C** program  
- Daemonizing with **Python** (`daemon` module, `multiprocessing`)  
- **Go** long-running processes (signals, `context.Context`)  
- **Rust** daemon best practices  
- **Zig** daemon lifecycle (manual process control)  

---

## 🗂️ Suggested Subtopics (Link From Here)

- **Systemd Units** (`.service`, `.timer`, `.socket`)
- **Process Supervision** (runit, s6, daemontools)
- **Signals & Process Control**
- **Logging Systems**
- **PID Files & Lock Files**
- **Daemon Security Hardenings**
- **Writing a Daemon in C**
- **Writing a Daemon in Zig**
- **Containers vs Traditional Daemons**
- **Distributed Daemons in AI Pipelines**
- **Job Queues / Workers (Celery, Sidekiq, Resque)**

---

## 🔗 Related Higher-Level Topics
- [[Operating Systems]]
- [[Linux Internals]]
- [[Concurrency]]
- [[AI Algorithms]] (background workers for RL environments, trainers, evaluators)
- [[Process Management]]
- [[Containers and Orchestration]]

---

Use this page as the root index and add subpages for deeper dives into implementation, examples, comparisons, and OS-specific considerations.
