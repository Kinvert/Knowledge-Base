# SCP (Secure Copy)

SCP (Secure Copy) is a command-line utility used to securely transfer files between hosts over a network. It relies on the [[SSH]] (Secure Shell) protocol for authentication and encryption, making it safer than older tools like `rcp`. Engineers often use SCP for moving logs, binaries, datasets, or configuration files to and from embedded devices, development boards, and remote servers.

---

## ⚙️ Overview

SCP enables encrypted file transfer between a local machine and a remote system, or between two remote systems. It uses SSH under the hood, so if you have SSH access to a system, you can also copy files with SCP.

---

## 🧠 Core Concepts

- **SSH Integration**: SCP reuses SSH for authentication and encryption.
- **File Transfer Directions**:
  - Local → Remote
  - Remote → Local
  - Remote → Remote (via local client as a middle point)
- **Command Syntax**: Typically `scp [options] source target`
- **Security**: Data is encrypted in transit.

---

## 📊 Comparison Chart

| Tool           | Protocol | Encryption | Typical Use Case                     | Notes                                  |
|----------------|----------|------------|--------------------------------------|----------------------------------------|
| SCP            | SSH      | Yes        | Quick, secure file transfer           | Simple but limited feature set          |
| [[SFTP]]       | SSH      | Yes        | Interactive file transfer sessions    | More flexible than SCP                  |
| [[rsync]]      | SSH      | Yes        | Efficient syncing of files/directories| Delta transfers, more options           |
| [[FTP]]        | FTP      | No (by default) | Legacy file transfers              | Insecure without [[FTPS]]/[[SFTP]]      |
| [[TFTP]]       | UDP      | No         | Simple, lightweight transfers         | Used in embedded/firmware contexts      |

---

## 🛠️ Use Cases

- Uploading compiled binaries to a remote robotics platform
- Copying logs or datasets from a remote robot for analysis
- Deploying configuration files during development
- Transferring results from simulation servers to local machines

---

## ✅ Strengths

- Simple command-line usage
- Built-in to most UNIX/Linux systems
- Encrypted and secure (leverages SSH)
- Works with existing SSH key infrastructure

---

## ❌ Weaknesses

- Less feature-rich than [[rsync]]
- No resume support for interrupted transfers
- Slower for large or incremental file syncs
- Limited interactive capabilities compared to [[SFTP]]

---

## 🔧 Compatible Items

- [[SSH]] (underlying protocol)
- [[Linux]]
- [[Unix-like Systems]]
- Windows (via PuTTY or OpenSSH)
- [[rsync]]
- [[SFTP]]

---

## 📚 Related Concepts

- [[SFTP]] (SSH File Transfer Protocol)
- [[rsync]] (Remote Sync)
- [[FTP]] (File Transfer Protocol)
- [[TFTP]] (Trivial File Transfer Protocol)
- [[SSH]] (Secure Shell)

---

## 🌐 External Resources

- [OpenSSH SCP Manual](https://man.openbsd.org/scp)
- [Linux man pages – scp(1)](https://linux.die.net/man/1/scp)
- [PuTTY SCP (pscp) documentation](https://www.chiark.greenend.org.uk/~sgtatham/putty/latest.html)
- https://explainshell.com/explain/1/scp#

---

## 🏆 Summary

SCP remains a widely used tool for secure file transfers due to its simplicity and reliance on SSH. While newer alternatives like [[rsync]] and [[SFTP]] provide more features and flexibility, SCP is still an effective choice for quick, encrypted copying tasks in robotics, embedded systems, and server management.
