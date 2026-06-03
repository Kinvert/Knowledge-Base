# Bubblewrap

**Bubblewrap** is a low-level Linux sandboxing tool, usually run as `bwrap`, that starts a process inside a restricted environment. It is best known as the sandbox helper underneath [[Flatpak]], but it is also used by developer tools such as [[Codex Cheatsheet]] when they need to run shell commands with constrained filesystem, process, and network access.

---

## 📚 Overview

Bubblewrap creates a temporary execution environment using Linux kernel isolation features such as namespaces, bind mounts, `tmpfs`, and `no_new_privs`. Instead of shipping an entire container image like [[Docker]], it builds a sandbox around an existing executable on the host.

The practical idea is simple: a parent program decides what the child process can see, mounts only those paths, optionally removes network access, creates a private `/proc`, and then runs the command. This makes Bubblewrap useful for application packaging, AI coding agents, build tools, test runners, and other workflows where untrusted or semi-trusted code should not get unrestricted host access.

---

## 🧠 Core Concepts

- **`bwrap` executable**: The command-line program installed by the `bubblewrap` package.
- **Linux namespaces**: Kernel isolation boundaries for mounts, PIDs, network, IPC, UTS hostname, users, and cgroups.
- **Bind mounts**: Host directories exposed inside the sandbox, often read-only with `--ro-bind`.
- **`tmpfs` mounts**: Empty temporary filesystems used for private writable paths like `/tmp`.
- **Private `/proc`**: A sandbox-local process view created with `--proc /proc`.
- **Network isolation**: `--unshare-net` gives the sandbox its own network namespace with no normal host network access.
- **Parent lifecycle control**: `--die-with-parent` terminates the sandboxed process when the supervisor exits.
- **No image format**: Bubblewrap does not pull images, build layers, or manage long-running container services.

---

## ⚙️ How It Works

Bubblewrap is a sandbox constructor rather than a full container platform. A launcher builds a command line describing which namespaces to unshare and which filesystems to mount, then `bwrap` pivots the child process into that restricted view.

Typical sandbox setup includes:

- Create a new mount namespace.
- Mount a small root filesystem view.
- Add read-only access to system directories needed to execute programs.
- Add writable scratch space with `tmpfs`.
- Optionally hide the network with `--unshare-net`.
- Run the target command with fewer ways to reach the host.

This is why it fits tools like [[Flatpak]] and Codex: the controlling program can keep using the host OS while giving each launched process only the paths and permissions it needs.

---

## 🤖 Codex Warning Explained

If Codex prints:

```text
Codex could not find bubblewrap on PATH. Install bubblewrap with your OS package manager.
Codex will use the bundled bubblewrap in the meantime.
```

that usually means:

- Codex started on Linux or WSL2.
- It looked for `bwrap` on `PATH`.
- No system `bwrap` binary was found.
- Codex fell back to its bundled Bubblewrap binary.

This is normally a warning, not a fatal error. Codex can still sandbox commands if the bundled copy works. Installing the distro package is still preferable because it receives OS security updates, matches the host distribution better, and avoids relying on the bundled fallback.

On Ubuntu or Debian:

```bash
sudo apt update
sudo apt install bubblewrap
which bwrap
bwrap --version
```

Expected result:

```text
/usr/bin/bwrap
bwrap 0.x.x
```

If `which bwrap` prints nothing, the package is missing or the executable is not on `PATH`.

---

## 🔧 Useful Commands

- `which bwrap` - Check whether Bubblewrap is installed on `PATH`.
- `bwrap --version` - Print the installed version.
- `bwrap --help` - Show available sandbox flags.
- `apt show bubblewrap` - Inspect the distro package on Ubuntu or Debian.
- `sudo apt install bubblewrap` - Install the package on Ubuntu or Debian.

Minimal smoke test:

```bash
bwrap --ro-bind /usr /usr --ro-bind /bin /bin --proc /proc --dev /dev --tmpfs /tmp --unshare-pid --die-with-parent /bin/sh -c 'echo inside sandbox; ls /tmp'
```

Network-blocking example:

```bash
bwrap --ro-bind /usr /usr --ro-bind /bin /bin --proc /proc --dev /dev --tmpfs /tmp --unshare-net --die-with-parent /bin/sh
```

These examples are intentionally simple. Real sandboxes usually need additional read-only mounts such as `/lib`, `/lib64`, `/etc`, or distribution-specific loader paths depending on the binary being launched.

---

## 📊 Comparison Chart

| Tool | Main Role | Isolation Style | Image Format | Common Use | Best Fit |
|------|-----------|-----------------|--------------|------------|----------|
| Bubblewrap | Low-level sandbox launcher | Linux namespaces and bind mounts | No | Restrict one process tree | App/tool sandboxing |
| [[Flatpak]] | Desktop app packaging | Bubblewrap plus portals and runtimes | Flatpak bundles | Linux GUI apps | Cross-distro desktop apps |
| [[Docker]] | Container platform | Namespaces, cgroups, images | OCI-style images | Dev, CI, services | Reproducible environments |
| Podman | Daemonless container engine | OCI containers, rootless support | OCI images | Docker-like workflows | Rootless containers |
| [[Snap]] | Universal Linux package system | AppArmor, mounts, store model | Snap packages | Ubuntu app distribution | Managed app packaging |
| [[AppImage]] | Portable Linux app bundle | Usually little/no sandbox by default | Single executable image | Portable desktop apps | Easy app distribution |
| `chroot` | Filesystem root change | Filesystem view only | No | Legacy isolation/setup | Simple rootfs testing |

---

## ✅ Strengths

- Lightweight compared to full containers or virtual machines.
- Works well for per-command sandboxing.
- Good fit for rootless user workflows on modern Linux systems.
- Fine-grained filesystem exposure through bind mounts.
- Used by mature tooling such as [[Flatpak]].
- Useful when the launcher wants isolation without managing container images.

---

## ❌ Weaknesses and Gotchas

- Low-level and easy to misconfigure.
- Not a complete security policy by itself; the launcher must choose safe mounts and flags.
- Depends on Linux kernel namespace support.
- Some systems disable unprivileged user namespaces, which can break rootless use.
- Hardware, GPU, USB, and desktop integration require explicit device/socket access.
- Network behavior depends on whether the launcher uses `--unshare-net`.
- Command examples can be distro-specific because dynamic linkers and libraries live in different paths.
- Does not replace [[Docker]] for image builds, service orchestration, registries, or multi-container workflows.

---

## 🧰 Use Cases

- Sandboxing desktop apps via [[Flatpak]].
- Running AI-generated commands with restricted filesystem access.
- Building safer local developer tools and test runners.
- Launching commands with no network access.
- Creating disposable writable directories with `tmpfs`.
- Testing how software behaves with a minimal filesystem view.
- Reducing blast radius for scripts that should only read or write specific paths.

---

## 🔐 Security Notes

Bubblewrap is useful because it narrows what a child process can reach, but it should be treated as one layer of defense. The quality of the sandbox depends on the exact mount list, namespace flags, kernel configuration, and the permissions of the parent process.

Important checks:

- Prefer read-only binds for system directories: `--ro-bind`.
- Avoid binding sensitive paths unless required.
- Use `--unshare-net` when the command does not need network access.
- Use `--die-with-parent` for supervised tool execution.
- Do not assume a sandboxed process is safe if secrets, SSH agents, browser profiles, or writable project paths are mounted inside it.
- Keep the system package updated through [[apt]] or the distro package manager.

---

## 🧪 Troubleshooting

### `bwrap: command not found`

Install the package:

```bash
sudo apt install bubblewrap
```

Then verify:

```bash
which bwrap
bwrap --version
```

### Codex says Bubblewrap is missing

Install `bubblewrap` with the OS package manager, restart Codex, and check whether the warning disappears. If Codex still reports the warning, confirm that `/usr/bin` or the install location is on `PATH`.

### Sandbox fails on Linux or WSL2

Check whether unprivileged user namespaces are available. Some hardened systems disable them. On Ubuntu-like systems, the relevant settings may involve `kernel.unprivileged_userns_clone` or AppArmor restrictions.

### Command cannot find libraries

The sandbox may not include the dynamic linker or shared libraries needed by the executable. Add read-only binds for the required system paths, or let a higher-level tool build the Bubblewrap command for you.

---

## 🧩 Relationship to Codex

Codex uses sandboxing to limit what shell commands can do in the current workspace. On Linux, Bubblewrap is the default sandbox mechanism. The user-visible sandbox mode, such as `read-only` or `workspace-write`, is a higher-level Codex policy; Bubblewrap is one of the lower-level tools that helps enforce it.

That means the warning is about the sandbox helper binary, not about the Codex model or API. If Codex can use its bundled `bwrap`, work can continue. Installing the system package simply makes the dependency explicit and easier to maintain.

---

## 🔗 Related Concepts

- [[Codex Cheatsheet]] (Codex CLI commands, sandbox modes, and warning context)
- [[Flatpak]] (Desktop app packaging system built around Bubblewrap sandboxing)
- [[Docker]] (Full container platform with images and service workflows)
- [[Docker Container]] (Runtime instance of a container image)
- [[Ubuntu]] (Common distro where `sudo apt install bubblewrap` applies)
- [[apt]] (Debian/Ubuntu package manager used to install Bubblewrap)
- [[procfs]] (`/proc` view commonly mounted inside sandboxes)
- [[AppImage]] (Portable Linux app format with weaker default sandboxing)
- [[Snap]] (Linux app packaging system with a different sandbox model)

---

## 📚 Further Reading

- [Bubblewrap GitHub Repository](https://github.com/containers/bubblewrap)
- [Bubblewrap Debian Man Page](https://manpages.debian.org/testing/bubblewrap/bwrap.1.en.html)
- [Flatpak Sandboxing Documentation](https://docs.flatpak.org/en/latest/sandbox-permissions.html)
- [OpenAI Codex Sandboxing Prerequisites](https://developers.openai.com/codex/concepts/sandboxing#prerequisites)

---
