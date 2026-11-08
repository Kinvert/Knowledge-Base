# systemd

**systemd** is a Linux system/service manager that starts, supervises and coordinates units such as services, sockets, timers, mounts and targets. `systemctl` inspects and controls it; a daemon executable and its unit configuration are different things.

For embedded experiments, the important boundaries are startup ordering versus readiness, runtime versus persistent configuration, and manager configuration versus the state/files of the application itself.

---

## 🧩 Unit Types

| Unit | Purpose | Example use |
|---|---|---|
| `.service` | Manage a process or finite task | Robot engine or backend daemon |
| `.socket` | Own a socket and activate a service | Demand-driven IPC/network service |
| `.target` | Group/synchronize related units | Robot application stack |
| `.timer` | Trigger another unit on a schedule | Periodic update check |
| `.mount` / `.automount` | Manage filesystems or on-demand mounts | Runtime staging or data availability |
| `.device` | Represent a kernel device | Device-dependent services |
| `.path` | Activate work on filesystem changes | File-triggered task |

Targets group units; they are not necessarily processes. Stopping a target must be understood through the actual dependency relationships, not guessed from its name.

---

## 🔍 Inspect Before Changing

Read-only examples; `example.service` stands for the actual unit being investigated:

```bash
systemctl --version
systemctl --help
systemctl cat example.service
systemctl show example.service -p FragmentPath -p DropInPaths -p ExecStart -p User -p WorkingDirectory
systemctl status --no-pager example.service
journalctl -u example.service -b --no-pager
```

- `cat` shows backing unit files/drop-ins; `show` queries the manager's effective properties.
- `status` is a human-oriented current snapshot with recent logs, not an application's full history or a readiness probe.
- `list-units --all` lists loaded units; `list-unit-files` lists installed unit files/templates.
- `--user` addresses the current user's manager, not the system manager.
- `--no-pager` is useful when collecting bounded command output.

The existence of a CLI binary does not prove a systemd manager or user bus is running in the current environment.

---

## ⏱️ Dependencies, Ordering and Readiness

`Wants=` and `Requires=` express activation/dependency relationships. `After=` and `Before=` express ordering; **`After=` alone does not pull in another unit**.

Readiness depends on `Type=`:

- `simple`: considered started after the process is forked; application initialization may still be pending.
- `exec`: waits for successful program execution, not application-level readiness.
- `notify`: waits for the application's readiness notification.
- `oneshot`: waits for the finite task to finish; `RemainAfterExit=` can preserve active state afterward.
- `forking`: legacy daemonization semantics, requiring careful main-process identification.

A PID, an `active` label and a successful end-to-end API request are different evidence. Use the service's real readiness protocol when dependent work requires it.

---

## 🔄 Lifecycle Commands Are Different

| Operation | Effect | Common misconception |
|---|---|---|
| `start` / `stop` | Change current activation | Automatically changes boot enablement |
| `restart` | Stop/start the unit | Restores old application binaries |
| `reload` | Ask a supporting application to reload its configuration | Re-read unit definitions |
| `daemon-reload` | Re-read manager unit files/dependency configuration | Restart already-running applications |
| `enable` / `disable` | Change enablement links | Start/stop immediately without `--now` |
| `mask` / `unmask` | Block/unblock activation through masking | Merely disable boot startup |
| `reset-failed` | Clear recorded failure/rate-limit state | Repair the cause of failure |

Changing system units generally requires appropriate authorization. Do not execute lifecycle commands on a robot or workstation merely to illustrate their syntax.

---

## 📁 Persistent Versus Runtime Overrides

Ordinary administrator units/drop-ins live under `/etc/systemd/system`; runtime equivalents live under `/run/systemd/system`. Vendor units commonly live under `/usr/lib/systemd/system` or a distribution-specific equivalent.

`systemctl edit --runtime example.service` requests runtime-only editing. Ordinary `edit` is persistent. Runtime configuration is lost on reboot, but external files written by the application are not thereby removed.

Precedence has two separate rules:

1. Higher-priority directories select among **same-named** unit/drop-in files; `/etc` wins over the corresponding `/run` file in the ordinary paths.
2. Different drop-in filenames are merged in lexicographic order, regardless of those directories.

Therefore, “everything in `/etc` always beats everything in `/run`” is too broad. Inspect the effective configuration after considering all drop-ins. Modern versions can also reject an edit that would be shadowed by higher-priority configuration.

---

## 🛠️ Replacing ExecStart Carefully

Many list-valued settings append across drop-ins. Replacing an existing `ExecStart=` normally needs an empty `ExecStart=` assignment before the new command; ordinary non-oneshot services cannot simply accumulate multiple start commands.

Preserve the original user, arguments, environment, working directory, dependencies and security constraints unless intentionally changing them. Unit execution is not automatically a shell command: shell operators and expansion need an explicitly chosen shell if actually required.

After manual unit/drop-in file changes, `daemon-reload` refreshes manager definitions; applying changed execution settings normally also requires restarting the service. Editing through `systemctl edit` handles manager reload, but does not make all running process settings change retroactively.

---

## 🧹 Narrow Cleanup and Recovery

Remove only the experiment-owned override, then refresh definitions and restore the intended service. Record ownership before changing anything.

`systemctl revert` can remove administrator customizations and drop-ins, including unrelated ones. It is not an appropriate generic cleanup command for a narrowly scoped experiment.

A runtime override disappearing on reboot does not prove a successful rollback if persistent binaries, settings, keys or databases changed. Test immediate teardown and reboot restoration separately before making those claims.

[[Vector Firmware Development]] applies this distinction to replacing an application temporarily without overwriting the robot's known-good tree.

---

## ⚠️ Versions and Practical Limits

Check the target's installed help/manual, not just current online documentation. A recent workstation's supported flags and directives may not exist in an older embedded image; some unknown directives can generate warnings rather than prevent a unit loading.

Restart policies, timeouts, start-rate limits, resource controls and sandboxing need deliberate configuration. Supervision does not turn a crashing or unsafe hardware controller into a correct one.

Local documentation checks used systemd/systemctl **255** help/version output. No host or robot service was started, stopped, enabled or edited to establish this note.

---

## 🔗 Related Notes

- [[Daemon]] — long-running process concepts.
- [[Linux]] — runtime environment.
- [[udevadm]] — device-triggered activation.
- [[tmpfs]] — volatile storage, not complete side-effect rollback.
- [[IPC]] — process communication and ownership.
- [[Vector Firmware Architecture]] — a concrete multiprocess embedded stack.

---

## 📚 External Resources

- [systemctl manual](https://www.freedesktop.org/software/systemd/man/latest/systemctl.html).
- [Unit search paths, drop-ins and dependencies](https://www.freedesktop.org/software/systemd/man/latest/systemd.unit.html).
- [Service types and execution](https://www.freedesktop.org/software/systemd/man/latest/systemd.service.html).
- [Upstream drop-in ordering discussion](https://github.com/systemd/systemd/issues/13198) — same-name precedence versus lexicographic merging.
- [Upstream NEWS](https://github.com/systemd/systemd/blob/main/NEWS) — release-specific/in-development changes; match the installed release.
