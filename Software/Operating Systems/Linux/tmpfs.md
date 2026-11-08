# tmpfs

**tmpfs** is a Linux virtual-memory filesystem whose contents normally reside in RAM and can use swap. It is useful for volatile staging, bounded scratch data and shared-memory backing, but it is not a guarantee that bytes never reach persistent media or that an application experiment leaves no side effects.

A filesystem's actual mount type and policy matter more than a directory name such as `/tmp`, `/run` or `/data`.

---

## ⚙️ Core Semantics

- Files consume memory/swap as data is stored; the filesystem does not allocate its whole size limit in advance.
- `size=` is an upper bound, not a reservation of physical RAM.
- Unmounting a tmpfs discards its contents; ordinary reboot also loses that mount's volatile contents.
- Swap may contain tmpfs pages unless the system/mount policy prevents it.
- Remounting can change the size limit without discarding existing data, subject to valid limits.
- Inode limits can exhaust before the byte limit when creating many small files.

The default byte limit is commonly half physical RAM when neither `size` nor `nr_blocks` is specified. Multiple mounts and processes still compete for the same system resources.

---

## 📊 Compare Storage Mechanisms

| Mechanism | Backing / persistence | Capacity control | Typical use |
|---|---|---|---|
| tmpfs | RAM and potentially swap; volatile mount contents | Size/inode limits | Scratch files and temporary application staging |
| ramfs | RAM; volatile | No comparable built-in size bound | Specialized use where unbounded growth is controlled elsewhere |
| Disk filesystem | Persistent block storage | Filesystem/device capacity | Durable application data |
| Bind mount | Existing backing filesystem exposed elsewhere | Inherits underlying storage constraints | Change path visibility, not persistence |
| Overlay filesystem | Combined lower/upper layers | Depends on backing upper/work filesystems | Layered view; persistence depends on the upper layer |
| procfs | Kernel-generated process/system view | Not ordinary file storage | Inspection, not general binary/log staging |

A bind mount of persistent storage is still persistent. An overlay becomes volatile only if its relevant writable backing/lifetime actually makes it so.

---

## 🔍 Inspect Actual Mounts

Read-only examples:

```bash
findmnt --target /tmp --output TARGET,FSTYPE,OPTIONS
findmnt --target /run --output TARGET,FSTYPE,OPTIONS
findmnt --target /data --output TARGET,FSTYPE,OPTIONS
```

`--target` finds the filesystem containing a path; a nonexistent path or unavailable mount can fail. A result pointing at the root filesystem is useful evidence that the directory is not a separate tmpfs mount.

Inspect the namespace relevant to the process as well as the host. Containers and service-private mount namespaces can see different mounts at the same path.

`df` reports filesystem space; memory/swap pressure and aggregate shared-memory accounting need separate observation. Do not equate a large size ceiling with available free RAM.

---

## 🔐 Permissions and Execution Policy

Mount options such as `noexec`, `nosuid`, `nodev`, ownership and mode restrict uses independently of whether the storage is tmpfs.

- Copying a binary to a `noexec` staging mount does not establish that it can run there.
- A readable script may still be read by an interpreter; `noexec` is not a complete untrusted-code sandbox.
- A shared writable directory is inappropriate for private keys or trusted executable staging without deliberate ownership/permissions.
- Dynamic loaders, libraries and process sandboxing can add constraints beyond the executable's path.

Changing mount policy is a separate privileged/security decision. This note's inspection examples do not mount or remount anything.

---

## 🧠 Swap and Kernel-Version Caveats

The tmpfs manual documents `noswap` as a Linux **6.4+** mount option. Do not copy that option into an older embedded kernel's workflow and assume it is supported.

A system without usable swap behaves differently under memory pressure from one with swap. Even with volatile filesystem semantics, swapping and crash dumps can affect sensitive-data persistence assumptions.

The kernel must support tmpfs for user-created mounts. POSIX shared-memory backing at `/dev/shm` and internal shared-memory mechanisms are related, but are not interchangeable with every arbitrary tmpfs staging directory.

---

## 🧪 Temporary Application Staging

A narrowly scoped application experiment should establish:

1. The path is actually tmpfs in the application's namespace.
2. It permits the intended execution/access under the target security policy.
3. The executable and required dependencies fit within a deliberate budget.
4. Other processes retain sufficient memory and swap headroom.
5. Telemetry output is bounded and intended writes outside the mount are recorded.
6. Teardown stops the experiment and restores the known-good service configuration.

[[Vector Firmware Development]] uses this as a proposed mechanism, not a universally tested recipe for every robot image. Copying a complete oversized application tree into a small robot's RAM can defeat the experiment's safety goal.

---

## ⚠️ What Volatility Does Not Undo

A tmpfs-staged program can still:

- Write persistent logs, settings, credentials or databases elsewhere.
- Send network requests or issue hardware commands.
- Change devices or control state while running.
- Consume memory/CPU and disrupt ordinary work.

Reboot can discard staged bytes while leaving those effects. A bootloader unlock is also unrelated to tmpfs lifetime.

Volatile storage, runtime-only service configuration and behaviorally reversible application changes are three different properties.

---

## 🔗 Related Notes

- [[systemd]] — runtime overrides and service restoration.
- [[Bubblewrap]] — tmpfs/bind mounts inside a restricted process view.
- [[Docker Volumes]] — persistent volumes versus volatile mounts.
- [[procfs]] — inspect process/system state.
- [[shmget]] — a related shared-memory API.
- [[Vector Partitions and Recovery]] — persistent robot data and rollback limits.

---

## 📚 External Resources

- [Linux tmpfs manual](https://man7.org/linux/man-pages/man5/tmpfs.5.html) — size, swap, lifetime and versioned mount options.
- [Linux kernel tmpfs documentation](https://docs.kernel.org/filesystems/tmpfs.html).
- [findmnt manual](https://man7.org/linux/man-pages/man8/findmnt.8.html) — inspect the actual filesystem rather than assume a path's backing.

Local help/version inspection used util-linux `findmnt` **2.39.3**. No new mount or target execution was performed to establish this note.
