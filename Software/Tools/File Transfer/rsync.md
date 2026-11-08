# rsync

**rsync** copies and synchronizes files locally or between machines, with controls for metadata, selection, deletion and partial transfers. Its delta-transfer algorithm can reduce network traffic, but synchronization is not automatically a backup, atomic deployment or rollback mechanism.

It is especially important to inspect deletion scope and interruption behavior before using rsync to deploy embedded applications.

---

## 🔌 Transport Modes

| Mode | Syntax | Security / ownership boundary |
|---|---|---|
| Local copy | `rsync OPTIONS source/ destination/` | Local process permissions |
| Remote-shell push | `rsync OPTIONS source/ user@host:destination/` | Normally SSH; remote rsync runs as that user |
| Remote-shell pull | `rsync OPTIONS user@host:source/ destination/` | Same remote-shell authentication boundary |
| Direct daemon | `rsync OPTIONS source/ host::module/path/` | Daemon module policy; not encrypted by default |
| Daemon URL | `rsync OPTIONS source/ rsync://host/module/path/` | Same direct-daemon distinction |
| Daemon features through remote shell | Specialized `--rsh` workflow | Different from plain direct-daemon TCP |

Remote copying normally requires rsync at both ends. Ordinary rsync does not copy directly between two different remote hosts in one invocation.

A checksum detects data differences/integrity conditions; it is not peer authentication or encryption. Prefer [[SSH]] or an explicitly verified secure transport on untrusted networks.

---

## 📁 Source Slashes and Selection

With an existing destination directory:

- `source/` means copy the **contents** of that directory into the destination.
- `source` without the trailing slash normally includes the directory name as another level.
- The source directory's attributes can still affect the corresponding destination directory.
- Shell wildcards are expanded before rsync sees them; they can change the transfer root and deletion behavior.

Avoid a wildcard when the intent is a whole-tree mirror. Quote paths and verify the intended source root, destination root and filters before adding deletion.

---

## 🔍 Preview Before Writing

Example preview for operator-selected paths; no files are copied by this command:

```bash
rsync -ani --delete -- source/ destination/
```

- `-a`: archive behavior, equivalent to `-rlptgoD`.
- `-n` / `--dry-run`: describe planned changes without performing them.
- `-i` / `--itemize-changes`: show what differs, including planned deletions.
- `--`: finish options before path arguments.

A dry run is not a transaction: data, permissions or the remote tree can change before the real transfer. Read the deletion rows rather than checking only the exit status.

---

## 🧩 Important Options and Tradeoffs

| Option | Purpose | Gotcha |
|---|---|---|
| `--delete` | Remove destination entries absent from the selected source | A wrong/empty source can make a destructive mirror |
| `--delete-before` | Delete before updates | Failure can remove old files before replacements arrive |
| `--inplace` | Write directly into destination files | Existing readers can observe mixed data; interruption leaves partial modifications |
| `--partial` | Keep incomplete transfer data | Not proof that the final pathname holds a complete usable file |
| `--partial-dir=DIR` | Store reusable partial data separately | Protect directory permissions; conflicts with `--inplace` |
| `--delay-updates` | Delay file placement until transfer end | Extra storage; still not one atomic tree switch; conflicts with `--inplace` |
| `--checksum` | Select changed files using content checksums | Extra source/destination reads, not a free integrity/security mode |
| `--timeout=SECONDS` | Limit I/O inactivity | Not a universal total wall-clock limit |
| `--bwlimit=RATE` | Limit transfer rate | Does not bound all CPU, memory or filesystem impact |
| `--fsync` | Request synchronization of written files | Not an atomic application release or full database snapshot |

`--inplace` implies partial retention and conflicts with `--partial-dir` and `--delay-updates`. Do not combine flags merely because each sounds safer independently.

---

## 🔄 Interruption and Consistency

Normal file replacement commonly stages a temporary file then renames it into place. That can improve individual-file visibility compared with `--inplace`, but the entire directory tree is still updated over time.

`--delay-updates` narrows the final placement window rather than making all files one atomic operation. Filesystem durability, metadata changes, symlinks, deletions and applications reading related files have their own consistency implications.

For an application release requiring a consistent tree, use a separately designed staging/cutover/restoration process. Live database copying needs the database's snapshot/backup mechanism rather than a blind filesystem mirror.

A restart or read-only remount does not restore bytes already overwritten. A deployment script's normal cleanup does not prove that every failure path leaves usable applications.

---

## 🗂️ Metadata and Backup Limits

Archive mode does **not** include everything: hard links, ACLs and extended attributes need options such as `-H`, `-A` and `-X` where supported. Some ownership/device metadata needs privileges or appropriate destination support.

Preserve numeric identities deliberately when relevant; user/group name mapping can differ across hosts. Filesystem capabilities and case sensitivity also affect correctness and safety.

A mirror with deletion can faithfully reproduce accidental removal. Versioned backups, retention and proven restoration are separate requirements.

---

## 🛡️ Daemons, Versions and Troubleshooting

Direct daemon syntax uses module configuration and does not imply TLS or SSH protection. Keep temporary listeners restricted and close them through explicit cleanup.

Check both peers' versions/capabilities. Release fixes can change path safety and interrupted-transfer behavior; the upstream **3.5.2** release notes, for example, document fixes affecting interrupted `--inplace`/`--append` transfers. Do not infer an old embedded binary has those fixes.

For connection/protocol failures, investigate remote executable availability, permissions, storage capacity, daemon logs and unexpected remote-shell output. Upstream recommends itemized changes rather than excessive verbosity and documents diagnostic stderr options.

Exit codes matter: a partial-transfer or vanished-source result is not a complete successful deployment. Inspect what changed before attempting a recovery/retry.

---

## 🤖 Embedded Deployment Connection

The pinned Vector deployment script uses a daemon on port `1873` and `--inplace --delete` to synchronize `/anki`. That avoids an OTA while still changing persistent executables/libraries/assets.

[[Vector Firmware Development]] explains why this is not a promised temporary overlay. A more convenient wrapper or successful transfer does not establish source/base-image ABI compatibility.

---

## 🔗 Related Notes

- [[scp]] — simpler secure-copy workflow, with different synchronization semantics.
- [[File Transfer Protocols]] — transport comparisons.
- [[Dynamic Linking]] — replacing library files versus running process mappings.
- [[systemd]] — stopping/restoring services is separate from restoring file content.
- [[tmpfs]] — volatile staging with resource and execution-policy limits.

---

## 📚 External Resources

- [Official rsync manual](https://download.samba.org/pub/rsync/rsync.1) — flags, selection, interruption and security.
- [Release NEWS](https://download.samba.org/pub/rsync/NEWS) — version-specific fixes and compatibility.
- [Upstream current issues/debugging](https://rsync.samba.org/issues.html) and [project/security updates](https://rsync.samba.org/).
- [Pinned Vector deployment script](https://github.com/os-vector/wire-os-victor/blob/505eb84cfaa6ec01c075730f4be855e93713bb59/project/victor/scripts/deploy.sh).

Local help/version inspection used rsync **3.5.1**, protocol 33. That does not establish the version on any robot or remote peer.
