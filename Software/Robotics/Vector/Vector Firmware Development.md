# Vector Firmware Development

Vector Firmware Development separates PC clients, onboard applications, complete OS images and low-level firmware for [[Anki Vector]]. Most telemetry work should start at the highest layer that already owns the required information.

**No full reflash** and **temporary change** are different properties. Application deployment can overwrite persistent files even when no OTA is installed.

---

## ⚙️ Choose the Smallest Sufficient Layer

| Change | Starting boundary | Separate prerequisite or risk |
|---|---|---|
| Record existing acceleration/gyro | [[Vector SDK]] client | Working SDK credentials; no unlock solely for this |
| Export existing internal gyro history | Engine application and output path | Developer execution access and compatible applications |
| Preserve higher-rate acceleration samples | Robot-side collection and internal transport | New measurement/timing contract; preserve safety logic |
| Change public API | Producer, protobuf, gateway and client | Matching generated code, routing and authentication |
| Change kernel/platform libraries | Full [[Vector WireOS]] image | OS build, target selection and image validation |
| Unlock boot chain | [[Vector OSKR]] / production-unlock path | Potentially permanent bootloader/recovery changes |
| Change body MCU or cube | Separate controller firmware project | Separate tooling, device ownership and recovery investigation |

The application README recommends standalone `wire-os-victor` development on a compatible base OTA. That does not imply arbitrary production firmware permits application replacement.

---

## 🧰 Source and Build Environment

Pin more than the main repository commit:

- Application source and dependency submodules, including `EXTERNALS`.
- Base image, robot generation and developer-access category.
- Compiler, sysroot, libc and generated-code tool versions.
- Build configuration and complete runtime dependencies.
- Artifact hashes and the exact deployment/restoration method.

Build an unchanged baseline before introducing a telemetry modification. The pinned application README documents `./build/build-v.sh` for its Docker-based workflow and `vbuild` after its bare-metal environment setup. These are source-documented entry points, **not builds verified by this note**.

Upstream host setup may alter packages, Docker permissions or emulator configuration. Reading those instructions is not permission to make privileged workstation changes or replace a working SDK environment.

[[Yocto Project]] explains the full-image build layer; [[Docker]] explains build-environment isolation. Containers do not themselves prove reproducibility or target compatibility.

---

## 🧩 Architecture and ABI Compatibility

Vector's application target is ARM Linux, not x86 workstation Linux. Check CPU architecture, dynamic loader, library versions, calling convention and required dependencies together.

Current WireOS documentation describes **hard-float** calling conventions where older Vector environments used **softfp**. These ARM ABI variants are not interchangeable merely because both binaries contain ARM instructions or can use floating-point hardware.

[[C ABI]], [[GLIBC]], [[ELF]] and [[Dynamic Linking]] provide the underlying concepts. Inspecting an artifact is necessary but still does not replace running it on a compatible development environment.

Do not bypass version checks as a substitute for establishing compatibility. The pinned convenience deployment wrapper passes `-b` to bypass a version mismatch even though the underlying script warns against doing so.

---

## 📦 What Standard Deployment Changes

At application commit `505eb84cfaa6ec01c075730f4be855e93713bb59`, the deployment scripts:

1. Default the application installation root to `/anki`.
2. Remount the root filesystem writable when necessary.
3. Stop `anki-robot.target` before transferring application files.
4. Start an rsync service and transfer to its port `1873` module.
5. Use `--inplace --delete`; force mode can also delete before transfer.
6. Stop the rsync service on the normal transfer path and perform filesystem cleanup.
7. Have the convenience wrapper start the robot target afterward.

Consequences:

- Existing executables, libraries and assets can be overwritten or removed.
- Interrupted deployment can leave a mixed or incomplete application tree.
- Remounting read-only afterward does not restore old bytes.
- A reboot does not undo persistent application replacement.
- Normal-path cleanup is not proof that every interruption closes every service or restores application consistency.

Read [[rsync]] before interpreting a deployment command as atomic synchronization or rollback.

---

## ⏳ Compare Experiment Lifetimes

| Method | What ends automatically | What can remain | Appropriate interpretation |
|---|---|---|---|
| PC SDK recorder | Client process when closed | PC recordings/settings | Least invasive baseline |
| Helper in persistent onboard storage | Process when stopped | Binary, logs, side effects | Process-temporary only |
| Binary in verified executable tmpfs | Staged bytes after unmount/reboot | Writes outside that mount | Storage-temporary, not side-effect-free |
| Runtime-only service override | Override after removal/reboot | Application outputs and other persistent changes | Configuration-temporary if target semantics verified |
| Standard `/anki` deployment | Running process can restart | Replaced persistent application tree | Persistent deployment without full OTA |
| Developer unlock | Not undone by stopping applications | Bootloader/developer state | Separate permanent-risk decision |

Do not assume `/tmp`, `/run` or `/data` is executable, volatile or sufficiently large on every image.

---

## 🧪 Proposed Runtime-Only Application Trial

**Design proposal, not a tested Vector command recipe:**

1. Inspect the target's actual service unit, mount options, security policy, ABI and available memory.
2. Leave known-good `/anki` binaries and persistent unit configuration untouched.
3. Stage only the matching experimental executable/dependencies in a bounded executable [[tmpfs]].
4. Apply a narrowly owned runtime-only [[systemd]] override, preserving original arguments, user, environment, working directory and dependencies.
5. Stop the original owner before starting the replacement; never run two hardware/socket owners accidentally.
6. Arrange an independently verified restoration action, not one dependent solely on an uninterrupted PC connection.
7. Remove only experiment-owned configuration, reload definitions and restart the known-good application.
8. Verify the effective executable/unit and repeat baseline SDK/camera observations.

Persistent `/etc` configuration can override corresponding runtime configuration. `daemon-reload` and restarting an application are different operations. A generic service example from a modern systemd manual is not proof that an older robot image supports it unchanged.

Immediate teardown and ordinary reboot rollback are separate acceptance checks. Neither reverses an earlier unlock.

---

## 🛡️ Safety, Access and Resource Budgets

A telemetry observer should not modify sensor configuration, motor commands, control ownership or safety priorities. Nevertheless, it can consume CPU, memory, storage and IPC bandwidth.

- Use preallocated bounded producer storage; never wait for disk/network on a sensing/control path.
- Define queue-full behavior and observable drops.
- Keep finite captures small and logs outside persistent storage unless explicitly intended.
- Preserve normal SDK physical reactions; do not suppress safety behavior for cleaner experiments.
- Authenticate retrieval; do not add an unauthenticated LAN debug listener.
- Review accepted SSH keys and pin host identity. A community-shared root key is not private owner authentication.
- Inspect update checks and development web services rather than assume the whole image is hardened.

A RAM-staged engine can still write normal settings or caches elsewhere. Inventory those side effects before calling the experiment reversible.

---

## ✅ Validation and Rollback Record

Start with a finite stationary SDK baseline. Offline, build the unchanged applications and exercise the proposed decoder's units, invalid timestamps, gaps and finite shutdown. A simulator cannot establish physical IMU timing, driver behavior or target recovery.

On a separately approved developer robot, compare logging disabled/enabled for CPU, memory, dropped data, camera delivery and ordinary responsiveness. A disconnected or slow consumer must stop/drop telemetry rather than stall the producer.

Record target/model/image, source/submodule/toolchain versions, artifact hashes, mounts, overrides, processes, output paths, persistent changes and exact restoration steps. A backup is not a proven unbrick mechanism until the applicable restoration path is established.

With two robots, retain one known-good reference and use the other only after a baseline and explicit acceptance of unlock risk. Keep factory calibration, credentials and device identities separate.

---

## ⚖️ Source Provenance

The WireOS licensing note distinguishes the maintainer's GPLv3 work from earlier Qualcomm/Anki/DDL material the maintainer cannot license. Source availability does not settle redistribution rights for the entire inherited firmware, generated package or modified binary.

Treat redistribution/product use as a separate provenance review, not permission inferred from a repository license badge.

---

## 🔗 Related Notes

- [[Vector Firmware Architecture]] — application and message source map.
- [[Vector IMU Telemetry]] — a bounded engine-side export proposal.
- [[Vector Partitions and Recovery]] — recovery scope and per-device data.
- [[Vector Gobot]] — alternative stack with different ownership implications.
- [[Robot Policy Deployment]] — general real-robot deployment boundaries.

---

## 📚 External Resources

- [Pinned application README](https://github.com/os-vector/wire-os-victor/blob/505eb84cfaa6ec01c075730f4be855e93713bb59/README.md) and [dependency submodule](https://github.com/os-vector/wire-os-victor/blob/505eb84cfaa6ec01c075730f4be855e93713bb59/.gitmodules).
- [Pinned deploy script](https://github.com/os-vector/wire-os-victor/blob/505eb84cfaa6ec01c075730f4be855e93713bb59/project/victor/scripts/deploy.sh) and [convenience wrapper](https://github.com/os-vector/wire-os-victor/blob/505eb84cfaa6ec01c075730f4be855e93713bb59/build/deploy-v.sh).
- [WireOS full-image build and ABI notes](https://github.com/os-vector/wire-os/blob/main/README.md) — moving upstream documentation; match the selected image/revision.
- [WireOS licensing/provenance statement](https://github.com/os-vector/wire-os/blob/main/LICENSE-README).
- [systemctl manual](https://www.freedesktop.org/software/systemd/man/latest/systemctl.html), [unit precedence](https://www.freedesktop.org/software/systemd/man/latest/systemd.unit.html) and [tmpfs manual](https://man7.org/linux/man-pages/man5/tmpfs.5.html) — general Linux mechanisms, not target verification.
