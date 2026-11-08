# Vector WireOS

**Vector WireOS** is a community-maintained embedded Linux OS/application base for [[Anki Vector]]. It is distinct from the external WirePod backend and from a PC-side SDK package.

Its repositories separate construction of a complete OTA from the application code that supplies much of Vector's behavior. That separation allows many developer changes without rebuilding the entire OS.

---

## 📦 Repositories and Outputs

| Item | Role | Typical use |
|---|---|---|
| `wire-os` | Full OS, applications and OTA composition | Kernel/platform changes or distributable images |
| `wire-os-victor` | `/anki` applications, libraries and assets | Most behavior, telemetry and API development |
| `EXTERNALS` submodule | Application dependencies | Matching dependency/build environment |
| Yocto layers/BSP | Embedded OS recipes and board support | Platform packaging and cross-build configuration |
| Base OTA | Installed OS/runtime environment | Compatible foundation for application-only deployment |
| Standalone application artifacts | Target executables and dependencies | Developer iteration on an accessible robot |

The pinned application README explicitly recommends standalone application builds against a good base OTA. An arbitrary production image is not automatically an appropriate development base.

---

## 🧭 Image Categories

The community installation page distinguishes production robots unlocked with the generic production-unlock OTA from robots unlocked through the official OSKR program.

- **`dev`:** documented for the generic production-unlock path.
- **`oskr`:** documented for the official OSKR path.
- **Prototype/development states:** may have their own instructions.

Use the category matching the actual target. Do not infer it from an `ep` suffix, working SDK credentials or the existence of an SSH key.

[[Vector OSKR]] explains developer-access categories; [[Vector Partitions and Recovery]] explains why installing an image is not the same as undoing a bootloader transition.

---

## ⚙️ Platform Changes and Compatibility

Current upstream documentation describes modernization of the Yocto userspace, toolchain, libraries, camera processing and selected onboard components. It also describes a move from **softfp to hardfp** calling conventions.

Exact compiler/library versions change; the project README and community page can disagree. Match source, submodules, toolchain and installed base rather than copying a version list as a permanent compatibility contract.

A newer userspace does not imply a current upstream kernel, complete hardening or absence of external network traffic. Developer web services and automatic update behavior require separate inspection.

---

## 🛠️ Application Build Versus Full OS Build

| Requirement | Prefer | Reason |
|---|---|---|
| Add a behavior or engine diagnostic | Application repository | Required logic is normally above the platform layer |
| Export existing internal gyro history | Engine/application changes | Engine already receives the data |
| Change public state/RPC routing | Application producer, protobuf and gateway | End-to-end interface changes are needed |
| Change kernel/platform libraries | Full OS repository | Base runtime environment changes |
| Package a complete customized OTA | Full OS repository | Image composition is the deliverable |
| Log existing public state | PC SDK client | No onboard rebuild required |

Read [[Vector Firmware Development]] before using convenience deployment scripts. Application deployment can overwrite persistent `/anki` content despite avoiding an OTA.

---

## 🧰 Build Environment Boundaries

The repositories document Linux/Docker and selected native host workflows, with different requirements for the application and full OS builds. Full-image builds require significantly more disk, memory and platform tooling than a small PC client.

Preserve a working SDK environment. Host dependency installation, Docker permission changes and emulator/binfmt configuration are separate workstation changes, not harmless consequences of reading build instructions.

An unchanged baseline build should precede custom code. Simulator success cannot establish real hardware timing, driver correctness, image acceptance or recovery.

---

## 🛡️ Updates and Development Services

The architecture documentation describes scheduled update checks using an environment-configured URL. WireOS also documents development/configuration web services and shared-root-key access.

Inventory the chosen image's actual services, accepted keys, listeners and update policy. Keep network exposure narrow and use authenticated channels for telemetry. Do not assume a method exists under the same service name on every original/community image.

No running robot image was inspected for this note.

---

## ⚖️ Source Provenance

The project states that the maintainer's work is GPLv3 while preceding Qualcomm/Anki/DDL work includes material the maintainer cannot license.

A source repository or GPL badge therefore does not resolve rights to every inherited component. Redistribution of modified firmware or a product needs its own provenance review.

---

## 🔗 Related Notes

- [[Vector Firmware Architecture]] — processes, IPC and source boundaries.
- [[Vector SDK]] — existing public interfaces.
- [[Vector WirePod]] — external backend, not this OS.
- [[Yocto Project]] — image build concepts.
- [[C ABI]] and [[GLIBC]] — target runtime compatibility.

---

## 📚 External Resources

- [WireOS full OS README](https://github.com/os-vector/wire-os/blob/main/README.md) — moving build/platform documentation.
- [Pinned application README](https://github.com/os-vector/wire-os-victor/blob/505eb84cfaa6ec01c075730f4be855e93713bb59/README.md) and [dependency submodule](https://github.com/os-vector/wire-os-victor/blob/505eb84cfaa6ec01c075730f4be855e93713bb59/.gitmodules).
- [Community installation categories](https://os-vector.github.io/vector-docs/2.-Available-CFW/1-WireOS.html).
- [Community architecture and updates](https://os-vector.github.io/vector-docs/3.-Software-Architecture/architecture.html).
- [Licensing/provenance statement](https://github.com/os-vector/wire-os/blob/main/LICENSE-README).
