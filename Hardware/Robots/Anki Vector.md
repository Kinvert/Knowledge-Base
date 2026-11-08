# Anki Vector

**Anki Vector** is a small tracked social robot with an embedded Linux head computer, a separate body controller, a camera, an IMU and an interactive lift/cube ecosystem. It connects robotics perception and behavior with local-backend software, authenticated SDK access and several distinct firmware-development layers.

This is the hub for Vector notes. Start with the capability needed—not with flashing—and follow the relevant branch below.

---

## 🧭 Topic Map

| Area | Notes | Main question |
|---|---|---|
| Physical platform | [[Vector Hardware]] | Which computer/controller/sensor owns the work? |
| Software stack | [[Vector Firmware Architecture]] | How do onboard processes and messages connect? |
| Developer access | [[Vector OSKR]], [[Vector ABOOT]] | What is being unlocked, and what can become permanent? |
| Storage and recovery | [[Vector Partitions and Recovery]] | What do slots protect, and what is per-device data? |
| Community OS | [[Vector WireOS]] | Which repository/image/toolchain is appropriate? |
| Local backend | [[Vector WirePod]], [[Vector Escape Pod]] | Which external service and compatible firmware are involved? |
| PC applications | [[Vector SDK]] | What can an unchanged robot already expose? |
| Richer measurements | [[Vector IMU Telemetry]] | Which history/timing information is missing from the public API? |
| Builds and experiments | [[Vector Firmware Development]] | How do application deployment and temporary trials differ? |
| Alternative hardware stack | [[Vector Gobot]] | What changes when stock hardware owners are replaced? |

---

## ⚙️ Compare the Development Layers

| Layer | Typical change | Runs where? | Needs arbitrary onboard execution? |
|---|---|---|---|
| PC SDK client | Record existing state or request supported actions | Workstation | No |
| WirePod backend | Local voice/intent services | External host | Not merely for running the backend |
| Onboard application | Export extra engine state | Robot head/Linux | Yes |
| OS image | Kernel, platform libraries, packaged services | Robot head storage | Requires the appropriate image installation/access path |
| Bootloader/recovery | Production-to-developer unlock | Boot/update boundary | Separate, higher-risk prerequisite |
| Body-controller firmware | Low-level body I/O changes | STM32 MCU | Separate firmware/tooling problem |
| Cube firmware | BLE accessory behavior | Cube | Separate firmware target |

An SDK credential is not a root shell. A local backend is not an OS image. A firmware name containing `ep` is not proof of an unlocked boot chain.

---

## 🧠 Hardware and Software Boundaries

The headboard contains a Qualcomm APQ8009, 4 GB eMMC and **512 MB RAM**. The head's Bosch BMI160 supplies inertial measurements; it is not simply another sensor behind the STM32 body controller.

Linux applications divide behavior/perception, lower-level control, animation and authenticated SDK communications. Packaging differs between original firmware and community WireOS, including gateway integration into `vic-cloud` in the inspected source.

Vector 1.0 and 2.0 differ in camera/display handling and battery access. Identify the generation and installed image before assuming interchangeability or compatibility.

---

## 📡 What the SDK Already Provides

Existing public state includes acceleration, gyro, pose, head angle, wheel speeds, proximity, touch and status information. Reading available IMU values does **not** require a firmware reflash.

Important limitations:

- Property polling can return the same cached state repeatedly.
- Public inertial vectors use the moving **head frame**.
- Acceleration is documented in mm/s²; gyro in rad/s.
- The public image timestamp is not the inertial sample timestamp.
- Internal gyro history is richer than the public state but is not a full raw six-axis sample history.
- Raw encoder counts, individual cliff measurements and torque/grip-force streams must not be assumed from related public fields.

Use the SDK note for the existing interface and the telemetry note for source tracing and extension boundaries.

---

## 🛠️ Practical Development Order

1. **Baseline without firmware changes:** finite stationary SDK collection; document units, frames, receipt times and missing/repeated data.
2. **Inspect and build offline:** identify source/base-image compatibility and build unchanged applications before modifying them.
3. **Establish developer access separately:** review unlock permanence, image provenance and recovery uncertainty before crossing that boundary.
4. **Try a bounded application experiment:** export information already at the engine, retaining stock control ownership where possible.
5. **Expand only when necessary:** add public API support or acquisition history after measurement semantics and resource impact are understood.

A source-visible design is not automatically a feature available on a production robot. A successful simulator run does not prove physical timing or safe flashing.

---

## 🛡️ Safety and Reversibility

- An application deploy can avoid a full OTA while still overwriting persistent `/anki` files.
- A runtime-only override can disappear on reboot while logs/settings written elsewhere remain.
- A/B system slots are not complete independent backups of bootloader, factory material and device identity.
- Developer unlocking can be permanent; ordinary application teardown does not reverse it.
- Preserve calibration, keys and identity separately for each robot.
- Keep normal physical reactions, bounded resource use and authenticated access.
- With multiple robots, keep a known-good reference rather than modifying all units together.

No guaranteed consumer hard-brick recovery path follows merely from UART pads, a downloaded image or a generic Qualcomm tool.

---

## 📚 Reusable Foundations

| Concept | Existing/general notes | Vector connection |
|---|---|---|
| Embedded image construction | [[Yocto Project]] | Full OS builds rather than ordinary client changes |
| Process supervision | [[systemd]] | Effective units, overrides and experiment teardown |
| Volatile staging | [[tmpfs]] | Memory/swap limits and mount execution policy |
| File deployment | [[rsync]] | Deletion, interruption and persistent replacement |
| Hardware interfaces | [[HAL]], [[SPI]], [[UART]], [[BLE]] | Sensor access, controller communication and cube connectivity |
| Binary compatibility | [[C ABI]], [[GLIBC]], [[Dynamic Linking]] | ARM calling convention and library matching |
| Messages and APIs | [[IPC]], [[Protobuf]], [[gRPC]] | Internal communication versus public authenticated interfaces |
| Perception and state | [[IMU]], [[Encoder]], [[SLAM]] | Measurements, motion estimates and mapping |

These general notes do not imply Vector implements every protocol feature or Linux mechanism they describe.

---

## 🔎 Evidence and Scope

Detailed application-source findings in this note set are pinned to `wire-os-victor` commit `505eb84cfaa6ec01c075730f4be855e93713bb59`. Historical Anki SDK sources describe the original public API; current community documentation describes its own workflows and can change independently.

The notes distinguish source findings, proposed extensions and target-dependent unknowns. They are not a record of a successful unlock, firmware build, live robot test or verified rollback. Do not import a temporary research guide's local credentials, robot-specific observations or operational file paths into a general procedure.

Source visibility also does not settle redistribution rights for inherited Qualcomm/Anki/DDL components.

---

## 🌐 External Resources

- [Community Vector documentation](https://os-vector.github.io/vector-docs/) — hardware, architecture and firmware categories.
- [Anki Python SDK source](https://github.com/anki/vector-python-sdk) — public API and generated message definitions.
- [WireOS full OS repository](https://github.com/os-vector/wire-os) and [pinned application source](https://github.com/os-vector/wire-os-victor/tree/505eb84cfaa6ec01c075730f4be855e93713bb59).
- [WirePod project](https://github.com/kercre123/wire-pod) — external local-backend implementation.
- [Unlock maintainer's current status/corrections](https://github.com/kercre123/unlocking-vector) — distinguish historical constraints from current workflows.
