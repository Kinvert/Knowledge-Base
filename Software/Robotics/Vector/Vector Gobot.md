# Vector Gobot

**Vector Gobot** (`vector-gobot`) is a Go/C hardware-access library for [[Anki Vector]], wrapping the lower-level `poc.vic-hack` work. It is an alternative way to own hardware interfaces, not simply a new PC SDK talking to an unchanged production robot over Wi-Fi.

The upstream README documents developer/SSH access, onboard libraries and stopping the normal robot target before running its programs.

---

## ⚙️ Modules and Capabilities

| Module / path | Documented role | Important qualification |
|---|---|---|
| `vbody` | Body/spine frames, motors, encoders, cliffs and power data | Replaces/interacts with the body communication owner |
| `vimu` | IMU access | README describes 10 ms polling, not verified PC delivery rate |
| `vcam` | Camera frames through the camera service | Includes service/configuration implications |
| `vjpeg` | Raw-camera conversion/compression with turbojpeg | Requires an additional library build |
| `vscreen` | Display control | Model/format compatibility still matters |
| Microphone/body fields | Body microphone samples and related I/O | Convenient complete audio interface remains listed as unfinished |

The README also lists unfinished ToF conversion, speaker work, calibration/image normalization work and memory-leak cleanup. Treat its feature list as a project status report, not a production-readiness or safety certification.

---

## 🧠 What the Body Frame Contains

The inspected `pkg/vbody/spine.go` defines a `DataFrame` containing:

- A sequence field and four cliff values.
- Four motor entries with position/delta/time fields.
- Battery/charger voltages, body temperature and touch/button state.
- Microphone data.
- Several proximity range/quality/calibration-related fields.

These fields are useful evidence of lower-level information absent from the public SDK. Their names alone do not establish complete physical units, calibration quality, timestamp reconstruction or a usable distance estimate.

In particular, the README says ToF data can be read but conversion to a clean millimeter result is unfinished.

---

## 🔄 Hardware Ownership and Safety

The documented install path stops `anki-robot.target`. That changes the normal behavior/control stack and is fundamentally different from adding a passive export inside an existing engine owner.

Do not run competing owners of body serial interfaces, device files or exclusive sockets. A side process is not passive merely because the application developer only intends to print measurements.

The source has a `ReadOnly` flag affecting body communication behavior, but its existence does not prove that replacing stock services preserves stock fall/cliff/low-battery reactions. Initialization, device configuration and application writes require separate review.

For a first telemetry experiment, [[Vector Firmware Development]] favors observing data already received by the stock engine where sufficient.

---

## 🧩 Build and Runtime Compatibility

The README's example uses cgo, `GOARCH=arm`, `GOARM=7` and an `arm-linux-gnueabi` toolchain. The Go program and C/C++ libraries must use a compatible toolchain and target ABI.

Modern [[Vector WireOS]] documents hardfp calling conventions, while older Vector environments used softfp. Do not assume an old example binary or library matches a modern image.

The documented runtime library search can use `LD_LIBRARY_PATH`. This does not resolve libc, calling-convention or API mismatches by itself.

Copying libraries into `/lib` or changing `/data` execution policy would be persistent/privileged target changes, not automatically a temporary experiment. This note deliberately does not repeat those operations as a recommended deployment recipe.

---

## 🖥️ Native Builds Are Not a Stock LAN SDK

The project offers an x86/native build option. Its README's native body example assumes a serial interface such as `/dev/ttyUSB0` presented under the expected device name.

That is hardware/serial access, not proof that the normal production SDK remotely exposes every Gobot interface. Decide where the program runs and which physical/service interface it owns before comparing it to [[Vector SDK]].

---

## 🧪 Evaluate Before Adoption

- Identify the actual source revision, module status and robot generation.
- Match C/C++ libraries, Go/cgo build settings and installed image ABI.
- Inspect initialization, calibration and device writes, not only the readout loop.
- Define control/safety responsibilities while stock services are stopped.
- Bound logging, CPU and memory use; validate measured rates and drops.
- Establish teardown and restoration before a supervised stationary trial.

The project is useful as a decoding/protocol reference even when replacing the normal stack is not justified. No deployment or safety test of Gobot is asserted here.

---

## 🔗 Related Notes

- [[Vector Hardware]] — controllers and sensor placement.
- [[Vector Firmware Architecture]] — normal process ownership.
- [[Vector IMU Telemetry]] — engine-side history without replacing the whole stack.
- [[C ABI]] and [[Dynamic Linking]] — native library compatibility.
- [[UART]] and [[HAL]] — lower-level access boundaries.

---

## 📚 External Resources

- [vector-gobot README/source](https://github.com/os-vector/vector-gobot) — build, install, feature status and native-serial assumptions.
- [Body frame and communication implementation](https://github.com/os-vector/vector-gobot/blob/main/pkg/vbody/spine.go).
- [WireOS ABI notes](https://github.com/os-vector/wire-os/blob/main/README.md).

These are moving project sources, not a pinned compatibility guarantee for the selected robot image.
