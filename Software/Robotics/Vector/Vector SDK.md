# Vector SDK

**Vector SDK** is the authenticated public interface used by PC-side applications to communicate with [[Anki Vector]]. Changing a client can improve recording and interpretation without replacing onboard firmware, but it cannot add measurements the robot does not export.

The historical Anki Python SDK and the pinned community firmware sources describe an interface surface—not a guarantee that every firmware/SDK fork has identical behavior or setup requirements.

---

## ⚙️ Existing State Surface

| Information | Public interface finding | Important limitation |
|---|---|---|
| Acceleration | Python `robot.accel`; public `RobotState.accel` | Head-frame vector in mm/s² |
| Angular velocity | Python `robot.gyro`; public `RobotState.gyro` | Head-frame vector in rad/s |
| Head/lift state | Head angle and lift height | Not proof of synchronous capture with every IMU sample |
| Wheel motion | Left/right wheel speeds | Not cumulative raw motor encoder counts |
| Pose/status | Pose, localization and status fields | Estimate/state, not ground-truth position |
| Proximity/touch | Structured public fields | Preserve validity and obstruction information |
| Image timing | `last_image_time_stamp` | Last processed image, not IMU acquisition time |
| Cliff detection | Public status bit | Not the four individual internal cliff readings |

There is already an IMU API. A working connection needs no unlock merely to read the available acceleration and gyro vectors.

---

## 🔐 SDK Authentication Is Not Root Access

The robot-side gateway handles TLS and authorization for its implemented API. SDK identity/configuration and the selected robot's certificate/token requirements belong to that connection path.

Those credentials do not grant arbitrary Linux execution, root SSH, bootloader unlock or permission to overwrite applications.

[[Vector WirePod]] is an external backend with its own setup/authentication workflow. The backend can help with ecosystem setup, but its existence does not collapse SDK, backend and root authentication into one identity.

Keep actual certificates, GUIDs/tokens and private keys out of notes and shared captures. Confirm the selected SDK fork's setup instructions instead of assuming historical cloud activation still applies unchanged.

---

## 🧠 Cached Properties Versus Events

The Python SDK stores current state in properties. A tight loop reading a property can observe the same state repeatedly; it does not request a fresh physical sensor conversion each time.

Prefer state-event-driven recording when identifying distinct delivered updates. Add a host monotonic receipt time at the event boundary, and keep that separate from robot/image timestamps.

An unchanged value is not proof of a duplicate packet: a stationary robot can legitimately report equal readings. Conversely, a changed value alone does not establish its physical sampling time.

Rate and freshness must be measured at the delivered stream under relevant load, not inferred from the client's sleep interval.

---

## 🧭 Frames, Units and Timestamp Limits

The pinned engine getters define X forward through the face, Y left and Z up from the head. The IMU is on the moving head, so these are not automatically chassis-frame vectors.

- Divide acceleration by 1000 when converting mm/s² to m/s².
- Gyro is already in rad/s.
- Preserve the original signal/processing definition when converting units.
- Do not label the image timestamp as an inertial timestamp.
- Do not subtract a host monotonic clock directly from an unrelated robot clock.

The inspected public state has no dedicated IMU-sample timestamp or internal gyro-history array. [[Vector IMU Telemetry]] explains what is available upstream and what an extension would need to preserve.

---

## 🛡️ Behavior Control and Physical Reactions

The official SDK defines normal/default behavior control below mandatory physical reactions. Its override priority is explicitly above those reactions and can ignore hazards such as low battery or tabletop edges.

Use normal priority for ordinary operation; do not choose override priority to make an experiment seem more deterministic. A stationary telemetry recorder needs no motor command path or safety override.

Acquiring behavior control can itself affect idle behavior. Decide whether a recorder actually needs control using the selected SDK's supported configuration, rather than treating a standard interactive example as passive observation.

---

## 🧪 A Useful First Recording

Record a finite stationary interval with:

- Delivered state values and host monotonic receipt times.
- Head angle, acceleration/gyro, relevant status and proximity validity.
- Explicit distinction between repeated client reads and newly delivered events.
- Separate image timestamps and uncertainty about acquisition timing.
- A bounded output size and a clean disconnect.

This establishes a baseline, not a hardware-accurate sample clock. No robot connection or capture is claimed by this note.

---

## 🔌 Extending the Public Interface

A complete extension may require:

1. An onboard producer and bounded storage.
2. Internal messages if the engine lacks the required information.
3. Public protobuf fields/events or RPC definitions.
4. Matching generated bindings and engine/gateway routing.
5. A client-facing API with units, frames, clock, validity and cancellation semantics.

Reuse authentication and verify authorization on the actual route. Do not renumber/reuse protobuf fields, hand-edit generated output or interpret an absent timestamp's zero default as a valid sample.

Wire-format compatibility does not guarantee client understanding, populated values or correct event filtering. A new PC field cannot cause old firmware to produce new information.

---

## 🔗 Related Notes

- [[Vector Firmware Architecture]] — producer, export and gateway boundaries.
- [[Vector Firmware Development]] — when onboard changes are justified.
- [[Vector Escape Pod]] — firmware compatibility versus developer access.
- [[Protobuf]] and [[gRPC]] — serialization and RPC foundations.
- [[Robot Data Collection and Teleoperation]] — recording discipline.
- [[venv]] — isolate client dependencies from firmware build tooling.

---

## 📚 External Resources

- [Official SDK repository](https://github.com/anki/vector-python-sdk).
- [Acceleration/gyro properties](https://github.com/anki/vector-python-sdk/blob/master/anki_vector/robot.py#L465-L492) and [historical public schema](https://github.com/anki/vector-python-sdk/blob/master/anki_vector/messaging/messages.proto).
- [Official control-priority semantics](https://github.com/anki/vector-python-sdk/blob/master/anki_vector/connection.py#L49-L59).
- [Pinned community public schema](https://github.com/os-vector/wire-os-victor/blob/505eb84cfaa6ec01c075730f4be855e93713bb59/tools/protobuf/gateway/public/messages.proto).
- [Pinned head-frame definitions](https://github.com/os-vector/wire-os-victor/blob/505eb84cfaa6ec01c075730f4be855e93713bb59/engine/robot.h#L424-L436) and [public export](https://github.com/os-vector/wire-os-victor/blob/505eb84cfaa6ec01c075730f4be855e93713bb59/engine/robot.cpp#L2272-L2345).
- [Gateway implementation](https://github.com/os-vector/wire-os-victor/blob/505eb84cfaa6ec01c075730f4be855e93713bb59/cloud/cloud/gateway.go).
