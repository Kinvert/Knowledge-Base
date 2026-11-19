# ArduPlane

`ArduPlane` is the fixed-wing vehicle branch in the `ArduPilot` family.  
It is software, not a physical board.

---

## What ArduPlane is (and is not)

- It is the fixed-wing/autonomous fixed-wing controller model inside ArduPilot.
- It is not a flight-controller model name.
- It is used for planes, tailsitters, flying wings, and some VTOL modes where fixed-wing guidance logic is primary.
- Companion systems connect to the flight controller via MAVLink; they generally do not replace the FC as the primary flight authority.

---

## When you should use ArduPlane

- You have a vehicle whose primary model is fixed-wing / delta / lifting-body / tailsitter.
- You need mission modes (`AUTO`, `FBWA`, `GUIDED`, `RTL`, `LOITER` variants in this branch).
- You need strong mission tooling and vehicle logging from the same ecosystem as ArduPilot GCS tools.
- You need a mature autotune and failsafe behavior with a broad community body of flight examples.

ArduPlane is usually not the default choice for:

- pure multicopter flight stacks (that is `ArduCopter` territory),
- submersible/boat-only logic,
- pure research stacks where you want ROS-only control loops with no flight-controller abstraction.

---

## Recommended radio-to-telemetry path for this stack

The pattern you asked about is valid:

`ELRS receiver --CRSF UART--> ArduPlane FC --> MAVLink UART --> ESP32-S3`

- the FC performs RF/RC decoding,
- the FC publishes normalized channels over MAVLink (`RC_CHANNELS`/`RC_CHANNELS_RAW` style message path),
- the ESP32-S3 consumes those messages for companion tasks, logging, and higher-level behavior.

This is consistent with ArduPilot RC architecture and avoids having the ESP32-S3 directly decode RF in this path.

Reference: ArduPilot RC systems and MAVLink compatibility notes.

---

## Fixed-wing controller families that commonly run ArduPlane well

| Board | Why it is common with ArduPlane | Typical caveat |
|---|---|---|
| [[Matek H743-Wing]] | Dense `UART/I2C/CAN/PWM` options for ArduPilot fixed-wing features | no onboard compass/GPS; you add nav modules externally |
| [[Holybro Pixhawk 6C Mini]] | H743-class compute in very compact footprint | tighter thermal and wiring clearance than larger 6C boards |
| [[Holybro Pixhawk 6C]] | Broad PX4/ArduPilot compatibility and solid stack tooling | reduced expansion margin compared with 6X |
| [[Holybro Pixhawk 6X]] | Higher I/O and more space than 6C class | larger form factor and part count |
| [[CUAV X7]] | industrial-style H7-class stack with strong mission stack maturity | larger, higher unit-cost, stricter pin planning |

---

## H7/FPV hardware considerations for ArduPlane

Because ArduPlane is firmware, your hardware choice mostly affects:

- sensor bus load (how many UART/I2C peripherals you can keep on a single controller),
- mission compute needs (estimator + nav + mission logic margin),
- and power/thermal headroom under continuous airflow and long mission profiles.

Practical defaults on ArduPilot H7-class boards often come down to these choices:

- RC input path uses serial receiver mapping (`RC` over UART), not PWM input unless configured intentionally,
- at least one spare UART for MAVLink/companion transport,
- reliable current/voltage measurement for long-range fixed-wing logging,
- clean CAN or I2C strategy for additional buses (GPS/airspeed/compass).

---

## ArduPlane comparison across autopilot branches

| Branch | Vehicle domain | Mission depth | MAVLink behavior | Typical use case |
|---|---|---|---|---|
| `ArduPlane` | fixed-wing VTOL/fixed-wing | full mission set | MAVLink-first telemetry/control | production/autonomy fixed-wing paths |
| `ArduCopter` | multicopter | full mission + position hold stack | MAVLink-first telemetry/control | quadcopters and heavy multicopter workflows |
| `ArduRover` | ground vehicles | route + obstacle workflows | MAVLink-first telemetry/control | differential/ Ackermann rover logic |
| `ArduSub` | underwater | mission + depth-aware modes | MAVLink-like telemetry controls | ROV/ROV-AUV style stack |
| `PX4 fixed-wing stack` | fixed-wing | modern MAVLink + QGC ecosystem | MAVLink + middleware bridges | ROS-heavy, middleware-first projects |

---

## Companion-computer integration notes

- [[ESP32-S3]] is common for compact companion roles.
- Use `SERIALx_BAUD`/`SERIALx_PROTOCOL` consistently on the FC.
- Run link health checks at startup and watch sequence counters / packet drop behavior.
- Use only one authoritative RC source at a time (either FC RC input or companion-generated RC override, never mixed in a way you cannot audit).

Common mistakes:

- Feeding `RC_CHANNELS` data into a radio link path that already has RC decoding done upstream.
- Letting the FC and companion fight over mode/arming controls.
- Overloading one low-bandwidth UART with raw telemetry + RC channels + logs.

---

## Related notes

- [[ArduPilot]]
- [[ArduPilot Mission Planner]]
- [[MAVLink]]
- [[Matek H743-Wing]]
- [[Holybro Pixhawk 6C Mini]]
- [[ESP32-S3]]
- [[PX4]]
- [[MAVProxy]]
- [[QGroundControl]]

## External resources

- ArduPilot ArduPlane docs: `https://ardupilot.org/plane/index.html`
- ArduPilot RC systems: `https://ardupilot.org/plane/docs/common-rc-systems.html`
- ArduPilot Plane autopilot parameters: `https://ardupilot.org/plane/docs/parameters.html`
- RC input over MAVLink (ArduPilot + companion discussion): `https://ardupilot.org/plane/docs/common-rc-over-mavlink.html`
- `MAVLink` protocol reference: `https://mavlink.io/en/`
