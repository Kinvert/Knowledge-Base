# ArduPilot

`ArduPilot` is open-source flight-control firmware used for multirotor, fixed-wing, rover, rover/boat, submarine, and other vehicle classes.

The important distinction:

- `ArduPilot` is software/family + vehicle mode logic.
- `Pixhawk`/`Matek`/`CUAV`/`Holybro` are hardware families.

---

## Core architecture (practical view)

- Central control and vehicle estimator loop runs on the flight controller MCU.
- Hardware abstraction layer maps board peripherals (PWM, I/O, CAN, GPS, baro, compasses, IMU, power, SD logging).
- Autonomy and mission tooling are delivered through parameter/mode architecture and companion links.
- MAVLink is the de-facto external protocol for telemetry, mission upload, parameters, and companion automation.

---

## What ArduPilot is especially known for

- very large flight-controller compatibility history,
- deep mission tooling maturity,
- broad mode coverage for fixed-wing/autopilot workflows,
- robust field-proven logging behavior for repeatability.

---

## ArduPilot vehicle branches

| Branch | Domain | Main behavior emphasis |
|---|---|---|
| `ArduPlane` | fixed-wing/tailsitter/VTOL | mission logic, guidance, speed/altitude modes |
| `ArduCopter` | multicopter | aggressive attitude + position control, mission modes |
| `ArduRover` | rover/ground | path following and autonomy on road/ground constraints |
| `ArduSub` | underwater vehicle | depth + underwater navigation patterns |
| `ArduRover` variants | lawn/field automation | route-first control with autonomy modes |

---

## Comparison: ArduPilot vs nearby options

| Autopilot target | Vehicle focus | Tooling culture | Why pick it here |
|---|---|---|---|
| ArduPilot | multi-vehicle family | Mission Planner, MAVProxy, broad ecosystem | full stack breadth and mission compatibility |
| PX4 | multi-vehicle + strong ROS/Middleware orientation | QGC + ROS2 integration patterns | middleware-first workflows and strict modularity |
| iNav | multicopter + fixed-wing-hybrid hobby stack | firmware simplicity and tuning-centric workflows | low-friction beginner/autopilot crossover |
| Betaflight | multicopter flight stack | low-latency tuning stack | minimal autonomous mission support |
| INAV | indoor/outdoor navigation stack for simpler vehicles | convenience-first autopilot settings | lighter feature depth than ArduPilot/PX4 |

---

## Where ArduPilot is commonly deployed (our drone corpus)

- [[Matek H743-Wing]]
- [[T-Motor H7 Mini]]
- [[Holybro Pixhawk 6C]]
- [[Holybro Pixhawk 6C Mini]]
- [[CUAV V5+]]

---

## Related notes

- [[ArduPlane]]
- [[MAVLink]]
- [[ArduPilot Mission Planner]]
- [[PX4]]
- [[Pixhawk]]
- [[Matek H743-Wing]]
- [[ArduPilot vs Pixhawk]]

## External resources

- Official project docs: `https://ardupilot.org/`
- Board compatibility index: `https://ardupilot.org/dev/docs/choosing-an-autopilot.html`
- Vehicle docs (plane): `https://ardupilot.org/plane/index.html`
- Developer docs: `https://ardupilot.org/dev/`

