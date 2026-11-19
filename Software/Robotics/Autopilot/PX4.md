# PX4

`PX4` is a modern autopilot software stack used heavily in ROS-centric and middleware-heavy workflows.

As with ArduPilot, it is software:
- it runs on compatible flight controllers,
- it is not a board family by itself.

---

## What this matters for setup decisions

- If your integration stack is strongly ROS2-first, PX4 often maps cleanly to middleware and message bridging patterns.
- If your integration stack is MAVLink-first + mature mission tool ecosystems, ArduPilot is usually more direct for many classical autonomy tasks.
- Both are valid choices when the board family is compatible.

---

## Typical strengths

- strong middleware and software architecture alignment with ROS2 toolchains,
- clean ecosystem path through QGroundControl and reference autopilot pipelines,
- clear separation between flight stack and companion systems at the software boundary.

---

## PX4 vs nearby alternatives

| Stack | Design style | Best use pattern | Main trade-off |
|---|---|---|---|
| PX4 | middleware-first flight stack | ROS2 + complex autonomy + external command graphs | onboarding can feel heavier for simple hobby stacks |
| ArduPilot | mission-first + broad vehicle-family model | quick mission-heavy autonomy on many boards | larger codebase + broader historical behavior surface |
| iNav | lightweight flight-capable stack | simpler vehicle-specific quick starts | lighter mission ecosystem than PX4 |
| Betaflight | acro/fpv flight stack | latency-focused manual flight | limited autonomous mission depth |
| PX4 + companion custom | hybrid stack | large autonomy pipelines with external mission authority | stronger integration effort for every vehicle class |

---

## Board families commonly paired with PX4

- [[Matek H743-Wing]] (via compatible boot/flashing routes where supported)
- [[Holybro Pixhawk 6C]]
- [[Holybro Pixhawk 6X]]
- [[CUAV X7]]
- [[Cube Orange+]]

---

## Related notes

- [[PX4 Boards]]
- [[ArduPilot]]
- [[QGroundControl]]
- [[MAVLink]]
- [[ROS2]]
- [[Pixhawk]]
- [[ArduPilot vs Pixhawk]]

## External resources

- Official PX4 docs: `https://docs.px4.io/main/en/`
- QGroundControl integration guidance: `https://docs.qgroundcontrol.com/`
- PX4 + ROS2 overview: `https://docs.px4.io/main/en/ros2/`
- Supported autopilot hardware list references (via official docs and vendor pages)

