# CUAV X7

`CUAV X7` is CUAV’s H7-generation flight-controller family (`X7`, `X7+`, `Nora`, `X7 Pro` variants) targeting industrial and research platforms with `ArduPilot` and `PX4` compatibility.

---

## Processor and sensor profile

- Processor: `STM32H743` (all listed X7 family variants in CUAV docs)
- Form factor: modular option plus integrated baseboard options depending on variant
- Notable stack intent: platform-scale integration where firmware support and IMU quality matter more than minimum BOM cost

--- 

## When to use CUAV X7

- Academic/industrial robotics where vibration and integration margin matter
- Fixed-wing and rover-heavy builds that benefit from larger peripheral counts than compact F4/F7 boards
- Teams needing an H7 generation option from CUAV without the premium of other flagship lines

---

## Setup and support notes

- CUAV states the board is compatible with ArduPilot and PX4 firmware families in current maintenance windows.
- Choose the exact X7 sub-variant (`X7`, `X7+`, `X7 Pro`, `Nora`, `Nora+`) before planning baseboard connections.
- For mission-critical projects, verify sensor module compatibility with your exact firmware version.
- For PX4 paths, follow the vendor stack + standard Pixhawk-family flashing flow for your target branch.

---

## Practical comparison posture

Compared with `[[CUAV V5+]]`:
- more modern MCU baseline (`H743`),
- often useful when you are standardizing on a lighter industrial profile.

Compared with H7-class Pixhawk options (`[[Holybro Pixhawk 6X]]`, `[[Holybro Pixhawk 6C]]`, `[[Holybro Pixhawk 6C Mini]]`):
- X7 offers a different carrier/plug-in approach in CUAV ecosystem
- still belongs to the same broader “Pixhawk-compatible with caveats” decision class rather than a full drop-in for all Pixhawk SKUs

---

## Related notes

- [[CUAV V5+]]
- [[CUAV Boards]]
- [[ArduPilot Boards]]
- [[PX4 Boards]]
- [[iNav]]

---

## External resources

- https://doc.cuav.net/controller/x7/en/
- https://doc.cuav.net/controller/x7/en/x7-plus.html
- https://doc.cuav.net/controller/x7/en/ardupilot-users-manual.html
- https://doc.cuav.net/controller/x7/en/quick-start/quick-start-x7-plus.html
