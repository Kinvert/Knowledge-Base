# CUAV V5+

`CUAV V5+` is a rugged, field-oriented Pixhawk-style stack with strong support for both `ArduPilot` and `PX4`, often used for mixed mission builds where hardware stability matters more than raw price.

For other stack-aligned board groups, use [[ArduPilot Boards]] and [[PX4 Boards]].

---

## Board profile

- Flight processor: `STM32F765` (FMU)
- IO processor: `STM32F100`
- Design: two-layer processing split with redundancy-oriented signal paths and common telemetry/companion integration paths
- Product intent: field work, long tuning loops, and mixed sensor stacks

---

## Why teams pick CUAV V5+

- Broad support posture for both stack ecosystems.
- Stronger availability of revision documentation compared with some legacy FMUv5 boards.
- Better fit for hard-use cases than cheap value-class controllers when failure cost is high.

---

## ArduPilot + PX4 support

- CUAV documentation explicitly lists V5+ as compatible with both ecosystems.
- ArduPilot and PX4 users rely on the same board target family patterns, but tooling and vendor packages should be matched to revision.

Typical practical flow:

- Flash ArduPilot (`.apj` flow, Mission Planner path when available).
- Flash PX4 (`QGroundControl` standard flow).
- Validate serial routing and bootloader behavior before first flight.
- Keep parameter backups for cross-stack migration.

---


## Board comparison chart

| Metric | CUAV V5+ | Holybro Pixhawk 6X | Holybro Pixhawk 6C | Holybro Pixhawk 5X | Cube Orange+ |
|---|---|---|---|---|---|
| Main FMU processor | STM32F765 + STM32F100 co-processor | STM32H753 | STM32H743 | STM32F765 | STM32H757 (dual-core M7+M4) |
| Current ecosystem default use | Field robotics and mixed autonomy | Value + modern autopilots | Compact modern stack builds | Budget modern-autopilot entry | Carrier board ecosystems, mission uptime |
| ArduPilot support pattern | Official docs / broad FC target family support | ArduPilot firmware support in common target paths | ArduPilot support through standard target workflow | Legacy-compatible ArduPilot usage path | ArduPilot install flows documented by vendor |
| PX4 support pattern | Available as supported class in CUAV docs and ecosystem | PX4 standard-compatible autopilot path | PX4 FMUv6C-class path | PX4 FMUv5-class legacy entries | PX4 documented as footprint-compatible branch |
| Typical downside | Higher unit cost than hobby boards | Can require exact revision checks for USB/serial | Lower expansion margin than 6X-class | Older architecture pressure in heavy sensor builds | Connector/stack compatibility can be variant-sensitive |

## Pros and cons

### What it can do / what it can’t do

| Use case | Good fit | Weak point |
|---|---|---|
| Long-duration field missions | Strong | Costly compared to hobby-only boards |
| Rover + fixed-wing mixed stack | Strong | Requires careful revision/companion wiring checks |
| Autonomous mission testing | Strong | Price and procurement friction for pure prototyping |
| Rapid budget prototypes | Weak | Expensive for one-off classroom builds |

### Practical pros / cons

| Pros | Cons |
|---|---|
| Reliable field platform for mixed ArduPilot/PX4 workflows | Higher unit cost than compact hobby-target controllers |
| Mature wiring assumptions when revision is fixed | Tighter procurement loops because variant-specific docs matter |
| Good starting point for long-duration autonomy projects | Less suitable for disposable prototypes and ultra-tight stacks |

---

## Board comparison context

This note is a candidate for the higher end of the [[ArduPilot Boards]] and [[PX4 Boards]] matrix where reliability and ecosystem compatibility are prioritized.

---

## Related notes

- [[CUAV Boards]]
- [[ArduPilot Boards]]
- [[PX4 Boards]]
- [[CUAV X7]]

---

## External resources

- https://doc.cuav.net/controller/v5-autopilot/en/v5%2B.html
- https://store.cuav.net/shop/v5-controller/
- https://ardupilot.org/ardupilot/docs/common-autopilots.html
- https://docs.px4.io/main/en/flight_controller/pixhawk_series
