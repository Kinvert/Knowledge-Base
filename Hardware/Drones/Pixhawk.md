# Pixhawk

`Pixhawk` is a hardware ecosystem label, not a software stack.  
The term covers a set of compatible autopilot hardware families that can run ArduPilot or PX4 depending on target and vendor support.

---

## Why this distinction matters

When teams say "PX4 vs Pixhawk" or "ArduPilot vs Pixhawk", they are mixing categories:
- **PX4 / ArduPilot** = flight-control software families
- **Pixhawk** = form-factor/board lineage concept

This confusion usually causes bad procurement decisions (wrong target selected for a selected firmware workflow).

---

## What a Pixhawk-family board typically provides

- `STM32` MCU core (generation and revision depends on exact model)
- flight I/O set for PWM/UART/I2C/CAN depending on model
- power monitor + OSD/logging capabilities in many variants
- community tooling compatibility across many FC targets

---

## Common Pixhawk-family hardware in this vault

- [[Holybro Pixhawk 6C]], [[Holybro Pixhawk 6C Mini]], [[Holybro Pixhawk 6X]]
- [[Holybro Pixhawk 6X Pro]], [[Holybro Pixhawk 5X]], [[Holybro Pixhawk 4]]
- [[CUAV V5+]], [[Cube Orange+]], [[Matek H743-Wing]]

---

## Comparison: choosing by constraint (not by brand)

| Constraint | Pixhawk-family pattern | Why it helps |
|---|---|---|
| Budget + solid stack | [[Holybro Pixhawk 6C]] / [[Holybro Pixhawk 6C Mini]] | strong software ecosystem with compact footprint |
| Modern feature ceiling | [[Holybro Pixhawk 6X]] / [[Holybro Pixhawk 6X Pro]] | larger I/O/feature room for growth |
| Legacy familiarity | [[Holybro Pixhawk 5X]] / [[Holybro Pixhawk 4]] | broad baseline docs for older firmware pipelines |
| Industrial/reliability posture | [[Cube Orange+]], [[CUAV V5+]] | reputation and structured hardware ecosystems |
| Maximum H7 density | [[Matek H743-Wing]] and related H743 variants | bigger fixed-wing I/O density than typical mini boards |

---

## Related notes

- [[ArduPilot]]
- [[PX4]]
- [[ArduPilot Boards]]
- [[PX4 Boards]]
- [[ArduPilot vs Pixhawk]]

## External resources

- Holybro Pixhawk pages: `https://docs.holybro.com/` (family hardware pages)
- CUAV autopilot resources: `https://doc.cuav.net/`
- Cubepilot carrier guidance: `https://docs.cubepilot.org/`
- ArduPilot supported hardware flow: `https://ardupilot.org/dev/docs/choosing-an-autopilot.html`
- PX4 standard hardware references: `https://docs.px4.io/main/en/flight_controller/` (version-specific pages)

