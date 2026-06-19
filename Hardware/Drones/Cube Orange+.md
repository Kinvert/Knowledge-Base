# Cube Orange+

`Cube Orange+` is CubePilot’s flagship modular autopilot around `STM32H757` with heavy ecosystem integration and active `ArduPilot` usage patterns.

---

## Processor and architecture

- Flight module MCU: `STM32H757` (dual-core M7 + M4 in published docs)
- Strongly ecosystemed around modular carrier board approach
- Typical mission profile: redundancy-heavy builds where maintenance and connector ecosystem maturity matter

---

## What this board is known for

- `ArduPilot` workflow maturity through Cube-specific installation documentation.
- Broad compatibility with companion computers and field systems that are already aligned with Cube modules.
- Different from strict standard Pixhawk footprints in some connector/stack variants, so adapter and baseboard choice matters.

---

## Flashing and deployment

- ArduPilot: CubePilot vendor flow for `Install ArduPilot`.
- PX4: use Cube-specific compatibility documentation and standard board setup pages (not always identical to generic Pixhawk standard paths).
- Across both stacks, matching firmware target to specific module/baseboard revision is the #1 setup reliability step.

---

## Practical strengths / limits

| Aspect | Strong | Weak |
|---|---|---|
| Redundancy and mission uptime behavior | Strong | Higher integration overhead than single-PCB race/consumer boards |
| Community and vendor support depth | Strong | Naming and connector confusion can cause setup mistakes |
| Use in mixed ArduPilot + PX4 labs | Strong | Not a pure “generic Pixhawk standard + no caveat” card |

---

## Related notes

- [[CubePilot Boards]]
- [[ArduPilot Boards]]
- [[PX4 Boards]]
- [[CUAV V5+]]

---

## External resources

- https://docs.cubepilot.org/user-guides/autopilot/the-cube/setup/firmware/installing-ardupilot
- https://docs.cubepilot.org/user-guides/autopilot/the-cube/introduction/interface-specifications
- https://docs.px4.io/v1.15/en/flight_controller/cubepilot_cube_orangeplus.html
- https://docs.cubepilot.org/user-guides/autopilot/the-cube/introduction/flight-management-unit-fmu
