# PX4 Boards

This note is the main index for boards commonly targeted by `PX4` in the current Pixhawk-family workflows.

Use this when the decision is middleware-first, ROS2-first, or strict HITL/SITL alignment with PX4 documentation.

---

## Mainstream PX4-compatible FC shortlist

| Board | Main processor | Why teams pick it for PX4 | Common caveat |
|---|---|---|---|
| [[CUAV V5+]] | STM32F765 + STM32F100 | Broad firmware pathway with practical dual-stack overlap | Revision and packaging details still need vendor-side confirmation |
| [[Cube Orange+]] | STM32H757 (dual-core) | Mature ecosystem plus PX4 ecosystem references for companion workflows | Non-canonical footprint details vary by variant |
| [[Holybro Pixhawk 6X]] | STM32H753 | Standard FMUv6X class in PX4 supported hardware maps | Ensure target branch alignment |
| [[Holybro Pixhawk 6C]] | STM32H743 | FMUv6C-class option with strong compatibility ratio | IO and thermal margin tradeoff in dense builds |
| [[Holybro Pixhawk 6C Mini]] | STM32H743 | Compact variant in the same class | Very tight form-factor constraints |
| [[Holybro Pixhawk 6X Pro]] | STM32H753 | Same FMUv6 core class with packaging options | Price is high for baseline software loops |
| [[Holybro Pixhawk 5X]] | STM32F765 | Legacy-class bridge where older tooling is already in place | Discontinued-style behavior in some contexts |
| [[Holybro Pixhawk 4]] | STM32F765 | Legacy/legacy-compatible test hardware if existing stack already assumes it | Reduced active headroom in modern pipelines |

## How to choose in PX4-first projects

- Prioritize ROS2/offboard stacks and strict middleware alignment: `[[Holybro Pixhawk 6X]]` or `[[Cube Orange+]]`.
- Prioritize compact modern hardware: `[[Holybro Pixhawk 6C]]` / `[[Holybro Pixhawk 6C Mini]]`.
- Prioritize legacy migration compatibility: `[[Holybro Pixhawk 5X]]` or `[[Holybro Pixhawk 4]]` only if your toolchain already tolerates it.

## Related notes

- [[ArduPilot Boards]]
- [[PX4]]
- [[Pixhawk]]
- [[CUAV Boards]]
- [[Holybro Boards]]
- [[CubePilot Boards]]

## Sources

- https://docs.px4.io/main/en/flight_controller/pixhawk_series
- https://docs.px4.io/v1.14/en/flight_controller/autopilot_pixhawk_standard
- https://docs.px4.io/v1.13/en/flight_controller/cubepilot_cube_orange
- https://docs.holybro.com/autopilot/pixhawk-6x/supported-firmware
- https://docs.holybro.com/autopilot/pixhawk-6x-pro/supported-firmware
- https://docs.holybro.com/autopilot/pixhawk-6c/supported-firmware
- https://doc.cuav.net/controller/v5-autopilot/en/v5%2B.html
