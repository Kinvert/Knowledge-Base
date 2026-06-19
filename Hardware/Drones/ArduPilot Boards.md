# ArduPilot Boards

This note is the main index for boards commonly used with `ArduPilot`, with emphasis on practical compatibility and board-selection tradeoffs.

Use this when the decision is stack-first for autonomy and mission workflows rather than only price.

---

## Mainstream ArduPilot-compatible FC shortlist

| Board | Main processor | Why teams pick it for ArduPilot | Common caveat |
|---|---|---|---|
| [[CUAV V5+]] | STM32F765 + STM32F100 | Field reliability and conservative firmware behavior on mixed vehicles | Revision discipline and unit cost |
| [[CUAV X7]] | STM32H743 | H7-class option with CUAV ecosystem support and cleaner industrial profile | Variant spread across `X7`/`X7+`/`Nora` affects baseboard behavior |
| [[Cube Orange+]] | STM32H757 (dual-core) | Strong ecosystem + modular redundancy workflows | Requires vendor-specific connector and stack alignment |
| [[Holybro Pixhawk 6X]] | STM32H753 | Best modern balance of cost and capability | Needs revision-level parameter checks |
| [[Holybro Pixhawk 6X Pro]] | STM32H753 | Packaging plus enclosure/value bundle options in mission workflows | High price for mostly same FMU core as 6X |
| [[Holybro Pixhawk 6C]] | STM32H743 | Compact ArduPilot-capable middle stack | Slightly reduced expansion margin compared to 6X |
| [[Holybro Pixhawk 6C Mini]] | STM32H743 | Smaller form factor with ArduPilot-compatible behavior class | Tight mechanical and thermal margins in dense setups |
| [[Holybro Pixhawk 5X]] | STM32F765 | Lower-cost legacy entry for known wiring stacks | FMUv5-class headroom ceiling |
| [[Holybro Pixhawk 4]] | STM32F765 | Lowest-cost Pixhawk-style entry in this set | Older-generation support behavior and tooling friction |

## How to choose in ArduPilot-first projects

- Prioritize mission-critical deployments: `[[CUAV V5+]]`, `[[Cube Orange+]]`.
- Prioritize modern value: `[[Holybro Pixhawk 6X]]` and `[[Holybro Pixhawk 6C]]` variants.
- Prioritize legacy budget recovery: `[[Holybro Pixhawk 5X]]` or `[[Holybro Pixhawk 4]]` when you have existing wiring assumptions.

## Related notes

- [[PX4 Boards]]
- [[Pixhawk]]
- [[MAVLink]]
- [[CUAV Boards]]
- [[Holybro Boards]]
- [[CubePilot Boards]]

## Sources

- https://ardupilot.org/dev/docs/choosing-an-autopilot.html
- https://ardupilot.org/plane/docs/common-autopilots.html
- https://ardupilot.org/planner/docs/common-loading-firmware-onto-pixhawk.html
- https://doc.cuav.net/controller/v5-autopilot/en/v5%2B.html
- https://doc.cuav.net/controller/x7/en/
- https://docs.cubepilot.org/user-guides/autopilot/the-cube/setup/firmware/installing-ardupilot
- https://docs.holybro.com/autopilot/pixhawk-6x/technical-specification
- https://docs.holybro.com/autopilot/pixhawk-6c/technical-specification
