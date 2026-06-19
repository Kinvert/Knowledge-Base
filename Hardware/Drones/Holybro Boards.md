# Holybro Boards

`Holybro` hosts multiple Pixhawk-family variants that are commonly cross-compared in ArduPilot/PX4 stack planning. The links here cover the Pixhawk line used in current field and hobby workflows.

---

## Holybro Pixhawk family shortlist

| Board | Main processor | Why teams pick it | Common caveat |
|---|---|---|---|
| [[Holybro Pixhawk 6X]] | `STM32H753` | Balanced modern value for mission/autonomy workflows | Revision-level USB/serial checks are common |
| [[Holybro Pixhawk 6X Pro]] | `STM32H753` | Premium packaging and field-handling value | Premium price for mostly same FMU core class |
| [[Holybro Pixhawk 6C]] | `STM32H743` | Compact modern alternative with ArduPilot/PX4 parity options | Reduced expansion margin versus 6X |
| [[Holybro Pixhawk 6C Mini]] | `STM32H743` | Compact build profile for space-limited vehicles | Tight thermal/mechanical integration windows |
| [[Holybro Pixhawk 5X]] | `STM32F765` | Lowest-cost migration path for legacy wiring assumptions | Legacy FMUv5-generation headroom limits |
| [[Holybro Pixhawk 4]] | `STM32F765` | Legacy starter board for constrained budgets | Older tooling and ecosystem friction on modern stacks |

---

## Stack-level direction

- Board selection usually gates the final choice in this stack family, not the other way around.
- For stack-by-stack comparison, use `[[ArduPilot Boards]]` and `[[PX4 Boards]]`.

---

## Related notes

- [[Holybro Pixhawk 4]]
- [[Holybro Pixhawk 5X]]
- [[Holybro Pixhawk 6X]]
- [[Holybro Pixhawk 6X Pro]]
- [[Holybro Pixhawk 6C]]
- [[Holybro Pixhawk 6C Mini]]
- [[ArduPilot Boards]]
- [[PX4 Boards]]

---

## Sources

- https://holybro.com/collections/flight-controllers
- https://docs.holybro.com/autopilot/pixhawk-6x
- https://docs.holybro.com/autopilot/pixhawk-6x-pro
- https://docs.holybro.com/autopilot/pixhawk-6c
- https://docs.holybro.com/autopilot/pixhawk-6c-mini
- https://docs.holybro.com/autopilot/pixhawk-5x
- https://docs.holybro.com/autopilot/pixhawk-4

