# CUAV Boards

`CUAV` produces Pixhawk-compatible flight computers with CUAV-branded firmware and ecosystem tooling. This page groups CUAV choices by role and links to the stack-level comparisons.

---

## Main CUAV autopilots

| Board | Main processor | Why teams pick it | Common caveat |
|---|---|---|---|
| [[CUAV V5+]] | `STM32F765` + `STM32F100` | Field-reliable mixed mission profile, strong redundancy-oriented build assumptions | Costly compared to compact hobby targets |
| [[CUAV X7]] | `STM32H743` | Cleaner H7-class option with industrial/regulatory integration posture | Variant spread (`X7`, `X7+`, `Nora`, `Nora+`) changes baseboard behavior |

---

## Stack-level direction

- CUAV hardware is often used in `ArduPilot` and `PX4` workflows.
- In this vault, route to stacks through `[[ArduPilot Boards]]` and `[[PX4 Boards]]` for stack-by-stack comparison.

---

## Related notes

- [[CUAV X7]]
- [[CUAV V5+]]
- [[ArduPilot Boards]]
- [[PX4 Boards]]

---

## Sources

- https://doc.cuav.net
- https://doc.cuav.net/controller/v5-autopilot/en/v5%2B.html
- https://doc.cuav.net/controller/x7/en/
- https://doc.cuav.net/controller/x7/en/ardupilot-users-manual.html

