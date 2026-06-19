# Holybro Pixhawk 6X

`Holybro Pixhawk 6X` is a modern FMUv6X-class controller around `STM32H753`, commonly treated as a value-to-performance baseline for ArduPilot/PX4 mixed deployments.

---

## Processor and split

- FMU: `STM32H753` (Cortex-M7, 32-bit Arm)
- IO coprocessor: `STM32F103`
- Bus layout aligns with standard Pixhawk-family assumptions for many mission stacks

---

## Platform intent

- Balanced cost-performance for full mission workflows
- Strong fit for modern multicopter and fixed-wing controllers needing richer telemetry and serial/UART footprint
- Good bridge board for projects that later move between `ArduPilot` and `PX4` environments

---

## Software support behavior

- Official Holybro docs include dedicated technical specification and supported firmware pages.
- The board appears in mainstream Pixhawk-family support flows used by both ArduPilot and PX4 ecosystems.
- Revision tracking matters (sensor revision and USB behavior are among the practical differences called out by users and docs).

---

## What to expect vs what not

| Aspect | Good at | Not ideal for |
|---|---|---|
| Baseline mission stack | yes | Extreme-cost-sensitive bare minimum projects |
| Sensor / telemetry diversity | yes | Teams expecting one-size-fits-all behavior across all variants |
| Rapid first bring-up | yes | Zero-revision-check workflows |
| Upgrade churn tolerance | medium-high | Unmanaged mixed-hardware legacy migrations |

---

## Related notes

- [[Holybro Boards]]
- [[ArduPilot Boards]]
- [[PX4 Boards]]
- [[Holybro Pixhawk 6C]]
- [[Holybro Pixhawk 6X Pro]]
- [[Holybro Pixhawk 6C Mini]]

---

## External resources

- https://docs.holybro.com/autopilot/pixhawk-6x/technical-specification
- https://docs.holybro.com/autopilot/pixhawk-6x/supported-firmware
- https://docs.holybro.com/autopilot/pixhawk-6x
- https://docs.px4.io/main/en/flight_controller/pixhawk6x.html
