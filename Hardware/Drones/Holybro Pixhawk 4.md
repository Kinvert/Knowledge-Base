# Holybro Pixhawk 4

`Holybro Pixhawk 4` is the older FMUv5-era Pixhawk-style controller (`STM32F765`) still used in budget and legacy integration contexts.

---

## Processor and architecture

- FMU processor: `STM32F765`
- Historical relevance: one of the long-lived FMUv5 designs in the Pixhawk ecosystem
- Current relevance: still works for many legacy stacks, but support clarity is generally lower than modern flagship options

---

## Where it still works well

- Legacy stacks already wired around older Pixhawk assumptions
- Low-cost training and evaluation kits where procurement cost dominates
- Controlled indoor or low-stress scenarios with conservative sensor load

---

## What to avoid

- Treating it as a default modern benchmark for large new mixed-autonomy projects
- Assuming equal behavior with FMUv6-line modern payload stacks
- Ignoring explicit firmware branch and revision compatibility checks

---

## Related notes

- [[Holybro Boards]]
- [[ArduPilot Boards]]
- [[PX4 Boards]]
- [[Holybro Pixhawk 5X]]
- [[Holybro Pixhawk 6C]]

---

## External resources

- https://docs.px4.io/v1.15/en/flight_controller/pixhawk4.html
- https://docs.px4.io/v1.17/en/hardware/reference_design
- https://holybro.com/product/pixhawk-4
- https://docs.px4.io/main/en/assembly/quick_start_pixhawk4.html
