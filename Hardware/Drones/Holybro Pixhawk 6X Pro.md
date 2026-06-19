# Holybro Pixhawk 6X Pro

`Holybro Pixhawk 6X Pro` is the premium packaging variant of Holybro’s FMUv6X line, using the same core processor family as the standard 6X with stronger enclosure/value bundle options.

---

## Processor and packaging

- FMU: `STM32H753`
- Family-class: FMUv6X
- Practical distinction: variant-specific mechanical, accessory, and connector considerations versus raw FMU capability

---

## When the Pro variant is sensible

- You need mechanical packaging/accessory options more than raw capability
- Team preferences for production-grade enclosure handling
- You prefer fewer surprises from repeated field handling than one-off lab hardware swaps

## When not worth it

- Strict budget prototyping
- Projects where only MCU headroom is the decision factor
- When you can meet reliability goals with non-Pro packaging and robust handling discipline

---

## Flashing/compatibility posture

- Vendor-supported firmware paths are tied to Holybro family target documentation.
- Stack behavior is best handled like `[[Holybro Pixhawk 6X]]` with explicit revision checks.
- For mixed ArduPilot/PX4 pipelines, treat variant differences as a hardware-selection dimension, not a software miracle.

---

## Related notes

- [[Holybro Boards]]
- [[ArduPilot Boards]]
- [[PX4 Boards]]
- [[Holybro Pixhawk 6X]]
- [[Holybro Pixhawk 6C]]
- [[Holybro Pixhawk 6C Mini]]

---

## External resources

- https://docs.holybro.com/autopilot/pixhawk-6x-pro/supported-firmware
- https://docs.holybro.com/autopilot/pixhawk-6x-pro/overview
- https://holybro.com/products/pixhawk-6x-pro
- https://docs.px4.io/main/en/flight_controller/pixhawk6x.html
