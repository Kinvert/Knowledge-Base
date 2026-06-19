# Holybro Pixhawk 6C

`Holybro Pixhawk 6C` is the compact FMUv6C-class option centered on `STM32H743`, positioned as a lower-cost modern stack than 6X while preserving broad Pixhawk-family behavior.

---

## Processor and architecture

- FMU: `STM32H743`
- IO coprocessor: `STM32F103` (as documented in Holybro technical specs)
- Form factor and routing: compact layout with good serial/I/O utility

---

## Typical use profile

- Modern cost-conscious projects that still need `ArduPilot` / `PX4` capability
- Good intermediate option for rover and fixed-wing payloads with moderate expansion needs
- Often considered before stepping up to 6X when enclosure and variant density are limited

---

## Limitations

- Not as broad in raw expansion margin as FMUv6X boards
- Some installations require careful power/noise routing discipline
- Value comes from compactness, so integration quality matters on first bring-up

---

## Related notes

- [[Holybro Boards]]
- [[ArduPilot Boards]]
- [[PX4 Boards]]
- [[Holybro Pixhawk 6C Mini]]
- [[Holybro Pixhawk 6X]]
- [[Holybro Pixhawk 6X Pro]]

---

## External resources

- https://docs.holybro.com/autopilot/pixhawk-6c/technical-specification
- https://docs.holybro.com/autopilot/pixhawk-6c/supported-firmware
- https://docs.holybro.com/autopilot/pixhawk-6c
- https://docs.px4.io/main/en/flight_controller/pixhawk6c_mini.html
