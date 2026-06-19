# Holybro Pixhawk 5X

`Holybro Pixhawk 5X` is the FMUv5X class in the modernized line, built around `STM32F765` and used as a legacy-modern bridge between older FMUv5 designs and newer FMUv6 boards.

---

## Processor and role

- FMU processor: `STM32F765`
- Bus and payload profile: practical, stable for mid-capability mission stacks
- Software placement: one of the last strong FMUv5-class choices before FMUv6 lineups dominate modern default recommendations

---

## Best fit

- Teams standardizing on known PX4/ArduPilot workflows with moderate expansion requirements
- Budget-conscious builds where baseline mission behavior is required without stepping all the way to FMUv6 boards
- Legacy-compatible systems already wired for older Pixhawk-style interfaces

---

## What to watch

- Lower headroom for heavy sensor and serial density than FMUv6-class boards
- Long-term migration cost can be higher than buying FMUv6 on day one
- Better used when you have a legacy interface contract you do not want to rewrite

---

## Related notes

- [[Holybro Boards]]
- [[ArduPilot Boards]]
- [[PX4 Boards]]
- [[Holybro Pixhawk 6X]]
- [[Holybro Pixhawk 6C]]

---

## External resources

- https://docs.px4.io/v1.15/en/flight_controller/pixhawk5x.html
- https://doc.cuav.net (as FMUv5 compatibility and comparison context)
- https://docs.px4.io/main/en/flight_controller/pixhawk_series
- https://holybro.com/products/pixhawk-5x
