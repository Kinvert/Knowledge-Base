# 6C Mini

`6C Mini` is commonly used as the short form of [[Holybro Pixhawk 6C Mini]], the compact `STM32H743`-class member of the FMUv6C family.

If you need a full page for the hardware family, use [[Holybro Pixhawk 6C Mini]].

---

## What this shorthand usually means

- Same software family position as standard FMUv6C-class controllers.
- Reduced footprint and tighter thermal/mechanical margins than larger 6C-style boards.
- Strong fit for compact autopilot builds where you want ArduPilot/PX4 compatibility with a hard size limit.

---

## Quick selection check

Use the 6C Mini profile when:

- your payload bus is constrained (small fixed-wing or racing-sized vehicle),
- you want modern H7-grade compute but not the full 6C footprint,
- and you can validate connector/power margins before final mechanical packaging.

Avoid it when:

- you need very high current/voltage conversion headroom in the baseboard,
- you expect to grow into dense sensor/camera payloads without changing the FC,
- or thermal margin is difficult to manage inside enclosure constraints.

---

## 6C Mini vs nearby options

| Option | Board class | Main trade-off | Best use |
|---|---|---|---|
| [[Holybro Pixhawk 6C Mini]] | FMUv6C compact | Smallest form factor, same H743-class compute class | size-limited missions and compact fixed-wing/multi builds |
| [[Holybro Pixhawk 6C]] | FMUv6C full | More board-level margin and easier cable routing | same software stack with easier thermal/mechanical headroom |
| [[Holybro Pixhawk 6X]] | FMUv6X | Higher-end signal budget than 6C family | modern builds with heavier autonomy payload and future growth |
| [[Holybro Pixhawk 5X]] | FMUv5X | mature legacy ecosystem | budget legacy-friendly autopilot pipelines |
| [[Matek H743-Wing]] | H743 family | very high IO and mixed-sensor density | fixed-wing and VTOL builds where footprint is available |

---

## Related notes

- [[Holybro Pixhawk 6C Mini]]
- [[Holybro Pixhawk 6C]]
- [[Holybro Pixhawk 6X]]
- [[ArduPilot Boards]]
- [[PX4 Boards]]

