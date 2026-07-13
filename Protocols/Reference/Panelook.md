# Panelook

`Panelook` is an online parts-and-spec database for liquid crystal display modules/panels (LCD/LCM), commonly used as a fast reference during reverse engineering, sourcing, and compatibility checks.

---

## What it is for

- model and SKU lookups for display panels,
- mechanical sizes and pinouts,
- interface details (FPC/connector info),
- quick comparison between panel variants,
- sourcing/stock hints from panel marketplaces.

It is mainly useful when your workload is display integration, teardown support, or panel replacement selection.

---

## Why it helps an EE

If a display is dead/unknown, `Panelook` can give you:

- possible model matches,
- candidate electrical specs,
- quickly comparable alternatives,
- enough clues to narrow a working replacement before pulling datasheets from the manufacturer.

It is **not** an official regulatory authority and often requires manual verification against official datasheets and PCB-level checks.

---

## Practical usage pattern

1. Start with the known model number/photo from the device.
2. Use `Panelook` search for exact model string and brand variants.
3. Confirm connector/pin-count/interfacing details before ordering.
4. Validate in your schematic/PCB context before soldering.

---

## Notes

For regulated/safety-critical behavior, always verify against official manufacturer datasheets and EMC/rating docs.

