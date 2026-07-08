# Inverter

An **inverter** converts DC battery power into AC power. For off-grid machine-shop power, the important question is not just wattage. It is whether the inverter can make **real 120/240 V split phase**, how much current each leg can supply, what battery voltage it needs, and whether it can tolerate motor-drive loads through a phase converter.

For a Haas mill or similar 3-phase machine, the chain usually looks like:

```text
battery bank -> inverter / inverter-charger -> 120/240 V split phase -> Phase Perfect / digital phase converter -> 240 V 3-phase machine
```

The weak link is usually surge current, battery current, neutral/bonding design, and whether the inverter manufacturer supports the exact AC topology.

---

## Key point for the proposed +120 V / -120 V plan

The practical version of the idea is **not** "one inverter at -120 V and one at +120 V" as two independent outputs. It is:

- two same-model 120 V inverter/chargers,
- configured by the manufacturer software as a synchronized split-phase pair,
- one leg is `L1-N = 120 V`,
- the other leg is `L2-N = 120 V`,
- `L1-L2 = 240 V`,
- the two legs are 180 degrees apart,
- neutral and ground bonding are handled exactly per inverter, transfer switch, code, and generator/grid mode.

Do not tie two random 120 V inverter outputs together to "make 240 V". That can destroy hardware and create unsafe neutral/ground behavior.

For Victron, the correct path is a supported **VE.Bus split-phase system**. Victron states that two units of the same 120 V model can be configured for 240 V split phase on supported models, and their VE.Bus manual requires same type, firmware/version class, compatible IDs, symmetric AC/DC wiring, and a shared DC bus.

---

## Short answer for this use case

Best Victron fit for the exact plan:

- **Two Victron Quattro 48/10000/140-100/100 120 V units**.
- Configure them as a VE.Bus split-phase pair.
- Output becomes 120/240 V split phase.
- Per unit: 120 V, 10 kVA, 8 kW continuous at 25 C, 6.5 kW at 40 C, 20 kW peak.
- Pair: about 20 kVA apparent, 16 kW real continuous at 25 C, 13 kW real continuous at 40 C.
- 240 V load current from pair: about 66.7 A real-power continuous at 25 C, about 54 A at 40 C.

The **Quattro 48/15000/200-100/100** is a different case:

- The standard datasheet is for a 230 V / European-style unit, not a normal North American 120 V leg unit.
- It can be programmed for 240 V / 60 Hz and used with a Victron autotransformer for split phase.
- It is not the simple "two 120 V units make L1/L2" plan unless you specifically buy a supported 120 V variant and confirm it with Victron/distributor documentation.
- It can feed a 240 V-only load path more naturally than a 120 V load path, but 120/240 split-phase distribution needs the right transformer/topology.

For a Phase Perfect feeding a Haas mill:

- A true 240 V single-phase source is the starting point.
- The inverter system must be sized for the Phase Perfect input current, converter losses, motor starts, and machine peak load.
- The Phase Perfect does not reduce the battery/inverter load; it converts phase. The batteries and inverter still provide the real power.

---

## Main comparison chart

Values are nominal published specs or direct calculations from published VA/W ratings. "Can make 120/240 from battery?" means without utility/grid present.

| Inverter / system | Battery DC | AC output | Continuous output amps | Charger / passthrough | Can make 120/240 from battery? | Notes |
|---|---:|---:|---:|---:|---|---|
| [Victron Quattro 48/10000/140-100/100 120 V](https://www.victronenergy.com/upload/documents/Datasheet-Quattro-3-10kVA-120V-EN-.pdf) | 48 V, 38-66 V input | 120 V single unit | 8 kW = 66.7 A real at 120 V; 10 kVA = 83.3 A apparent | 140 A DC charger; 2x100 A AC feed-through | **Yes, with two identical units configured split phase** | Strong Victron fit for 120/240 V split phase |
| [Victron Quattro 48/15000/200-100/100 230 V](https://www.victronenergy.com/upload/documents/Datasheet-Quattro-3kVA-15kVA-EN-.pdf) | 48 V, 38-66 V input | 230 V, adjustable to 60 Hz | 12 kW = about 52 A at 230 V; datasheet lists 53/50 A max continuous | 200 A DC charger; 2x100 A AC feed-through | **Not as two normal 120 V legs** | Use as 240 V-class source or with Victron autotransformer; verify any 120 V special-order variant |
| [Victron Quattro 48/5000/70-100/100 120 V](https://www.victronenergy.com/upload/documents/Datasheet-Quattro-3-10kVA-120V-EN-.pdf) | 48 V, 38-66 V input | 120 V single unit | 4 kW = 33.3 A real at 120 V; 5 kVA = 41.7 A apparent | 70 A DC charger; 2x100 A AC feed-through | **Yes, with two identical units configured split phase** | Smaller version of the Quattro split-phase path |
| [Victron MultiPlus-II 48/5000/70-95 120 V](https://www.victronenergy.com/upload/documents/Datasheet-MultiPlus-II-120Va-EN-.pdf) | 48 V, 38-66 V input | 120 V single unit | 4 kW = 33.3 A real at 120 V; 5 kVA = 41.7 A apparent | 70 A DC charger; 95 A transfer | **Yes, with two identical units configured split phase** | Good lower-cost Victron split-phase option |
| [Victron MultiPlus-II 48/3000/35-50 120 V](https://www.victronenergy.com/upload/documents/Datasheet-MultiPlus-II-120Va-EN-.pdf) | 48 V, 38-66 V input | 120 V single unit | 2.4 kW = 20 A real at 120 V; 3 kVA = 25 A apparent | 35 A DC charger; 50 A transfer | **Yes, with two identical units configured split phase** | Too small for a serious mill, useful for small shop loads |
| [Victron MultiPlus-II 2x120 V 12/3000 or 24/3000](https://www.victronenergy.com/inverters-chargers/multiplus-ii-2x-120v) | 12 V or 24 V models | 120/240 V passthrough when source exists; 120 V only when inverting | 2.4 kW = 20 A real at 120 V; 3 kVA = 25 A apparent | 120 A charger on 12 V, 70 A charger on 24 V; 50 A passthrough | **No, not by itself** | Important trap: in battery mode it ties L1/L2 together and supplies 120 V, not 240 V |
| [Victron MultiPlus-II 48/10000/140-100 230 V](https://www.victronenergy.com/upload/documents/Datasheet-MultiPlus-II-inverter-charger-EN-.pdf) | 48 V, 38-66 V input | 230 V single phase | 8 kW = about 35 A at 230 V; datasheet lists 37 A max continuous | 140 A DC charger; 100 A transfer | **Not as normal 120/240 split phase** | 240 V-class source; 120/240 requires transformer/topology |
| [Sol-Ark 15K-2P-N](https://sol-ark.com/wp-content/uploads/2024/06/SK150-0001_Rev5_15K-Datasheet_EN-18June2025.pdf) | 48 V nominal, 43-63 V operating | 120/240 V, 120/208 V, or 220 V | 15 kW max continuous = 62.5 A at 240 V; battery-only 12 kW = 50 A at 240 V | 275 A battery charge/discharge; 200 A grid passthrough | **Yes, single unit** | Native split-phase hybrid inverter; no paired 120 V units needed |
| [Sol-Ark 18K-2P-LV](https://sol-ark.com/wp-content/uploads/2025/09/18KDataSheet_PS-00036_Rev6_27Jan2026.pdf) | 48 V nominal, 41-63 V operating | 120/240 V class, 50/60 Hz | 18 kW = 75 A at 240 V; 36 kW surge for 10 s | 350 A battery charge/discharge; 200 A grid passthrough | **Yes, single unit** | Very strong native split-phase option if current revision/listing fits project |
| [Sol-Ark Limitless 12K-2P-LL](https://sol-ark.com/residential-energy-solutions/limitless-12k-2p-ll-hybrid-inverter/) | 48 V battery port | 120/240 V | 12 kW = 50 A at 240 V; 10 kW from batteries = 41.7 A at 240 V | 220 A battery charger; 100 A grid passthrough | **Yes, single unit** | Lower-cost native split-phase option; product page says coming soon |
| [Schneider XW Pro 6848](https://solar.se.com/us/en/product/xw-pro/) | 48 V class | 120/240 V split phase | 6.8 kW = about 28 A at 240 V | commonly 140 A charger class; verify exact SKU/revision | **Yes, single unit** | Mature native split-phase inverter/charger class; lower power than Quattro pair |

---

## Split-phase capability chart

| Topology | Works for 120 V loads? | Works for 240 V loads? | Works for Phase Perfect input? | Comments |
|---|---|---|---|---|
| Two identical 120 V Victron Quattro units in VE.Bus split phase | Yes | Yes | Yes, if sized and commissioned correctly | Best match to the proposed two-inverter plan |
| One 120 V inverter only | Yes | No | No | Needs a second synchronized leg or an autotransformer design |
| One Victron MultiPlus-II 2x120 V unit | Yes | Only when external 120/240 source is present | No in battery-only mode | It passes split phase but does not synthesize 240 V from battery |
| One 230/240 V Victron Quattro or MultiPlus-II | No direct 120 V without transformer | Yes | Usually yes if voltage/frequency/load are approved | Good 240 V source, but not native North American 120/240 distribution |
| 230/240 V Victron plus Victron autotransformer | Yes | Yes | Possible | Requires transformer sizing, neutral design, and Victron-supported wiring |
| Sol-Ark 12K/15K/18K 2P | Yes | Yes | Possible if load/surge accepted | Native 120/240 split-phase hybrid architecture |
| Schneider XW Pro 6848 | Yes | Yes | Possible if load/surge accepted | Native 120/240 split-phase but lower kW ceiling |
| Two arbitrary 120 V inverters wired against each other | Unsafe | Unsafe | No | Must be manufacturer-supported synchronization |

---

## Victron notes

### Quattro 48/10000 120 V

This is the cleanest Victron match for a high-power North American split-phase plan.

Per unit:

- 48 V battery model.
- DC input range: 38-66 V.
- Output in inverter mode: 120 VAC, 60 Hz.
- Continuous: 10 kVA / 8 kW at 25 C.
- Continuous at 40 C: 6.5 kW.
- Peak: 20 kW.
- Charger: 140 A DC.
- AC feed-through: two AC inputs, each 100 A max feed-through.

Two-unit split-phase pair:

```text
L1-N: 120 V from unit A
L2-N: 120 V from unit B
L1-L2: 240 V
continuous real power at 25 C: about 16 kW
continuous real power at 40 C: about 13 kW
240 V real-power current at 25 C: about 66.7 A
240 V real-power current at 40 C: about 54 A
```

This is plausible for a Phase Perfect feeding a moderate shop machine, but the real sizing must start from the mill nameplate input amps and the phase converter recommendation.

### Quattro 48/15000

The Quattro 15000 is high-end, but it is not automatically the right part for the two-120 V split-phase plan.

Standard datasheet values:

- 48 V battery model.
- DC input range: 38-66 V.
- Output: 230 VAC, adjustable to 60 Hz.
- Continuous: 15 kVA / 12 kW at 25 C.
- Continuous at 40 C: 10 kW.
- Continuous at 65 C: 7 kW.
- Peak: 25 kW.
- Maximum continuous AC output current: 53/50 A listed.
- Charger: 200 A DC.
- AC feed-through: two AC inputs, each 100 A max feed-through.

Interpretation:

- Good candidate if you want a **240 V-class single-phase source**.
- Not the simple two-inverter North American split-phase plan unless using a confirmed 120 V version.
- Victron's 230 V datasheet says split phase can be obtained by using their autotransformer with a European inverter programmed for 240 V / 60 Hz.
- If the only 120 V loads are separate shop circuits, handle them with a transformer/subpanel design, not by assuming the 230 V unit gives a neutral.

### MultiPlus-II 2x120 V warning

The MultiPlus-II 2x120 V sounds like it should solve this, but it does not solve the Haas/Phase Perfect case by itself.

Victron states:

- It accepts and passes through both lines of a 120/240 V source.
- When no AC source is available, inverter output is 120 V single phase.
- In inverter mode it internally connects L1 and L2 together so both sides of the panel get 120 V.
- Therefore 240 V is only available when supplied by an external split-phase source.

That means it is good for RV/shore-power panels where 240 V loads should not drain the battery. It is not the right single-box battery-only source for a Phase Perfect.

---

## Sol-Ark notes

Sol-Ark is simpler for North American split phase because the 2P units are native 120/240 V hybrid inverters.

### Sol-Ark 15K-2P-N

Published datasheet values:

- Nominal AC voltage: 120/240 V, 120/208 V, 220 V.
- Max continuous AC output: 15,000 W.
- Max output current: 62.5 A.
- Battery-only continuous: 12,000 W, listed as 50 A at 240 V.
- Grid passthrough: 200 A.
- Nominal DC: 48 V.
- Battery operating range: 43-63 V.
- Max battery charge/discharge current: 275 A.
- Stackable: up to 12 in parallel.

This is a strong single-unit answer when the goal is native 120/240 V without building a split-phase pair manually.

### Sol-Ark 18K-2P-LV

Published datasheet values:

- Continuous AC power: 18,000 W.
- Surge power: 36,000 W for 10 s.
- Max output current: 75 A.
- Grid passthrough: 200 A.
- Nominal DC: 48 V.
- Battery operating range: 41-63 V.
- Continuous battery charge/discharge current: 350 A.
- Stackable: yes, max 12.

This is the strongest single-box low-voltage battery option in this list by published continuous AC current.

### Sol-Ark Limitless 12K-2P-LL

Published product-page values:

- 12 kW 120/240 VAC.
- 10 kW from batteries.
- 100 A grid passthrough.
- 220 A battery charger.
- 48 V battery port.

This is smaller than the 15K/18K options but still native split phase.

---

## Battery voltage and current reality

At these power levels, "48 V" systems pull very high DC current.

Approximate DC current:

```text
DC amps ~= AC watts / (battery volts * inverter efficiency)
```

Examples assuming 48 V nominal and 96% efficiency:

| AC load | Approx DC current |
|---:|---:|
| 6.8 kW | 148 A |
| 8 kW | 174 A |
| 12 kW | 260 A |
| 16 kW | 347 A |
| 18 kW | 391 A |
| 24 kW | 521 A |

Practical implications:

- High-current 48 V systems need short DC runs, real busbars, Class-T or manufacturer-specified fusing, and a BMS that can support surge current.
- A two-Quattro 10000 pair can easily exceed 300 A DC under heavy 240 V load.
- A two-Quattro 15000-class design can exceed 500 A DC if actually loaded near 24 kW.
- Battery charge current is also large: 140 A per Quattro 10000, 200 A for Quattro 15000, 275 A for Sol-Ark 15K, 350 A for Sol-Ark 18K.
- Battery C-rate matters. A 48 V 400 Ah bank is about 19.2 kWh nominal and 350 A is about 0.875 C.

---

## Phase Perfect and Haas mill path

A Phase Perfect or similar digital phase converter expects a solid single-phase input and produces 3-phase output. For a Haas mill, do not size from inverter marketing watts alone.

Use this order:

1. Get the exact Haas model input spec and spindle/motor package.
2. Get the Phase Perfect model recommendation for that Haas.
3. Confirm required single-phase input voltage and breaker/current.
4. Confirm whether the converter can be fed from an inverter-backed source.
5. Size inverter continuous current above the converter input current.
6. Size surge capability for spindle starts, tool changer, coolant pump, compressor, and control transformer load.
7. Confirm what happens during regenerative braking or DC bus dump events.

Important machine-tool gotchas:

- A VFD/vector-drive machine can produce ugly current waveforms and transient demand.
- Some machine tools backfeed energy during deceleration; not every inverter tolerates backfeed.
- 240 V line-to-line is not the same as a utility service with infinite fault current.
- Haas controls may include 120 V internal loads from a transformer; do not feed controls from a generated wild leg unless the converter and machine docs explicitly permit it.
- Phase converters are usually sized by machine HP and input amps, but the inverter must be sized by **single-phase input current**, not just 3-phase output horsepower.

For a two-Quattro 10000 Victron system, the practical ceiling is around:

```text
240 V split phase
about 66 A continuous real-power current at 25 C
about 54 A continuous real-power current at 40 C
surge headroom exists, but thermal and battery limits matter
```

That is a serious source, but large Haas machines can still exceed it depending on spindle, transformer taps, and phase-converter sizing.

---

## Selection guidance

Use **two Victron Quattro 48/10000 120 V units** if:

- you specifically want Victron,
- you want 120 V and 240 V from a split-phase pair,
- you want dual AC inputs for grid/generator,
- you are comfortable commissioning VE.Bus split phase,
- you can build a large 48 V battery/DC bus correctly.

Use **Victron Quattro 48/15000** if:

- you mainly need a 240 V-class single-phase source,
- you are willing to use autotransformer-based split phase for 120 V loads,
- you verify the exact 120 V/230 V/240 V SKU before buying,
- you want higher per-unit power but accept more topology complexity.

Use **Sol-Ark 15K/18K 2P** if:

- you want native 120/240 V from one inverter,
- you want simpler North American split-phase wiring,
- you want high grid passthrough,
- you are comfortable with a hybrid solar inverter ecosystem instead of Victron VE.Bus.

Use **Schneider XW Pro 6848** if:

- you want a mature native 120/240 V inverter/charger,
- 6.8 kW continuous is enough,
- you care more about proven off-grid architecture than maximum output current.

Avoid **MultiPlus-II 2x120 V as the only inverter** if:

- the goal is battery-powered 240 V output,
- the goal is feeding a Phase Perfect from batteries,
- the shop must run 240 V loads during outage/off-grid mode.

---

## Related concepts

- [[Battery Management System]]
- [[Lithium Iron Phosphate Battery]]
- [[Solar Charge Controller]]
- [[VFD]]
- [[Three Phase Power]]
- [[Electric Motor]]
- [[Transformer]]
- [[Grounding]]

---

## External resources

- Victron Quattro product page: https://www.victronenergy.com/inverters-chargers/quattro
- Victron Quattro 120 V datasheet: https://www.victronenergy.com/upload/documents/Datasheet-Quattro-3-10kVA-120V-EN-.pdf
- Victron Quattro 230 V / 15 kVA datasheet: https://www.victronenergy.com/upload/documents/Datasheet-Quattro-3kVA-15kVA-EN-.pdf
- Victron VE.Bus parallel, split-phase, and three-phase systems: https://www.victronenergy.com/live/ve.bus%3Amanual_parallel_and_three_phase_systems
- Victron MultiPlus-II product page: https://www.victronenergy.com/inverters-chargers/multiplus-ii
- Victron MultiPlus-II 120 V datasheet: https://www.victronenergy.com/upload/documents/Datasheet-MultiPlus-II-120Va-EN-.pdf
- Victron MultiPlus-II 230 V datasheet: https://www.victronenergy.com/upload/documents/Datasheet-MultiPlus-II-inverter-charger-EN-.pdf
- Victron MultiPlus-II 2x120 V product page: https://www.victronenergy.com/inverters-chargers/multiplus-ii-2x-120v
- Sol-Ark 15K-2P-N product page: https://sol-ark.com/residential-energy-solutions/whole-home-15k-2p-hybrid-inverter/
- Sol-Ark 15K-2P-N datasheet: https://sol-ark.com/wp-content/uploads/2024/06/SK150-0001_Rev5_15K-Datasheet_EN-18June2025.pdf
- Sol-Ark 18K-2P product page: https://sol-ark.com/residential-energy-solutions/premium-18k-2p-hybrid-inverter/
- Sol-Ark 18K-2P datasheet: https://sol-ark.com/wp-content/uploads/2025/09/18KDataSheet_PS-00036_Rev6_27Jan2026.pdf
- Sol-Ark Limitless 12K-2P-LL product page: https://sol-ark.com/residential-energy-solutions/limitless-12k-2p-ll-hybrid-inverter/
- Schneider XW Pro product page: https://solar.se.com/us/en/product/xw-pro/
- Phase Technologies phase conversion page: https://www.phasetechnologies.com/explore/solutions/phase-conversion
