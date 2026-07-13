# Amaranth

`Amaranth` (formerly `nMigen`) is a Python-based hardware description language and toolchain for building digital logic and synthesizable hardware. It is used to write hardware designs in a Python-embedded style and target FPGA/Verilog flows.

---

## Overview

`Amaranth` lets you describe synchronous and combinational logic using Python objects, then elaborate to HDL (commonly Verilog) for synthesis.

Practical traits:

- Domain-specific HDL embedded in Python.
- Good for rapid iteration and parameterized designs.
- Strong Python integration for generators, metaprogramming, and test automation.
- Commonly used for FPGA designs, control-heavy digital logic, and small-to-mid hardware projects.

---

## Core concepts

### Modules and hierarchy

- You define hardware as hierarchical modules.
- Signals, FSMs, counters, and combinational logic are explicit objects in Python code.
- Build-time construction is separated from synthesis-time elaboration.

### Clock domains

- Multiple clock domains can be modelled explicitly.
- Clock-domain crossing logic is handled via explicit patterns in code and validation.

### Ports and interfaces

- `Signal`, `Record`, and bus abstractions let you define IO and internal wiring cleanly.
- You can build reusable interfaces for standard buses and protocol adapters.

### Frontend style

Compared with raw Verilog entry points, Amaranth favors:

- programmatic wiring,
- composable module factories,
- Python-native conditional and loop behavior during elaboration.

---

## Build and toolflow

Typical path:

1. Write an Amaranth design in Python.
2. Elaborate to Verilog via the toolchain.
3. Run synthesis/place-and-route with FPGA tools (e.g., open-source or vendor flows depending on target board).
4. Simulate and iterate.

Common workflow includes:

- versioned Python environments,
- scripted build tasks,
- generated netlists for repeatable synthesis.

---

## Why teams use Amaranth

- Faster architecture iteration compared to writing large repetitive RTL manually.
- Easier generation of parameterized blocks and repeated patterns.
- Better integration with Python test frameworks for quick testbench scripting.
- Good fit when hardware design logic and software control tools are co-developed in the same language stack.

---

## Comparison

| Option | Language style | Type system | Typical target | Strength | Tradeoff |
|---|---|---|---|---|---|
| Amaranth | Python-embedded HDL | Python object model + explicit signal typing | FPGA/Verilog output | rapid parameterization, Python ecosystem | Python runtime/tooling dependency during authoring |
| Verilog | Text HDL | static HDL syntax | FPGA/ASIC synthesis | universal compatibility, mature tools | verbose, harder for high-level generation |
| VHDL | Strongly typed HDL | static types, package-heavy | FPGA/ASIC synthesis | mature in industry, strict style | steeper learning curve, slower iteration |
| SystemVerilog | Extended Verilog | richer constructs | FPGA/ASIC synthesis | large ecosystem, broad adoption | heavier syntax and tool variance |
| Chisel | Scala-embedded HDL | Scala typing | ASIC/FPGA with generators | strong generators and libraries | Scala/build complexity |
| SpinalHDL | Scala-embedded HDL | Scala-based | FPGA/ASIC flows | expressive generators | learning curve similar to Scala |
| nMigen | Python-embedded (precursor) | Python model layer | FPGA/Verilog output | historical predecessor | largely superseded by Amaranth |

---

## Practical strengths

- Good for educational and production prototypes where iteration speed matters.
- Very practical for parameterized datapaths and control logic.
- Strong fit for teams already using Python in ML, robotics, or systems automation.

---

## Practical limits

- If the team standardizes on `SystemVerilog`, integrating mixed workflows can create extra review and lint friction.
- Vendor toolchains may still require familiar Verilog integration steps.
- Team onboarding can be awkward if contributors are hardware-only without Python background.

---

## Amaranth vs HDL alternatives

- Choose **Verilog/SystemVerilog** when you need the broadest tool compatibility and strictest industry standardization.
- Choose **Amaranth** when rapid design iteration and Python-driven hardware generation are higher value than raw syntax minimalism.
- Choose **Chisel/Spinal** when Scala-level hardware generators are preferred across larger teams with Scala fluency.

---

## Related notes

- [[Verilog]]
- [[FPGA]]
- [[Migen]]
- [[RISC-V]]

---

## Official resources

- Homepage: https://amaranth-lang.org/
- GitHub: https://github.com/amaranth-lang/amaranth
- Documentation: https://amaranth-lang.org/docs/amaranth/
- Language book/index: https://amaranth-lang.org/docs/amaranth/latest/index.html
