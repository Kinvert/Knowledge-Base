# 🧾 Verilog

**Verilog** is a **Hardware Description Language (HDL)** used to model and describe digital systems, including FPGAs and ASICs. It's used to express **Register-Transfer Level (RTL)** logic and simulate the behavior of circuits before they're synthesized into physical hardware.

---

## 🧠 Summary

- Created in the 1980s and standardized as IEEE 1364.
- Describes **combinational and sequential logic**.
- Used for **digital design**, **simulation**, and **synthesis**.
- Similar to C in syntax, making it approachable for software developers.

---

## ⚙️ Common Use Cases

| Use Case                     | Description                                                        |
|------------------------------|--------------------------------------------------------------------|
| FPGA Design                  | Create reconfigurable logic blocks                                |
| ASIC Development             | Define hardware before fabrication                                |
| Digital Simulation           | Test designs via tools like ModelSim, Icarus Verilog              |
| Testbench Creation           | Simulate input/output behavior for verification                   |
| System on Chip (SoC) Design  | Integrate custom blocks within larger digital systems              |

---

## 📚 Key Concepts

| Concept            | Description                                                              |
|--------------------|--------------------------------------------------------------------------|
| `module`           | Building block that defines logic and ports                              |
| `wire` vs `reg`    | `wire` for combinational, `reg` for sequential assignments               |
| `always` blocks    | For continuous or edge-triggered logic (e.g., clocks)                    |
| `assign`           | Used for continuous assignments to `wire`s                               |
| `initial` blocks   | Set up test conditions in simulations (not synthesized into hardware)    |

---

## 🧪 Common Verilog Constructs (No Code Blocks)

- `module` and `endmodule` define a circuit
- `always @ (posedge clk)` handles clocked logic
- `assign` used for continuous combinational logic
- `if` / `else`, `case`, and `for` statements for logic control

---

## 🧰 Toolchains

| Tool             | Description                                        |
|------------------|----------------------------------------------------|
| **Vivado**       | Xilinx IDE for synthesis and simulation            |
| **Quartus**      | Intel (Altera) IDE for FPGAs                       |
| **ModelSim**     | Popular simulator from Mentor Graphics             |
| **Icarus Verilog**| Open-source Verilog simulator                     |
| **GTKWave**      | Waveform viewer for simulation output              |
| **Verilator**    | Converts Verilog to C++ for fast simulation        |

---

## 🔗 Related Standards

- **IEEE 1364** – Verilog Language Standard
- **IEEE 1800** – SystemVerilog, a superset of Verilog

---

## 📐 Comparison Table: Verilog vs VHDL

| Feature           | Verilog                        | VHDL                               |
|-------------------|--------------------------------|------------------------------------|
| Syntax            | C-like                         | Ada-like                           |
| Learning Curve    | Easier for software devs       | More verbose and strict            |
| Popularity        | High in North America          | Higher in Europe/Academia          |
| Simulation Tools  | ModelSim, Verilator, Icarus    | ModelSim, GHDL                     |
| Use Case          | Industry, FPGA, ASIC           | Academia, defense, aerospace       |

---

## 🏆 Strengths

- Widely used in industry
- Compact, expressive syntax
- Extensive support from vendor tools
- Easy to simulate and test

---

## ⚠️ Weaknesses

- Limited type system vs VHDL
- Older constructs are less intuitive
- Lacks strictness for large designs

---

## 🔗 Related Topics

- [[FPGA]]
- [[VHDL]]
- [[RTL (Register Transfer Level)]]
- [[Digital Logic]]
- [[ASIC]]
- [[Vivado]]
- [[Quartus]]
- [[Verilator]]
- [[SystemVerilog]]
- [[Chisel]]
- [[Migen]]

---

## 🌐 External Resources

- [Verilog Wikipedia](https://en.wikipedia.org/wiki/Verilog)
- [Icarus Verilog](http://iverilog.icarus.com/)
- [Verilog Quick Reference](https://inst.eecs.berkeley.edu/~cs150/fa17/resources/verilog-ref.pdf)

---
