# 🛠️ G-code: Introduction & Overview

## 📚 What is G-code?

**G-code** (also known as **RS-274**) is the **standard language used to control CNC (Computer Numerical Control) machines**, such as mills, lathes, 3D printers, laser cutters, and other automated manufacturing tools. It's a **low-level, imperative language** that sends precise instructions to machines about movement, speed, tool usage, and operations.

G-code is often the final output of a **CAM (Computer-Aided Manufacturing)** workflow and serves as a direct interface between design (CAD) and fabrication.

---

## 📐 What Can G-code Do?

- Move machine tools along X, Y, and Z axes
- Control feed rates, spindle speeds, coolant, etc.
- Switch tools and manage tool paths
- Engage specific functions (e.g., drilling, cutting, heating, probing)
- Pause or stop operations based on conditions
- Embed metadata or subroutines (in more advanced versions)

---

## 🧠 G-code vs Similar Languages

| Language         | Used In                      | Abstraction Level | Human Readable | Standardized | Typical Use                          |
|------------------|-------------------------------|-------------------|----------------|---------------|--------------------------------------|
| **G-code**        | CNC machines, 3D printers     | Low               | Yes            | RS-274        | Machining, fabrication               |
| **HPGL**          | Plotters                      | Medium            | Semi           | HPGL          | Pen plotting                         |
| **PostScript**    | Printing                      | High              | No             | Yes           | 2D printing, typesetting             |
| **STEP-NC**       | Next-gen CNC control          | High              | Partially      | ISO 10303-238 | Feature-based CNC (not widely used)  |
| **Marlin**        | Firmware using G-code variant | Varies            | Yes            | Semi          | 3D printers (RepRap, Creality, etc.) |
| **Toolpath APIs** | CAM software                  | High              | No             | Varies         | Internally for generating G-code     |

---

## 🔢 Example G-code Commands (NO CODE YET)

- `G0` – Rapid movement
- `G1` – Linear movement at a controlled feed rate
- `M3` – Spindle on (clockwise)
- `M5` – Spindle off
- `G28` – Return to home position

> These will be explored in future files.

---

## 🧩 Key Concepts

- **Modal Commands**: Some commands remain active until changed (e.g., G1 stays in effect).
- **Coordinate Systems**: Machines may use absolute (`G90`) or relative (`G91`) positioning.
- **Machine Variants**: Not all machines implement G-code the same way. Some have vendor-specific codes or macros.
- **Tool Offsets and Compensation**: Critical in multi-tool setups.
- **Preprocessors/Postprocessors**: Modify G-code to fit specific machines or workflows.

---

## ⚙️ Standards and Extensions

- **RS-274-D**: The most common flavor of G-code.
- **ISO 6983**: International standard based on RS-274.
- **Vendor Extensions**:
  - Marlin (3D printers)
  - GRBL (CNC motion controller)
  - Fanuc (widely used in industrial CNC)

---

## 🛠️ Real-World Applications

- CNC Milling & Lathing
- PCB Prototyping
- Laser Cutting
- 3D Printing (FDM, SLA)
- Plasma Cutting
- Engraving
- Pick-and-place robotics

---

## 🧪 Pros and Cons

| Pros                              | Cons                                     |
|-----------------------------------|------------------------------------------|
| Widely supported and mature       | Difficult to debug complex toolpaths     |
| Precise and deterministic         | No built-in abstraction or validation    |
| Can be generated by many CAM tools| Vendor-specific differences               |
| Easy to manually inspect or tweak | Steep learning curve for complex tasks   |

---

## 🔍 Future Topics (Each Will Get Its Own Note)

- G-code basics (movement, tool control)
- G-code for 3D printing
- Machine-specific variants (Fanuc, Marlin, GRBL, etc.)
- Writing and debugging custom G-code
- Post-processing G-code for optimization
- Safety and toolpath verification
- Visualizing G-code
