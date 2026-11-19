# Parasolid (Kernel-native CAD exchange)

Parasolid is a geometric modeling kernel used by several CAD systems (for example, Siemens NX, Solid Edge, and historically SolidWorks workflows).  
`x_t`/`x_b` are the most common Parasolid exchange files, but this is still primarily **kernel-native** exchange, not a full cross-kernel neutral standard.  
If you are implementing from scratch, the practical implementation note is [[Parasolid Data Format]].

---

## 🧠 Overview

Parasolid can be treated as:

- a **modeling kernel** API inside CAD software, and
- a family of file formats for moving Parasolid-native models.

It is usually faster and more faithful than general neutral exchange when both sides are Parasolid-based.

---

## 📦 File variants you’ll see

- `.x_t` — Parasolid text format (human-readable text)
- `.x_b` — Parasolid binary format
- `.xmt_txt`, `.xmt_bin` — commonly listed additional Parasolid text/binary variants in some CAD tools

In common practice:

- `.x_t` is easier to diff/debug and is often larger.
- `.x_b` is usually smaller and faster, but not human-readable.
- Both carry the same modeled geometry content for the same source, just in different encoding.

---

## 🔍 Where Parasolid fits vs open neutral formats

| Use case | Parasolid (`x_t` / `x_b`) | STEP (`.step` / `.stp`) | IGES (`.igs` / `.iges`) | STL (`.stl`) |
|---|---|---|---|---|
| Best when both tools share Parasolid kernel | ✅ | ⚠️ (works but not always optimal) | ⚠️ | ❌ |
| Cross-vendor portability (general CAD mix) | ❌ | ✅ | ✅ (legacy) | ✅ (mesh only) |
| Editability downstream | High (within Parasolid chain) | High (if model structure/import supports it) | Often reduced | Very low |
| Metadata/assembly fidelity | High in same ecosystem | High (good for broad interchange) | Medium | Very low |
| Human readability | `.x_t` is text, `.x_b` not | Text (AP-specific) | Text | Text/binary mesh |

If both ends are Parasolid-based, this format can preserve more behavior than neutral translation routes.

---

## ⚠️ Practical caveats

- Parasolid formats are not the usual “universal neutral exchange” path for mixed-kernel fleets.
- Some vendors’ docs state that Parasolid is *proprietary* and not intended as a universal cross-kernel interchange language.
- You can export/import it from many CAD tools only if that tool provides a Parasolid translator.
- If your receiver is not Parasolid-native, test both `.x_t` and `.x_b` exports because importer behavior can vary by version.

---

## 🧱 What you get in practice

Useful characteristics:

- B-Rep solids/sheets/wire bodies and related topology
- better fidelity in parasolid-to-parasolid exchange than many neutral hops
- usually direct support for assembly structure in Parasolid workflows

Typical pitfalls:

- version mismatches (`x_t/x_b` generation from older/newer Parasolid revisions)
- hidden/suppressed component handling (some apps ask to resolve before export/import)
- naming/attributes may be mapped differently outside native Parasolid stack

---

## 🛠️ Where to get/export Parasolid data

Typical sources:

- Parasolid translators inside Parasolid-aware CADs (NX/Solid Edge/other)
- Import/export bridges in other CAD apps (AutoCAD Parasolid translator, SOLIDWORKS translator options, etc.)
- Third-party conversion tools when direct app-to-app path is unavailable

Because these are implementation-dependent, workflow choice is usually:

1. Export native Parasolid file from the source CAD
2. Open directly in Parasolid-compatible destination
3. If that fails, fallback to STEP (often AP214/AP242 depending on target)

---

## 📚 Related notes

- [[STEP]] (general neutral path)
- [[IGES]] (legacy fallback)
- [[STL]] (mesh-only workflows)
- [[STEP Data Format]] (if you are planning programmatic exchange)
- [[Parasolid Data Format]]

---

## 📚 Further reading

- Open Cascade forum note that `.x_t` is Parasolid proprietary and primarily kernel-oriented (not a universal kernel-agnostic exchange target).
- Autodesk Parasolid translator references for supported Parasolid versions and types.
- CAD app manuals for Parasolid translator support matrices and version limitations.
