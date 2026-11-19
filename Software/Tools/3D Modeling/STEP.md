# STEP: Neutral CAD/PMI Exchange (ISO 10303)

## 🧠 Overview

**STEP** (`.step`, `.stp`, or sometimes `.p21`) is the standard product-data exchange format for CAD models under the ISO 10303 family.

It is usually the correct answer when you need:

- Geometry that stays editable downstream (not only visualized),
- Assembly structure with part hierarchy,
- Units, tolerance, and metadata carry-through,
- Cross-vendor exchange between mechanical systems,
- Long-term archiving and re-import into PLM/CAD workflows.

---

## 🧱 Core terms

- **PART 21** (`ISO 10303-21`) is the most common file wrapper for STEP text files (`.stp` / `.step`).
- **AP (Application Protocol)** is the “capability profile”: a subset/spec of ISO 10303 for a domain.
- **AP203 / AP214 / AP242** are APs commonly seen in mechanical design, with AP242 being the modern consolidation.
- `FILE_SCHEMA(('APxxxx'));` in the header is where the declared AP appears.

Minimal header pattern (simplified):

```text
ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('Example'),'2;1');
FILE_NAME('part.step', '2026-07-27', ('...'), ('...'), 'CAD', 'Exporter', '');
FILE_SCHEMA(('AP242'));
ENDSEC;
DATA;
...
ENDSEC;
END-ISO-10303-21;
```

---

## 🗂️ Deep comparison: AP203 vs AP214 vs AP242

These are not file versions of each other the way people sometimes assume.  
They are separate **Application Protocols** in the STEP ecosystem.

| Item | AP203 | AP214 | AP242 |
|---|---|---|---|
| Primary intent | General mechanical configuration design exchange | Automotive-style richer product data exchange | Managed model-based 3D engineering and long-term CAD interoperability |
| Typical usage in the field | Legacy interoperability target | Transitional standard for improved metadata support | Current mainstream when both sides support it |
| Geometry model | Solid + free-form representation support | Solid geometry + richer color/attribute support | Solid + advanced PMI/annotation support, modernized structure |
| Assemblies | Supports part/component relationships | Stronger support than legacy AP203 variants | Stronger lifecycle and product-structure alignment |
| Colors/appearance | Minimal/limited in older implementations | Better support for colors/layers/naming | Better than AP214 for enterprise semantics in modern toolchains |
| Manufacturing/quality data | Limited | Expanded compared with AP203 | Explicit emphasis on PMI, annotations, metadata-rich data |
| Metadata longevity | Older, more fragile on modern edge features | Mixed interoperability depending on vendor implementation | Better for MBE/MBD workflows and downstream automation |
| Legacy compatibility | Widely supported in old workflows | Still often supported due inertia | Replaced many legacy use-cases, but not always backward compatible |
| Practical recommendation | Keep as a compatibility fallback | Use only if a downstream tool specifically requires it | Use by default for new mechanical exchange projects |

**Important nuance:** “AP242” is usually the right default today, but not always the *only* working answer. If a vendor stack is old (or badly implemented), AP214 can be more reliable than AP242 in practice.

---

## ⚖️ In-depth compare: STEP vs IGES vs Parasolid vs STL

| Format | Neutral standard | Core representation | Editability after import | Metadata fidelity | Best use case | Typical failure modes |
|---|---|---|---|---|---|---|
| `STEP AP203` | ISO 10303 | B-rep solid + topology | Usually editable | Low–medium | Legacy interchange and compatibility | Unit mismatch, naming loss, face ordering drift |
| `STEP AP214` | ISO 10303 | B-rep solid + richer assembly/product fields | Usually editable | Medium | Legacy-to-mid modernization bridges | Different AP interpretation across vendors |
| `STEP AP242` | ISO 10303 | B-rep + modern PMI/MBE semantics | Usually editable | High | Newer enterprise workflows | PMI interpretation differences |
| `IGES` | ISO 10303 predecessor ecosystem | Geometry primitives/surfaces | Typically less editable | Low | Legacy surface/pipework/older CAM where STEP is weakly supported | Missing topology continuity, topology drift, poor param consistency |
| `Parasolid x_t` | Native Parasolid kernel (not neutral) | Parasolid B-rep objects in text | Excellent in Parasolid-native toolchains | High inside Parasolid ecosystem | Same-kernel transfer, high-fidelity updates | Hard to consume outside Parasolid ecosystem |
| `Parasolid x_b` | Native Parasolid kernel (not neutral) | Same model semantics as `x_t`, compact binary | Excellent in Parasolid-native toolchains | High inside Parasolid ecosystem | Large assemblies, better transfer performance | Version/kernel compatibility issues |
| `STL` | Non-neutral mesh format | Triangulated mesh only | Not natively editable as CAD solid | Very low | FDM/SLA printing, quick viz/mesh simulation | Stair-stepped approximation, no units, no solids |

---

## 📌 x_t vs x_b (Parasolid) quick details

- `x_t` = text form
  - Human-readable enough for debugging/diffing
  - Easier with VCS when diffing is needed
  - Larger files and slower parsing
- `x_b` = binary form
  - Smaller and faster to load/save for large models
  - Better for bandwidth and speed-critical exchange
  - Harder to inspect manually

Both are **Parasolid-native** and are usually better than STEP only when the upstream and downstream both speak Parasolid directly.

---

## 🧠 Why IGES still exists (and why it usually shouldn’t be first choice)

IGES is still encountered because:

- Legacy aerospace/machine shops and older CAMs
- Simple curve/surface sharing
- Historical IT standards in regulated manufacturing environments

But IGES generally fails for modern requirements:

- weak assembly/product semantics,
- weaker attribute handling (color/material/name handling is inconsistent),
- weaker support for full parametric intent,
- higher risk of geometry recovery differences after roundtrip.

Use it mainly for legacy interoperability only, not as a new project default.

---

## ✅ When to choose each format

### Use `STEP AP242` if:

- You need a broadly interoperable mechanical exchange baseline,
- You need stable downstream CAD edits,
- You need better metadata handling than old APs,
- You’re implementing MBD/MBE-like workflows (drawings, dimensions, tolerances, attributes).

### Use `STEP AP214` if:

- You are constrained by a supplier or legacy system that is strict on AP214,
- You need more than AP203 but cannot force AP242 compatibility.

### Use `STEP AP203` if:

- The receiver is old or only certifies AP203,
- You can constrain expectations and accept reduced metadata richness.

### Use `Parasolid x_t/x_b` if:

- Both producer and consumer are Parasolid-native (e.g., Parasolid-based CAD/CAM/CAE),
- You want best-fidelity kernel-preserving transfer.

### Use `STL` if:

- You only need mesh output for print/SLA/CNC path prep or rough simulation preprocessing,
- Parametric editability is irrelevant and precision tolerances are already converted.

---

## 🛑 Common conversion gotchas

1. **Units**
   - Verify explicit unit declaration and recipient import units.
2. **Tessellation vs B-rep**
   - Don’t accept mesh output when you need machining edits.
3. **Tolerance stack**
   - If assembly fits become impossible after import, relax tolerance settings or fix downstream join logic.
4. **Feature semantics loss**
   - Fillets/chamfers, suppression trees, and sketch constraints are often not preserved semantically.
5. **PMI and annotation drift**
   - Dimension/annotation meaning may change across vendors, especially when AP choice differs.
6. **Color/material inconsistency**
   - Legacy APs and IGES often lose material/appearance details.
7. **Kernel mapping edge cases**
- Curves/surfaces like periodic/trimmed [[NURBS]] can map differently across translators.

---

## 🧪 How to validate a CAD exchange

Practical rule-of-thumb checks after import:

- Count/compare body and face counts across pre/post import.
- Run volume/mass sanity checks.
- Verify coordinate origin/orientation and unit scaling.
- Validate topology: sheet bodies, open edges, inverted normals.
- Diff metadata fields you care about (part numbers, revision IDs, drawing references).

For critical programs, include a **golden fixture**:

- same model exported in multiple APs,
- same import path repeated each CI run,
- acceptance criteria with geometric and metadata asserts.

---

## 🧭 Relation to surrounding notes

- [[CAD APIs]] (workflow and scriptable exports)
- [[IGES]] (legacy CAD exchange)
- [[STL]] (mesh output / 3D print)
- [[Parasolid]] (kernel-native model transfer)
- [[Parasolid Data Format]]
- [[CAD Data Exchange]] (broader interchange topic)
- [[3D Printing]] (mesh fabrication path)
- [[STEP Data Format]]

---

## 📚 Further reading

- ISO 10303 family overview and implementation notes from vendor and platform documentation
- OpenCascade, FreeCAD, and CAM/CAD vendor export guides for AP choice and caveats
- Siemens Parasolid transfer guides (`.x_t` / `.x_b`) for kernel-native details
- NIST or tooling documentation for PMI/MBE exchange expectations (AP242)

Use AP242 as the default unless a downstream constraint forces AP214 or AP203.
