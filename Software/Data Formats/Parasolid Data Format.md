# Parasolid Data Format

This note focuses on `x_t`, `x_b`, and adjacent Parasolid transfer formats when you are evaluating CAD exchange at the file-format layer.

---

## 🧠 Core reality check

Parasolid file formats are **kernel-native** exchange artifacts tied to the Parasolid modeling kernel.

- They are often used between Parasolid-aware systems and are usually high-fidelity in that context.
- They are *not* the same class of open, stable, language-agnostic schema docs as STEP/IGES.
- In practice, a full “from scratch in plain C/Zig” implementation is usually not realistic unless you have licensed SDK access and API-level support.

That said, workflows can still be reliable if you use the right integration boundary.

---

## 📦 What formats exist in the wild

- `.x_t` — Parasolid text format (human-readable, larger, diff-friendly)
- `.x_b` — Parasolid binary format (compact, faster I/O, opaque)
- `.xmt_txt`, `.xmt_bin` — additional textual/binary Parasolid-adjacent forms reported by several CAD translation tools

Common practical assumptions:

- `x_t` and `x_b` are usually equivalent in geometric content for a given source.
- Both can carry full B-rep solids/sheets/wires for downstream Parasolid-native tools.
- They are usually chosen when both sides are known to support Parasolid well.

---

## ⚠️ Why this can feel “placeholder-y”

If you are trying to write a serializer yourself, this is the hard part:

- public, complete parser/granular grammar docs for all Parasolid entities are typically not freely published;
- behavior and supported attributes vary by Parasolid revision and app translator;
- version negotiation details are often tied to SDK/translator release compatibility.

So for many teams, the safest plan is:

1. **Use a real Parasolid exporter API/SDK** to create `.x_t`/`.x_b`;
2. or **export from a trusted CAD host and treat files as opaque blobs**;
3. only convert to/from Parasolid as a controlled pipeline hop, not as a text manipulation format.

---

## 🧱 Practical implementation strategy in C/Zig (when you do have API access)

### Path A: best-practice (recommended)

Treat Parasolid as an API surface, not a manual textual grammar:

1. Build/receive model in your host format.
2. Use Parasolid API calls to create kernel objects.
3. Serialize through SDK writer API to `.x_t` or `.x_b`.
4. Verify import in target Parasolid-compatible consumer.

### Path B: fallback when direct writer is unavailable

- Export model to a neutral format you can reliably generate yourself (for example STEP AP242),
- import into a licensed Parasolid-enabled intermediary,
- immediately re-export `.x_t`/`.x_b` and proceed in the Parasolid-native chain.

In this path, your "from scratch" surface is only the neutral step, not Parasolid encoding.

---

## 🔎 What code still looks like for compatibility wrappers

Even with API support, robust pipelines usually still include adapter code:

- version pinning (source kernel vs target consumer),
- naming/metadata normalization,
- unit policy (mm vs inch),
- assembly/reference expansion checks,
- automated roundtrip validation in CI.

If you are forcing C/Zig side implementation:

- keep a narrow intermediate model,
- serialize only a tightly controlled subset first,
- run deterministic tests against fixed fixtures (golden files),
- compare body count, edges, faces, volume, and metadata before broadening scope.

---

## 🆚 Compare at a glance (within Parasolid + common mesh/neutral alternatives)

| Format | Type | Target ecosystem | Human readable | Editability after import | Best when | Main tradeoff |
|---|---|---|---|---|---|---|
| `x_t` | Parasolid kernel-native | Parasolid-compatible CAD/CAM/CAE | Mostly yes | High (within ecosystem) | same-kernel exchange, versioned repos | Large files, parser-specific behavior |
| `x_b` | Parasolid kernel-native | Parasolid-compatible CAD/CAM/CAE | No | High (within ecosystem) | large assemblies, speed-sensitive pipelines | opaque binary, harder diffing |
| `xmt_txt` | Parasolid-adjacent text | Parasolid translators | Yes | Medium–high | tool-specific interoperability wrappers | not uniformly supported |
| `xmt_bin` | Parasolid-adjacent binary | Parasolid translators | No | Medium–high | same as above | similarly implementation-specific |
| `STEP AP242` | Open neutral (ISO 10303) | Broad CAD stack | Text-ish | Usually high | vendor-mixed pipelines, enterprise CAD exchange | larger semantic burden on exporters/importers |
| `IGES` | Legacy CAD exchange | Legacy/older systems | Text-ish | Often low | legacy tooling compatibility | weak topology/product semantics |
| `STL` | Triangle mesh | Mesh tools, printers | Usually yes | Very low | manufacturing mesh workflows | no topology/parametric CAD |

---

## ✅ When Parasolid is usually the right choice

Use Parasolid when all of these are true:

- producer and consumer are both Parasolid-capable,
- you prioritize parametric/feature continuity over fully open interoperability,
- you can enforce compatible kernel versions,
- you can test against both old and new translator builds.

Use neutral standards otherwise (especially if downstream is mixed-kernel).

---

## 🔗 Related notes

- [[Parasolid]]
- [[STEP Data Format]]
- [[STEP]]
- [[CAD APIs]]

---

## 📚 Sources used for this note

- [Open CASCADE notes on Parasolid `.x_t` interoperability assumptions](https://dev.opencascade.org/content/it-possible-export-xt-file)
- Autodesk support and user-article references for Parasolid version compatibility issues (e.g., `x_t` import/export edge cases)
- STEP API and neutral exchange references used for cross-chain fallback strategy (for example [`STEPcode`](https://stepcode.github.io))
