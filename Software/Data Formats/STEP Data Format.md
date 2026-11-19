# STEP Data Format: code-level generation from C/Zig

This note is focused on what you need to do if you want to emit valid STEP files from source code rather than through a CAD GUI.

---

## 🔗 Related notes

- [[STEP]] (neutral CAD/PMI exchange overview)

---

## 🧠 Key fact: STEP is a schema-constrained graph, not just geometry text

STEPfile (`.step`, `.stp`, `.p21`) is **PART 21** of ISO 10303: a plain-text container for an
`entity_instance` graph that must conform to an **EXPRESS** schema and AP (Application Protocol).

To work at all, your writer must know:

- the exact AP schema (`AP203`, `AP214`, `AP242`, etc.),
- every entity and attribute you emit,
- what is required vs optional,
- ordering constraints from EXPRESS inheritance/where-rules,
- and how references (`#123`) are shared across entities.

It is normal for a CAD kernel to hide this complexity; if writing from scratch, you have to
rebuild it.

---

## 📦 STEP file structure you must target

PART 21 syntax is sectioned text. For common 2002-era files used in almost all CAD workflows:

- Start with `ISO-10303-21;`
- `HEADER; ... ENDSEC;` (mandatory, non-repeatable)
- one or more `DATA; ... ENDSEC;` (mandatory, repeatable per schema)
- end with `END-ISO-10303-21;`
- comments are `/* ... */`
- comments and values: `$` = NULL, `*` = "unset list/collection"

`DATA` entity line format:

```
#1 = ENTITY_NAME(attr1, attr2, #3, 'string', $, (1.0, 2.0, 3.0));
```

Minimal shell that is syntax-valid in many importers (but not a useful model):

```text
ISO-10303-21;
HEADER;
  FILE_DESCRIPTION(('Empty scaffold to test parser'),'2;1');
  FILE_NAME('empty.step','2026-07-27',('your-app'),('your-org'),'writer','1.0','');
  FILE_SCHEMA(('AP242'));
ENDSEC;
DATA;
ENDSEC;
END-ISO-10303-21;
```

For interoperability, a real model usually needs a full product/shape tree (units, context,
representation context, geometry representation, root product definition, etc.), not just a few standalone geometry entities.

---

## 🧬 AP203 vs AP214 vs AP242 in practice

These are not file versions; they are separate APs.

| Property | AP203 | AP214 | AP242 |
|---|---|---|---|
| Intended domain | Early neutral mechanical | Automotive/interchange expansion | Modern managed model-based 3D |
| Geometry target | B-Rep + 2D/3D curves/surfaces | Broader metadata and assembly support | AP203 + AP214 + PMI/MBE-oriented extensions |
| Typical today | Legacy compatibility fallback | Mixed legacy/modern interoperability | Usually best default for new CAD exchange |
| Good for | oldest toolchains | older enterprise CAD chains | new CAD/MBD + lifecycle workflows |
| Practical risk | missing modern fields | inconsistent interpretation across old importers | stricter schema checks can fail if your writer is incomplete |

Practical defaults:

- **AP242** if you control both ends and care about metadata/PMI.
- **AP214** if a legacy receiver rejects AP242.
- **AP203** only when forced by an old consumer.

`FILE_SCHEMA(('AP242'));` is the common minimum knob in header, but your emitted entity graph still must be AP-compliant.

---

## 🧱 Realistic path choices for C/Zig developers

### 1) Use an existing STEP implementation (recommended)

If you can add one dependency, this is always safer than hand-coding all EXPRESS constraints.

- **STEPcode**: generate schema-bound C++ bindings from EXPRESS (`exp2cxx`), then emit via STEPfile APIs.
- **Open CASCADE**: build geometry in OCCT and call `STEPControl_Writer`.
- **SDAI C/C++ (e.g., STEP Tools family)**: schema dictionary + name-based attribute set/get (`sdaiCreateInstanceBN`, `sdaiPutAttrBN`) then write Part 21.

This removes most of the parser-surface pitfalls (ID assignment, WHERE constraints, schema-specific required
constructs, aggregate shapes).

### 2) Write your own emitter (only when needed)

Do this only if you:

- need deterministic formatting across environments,
- need strict control over generated IDs/ordering,
- or are targeting a tiny custom subset.

For everything else, library support + regression tests is cheaper and far less buggy.

---

## 🛠️ Recommended implementation architecture (schema-driven)

Whether you use C++, C, or Zig, you should build the same internal model:

1. **Load/know schema** for one AP.
2. **Create canonical entities in memory** with typed attributes.
3. **Build dependency graph** and assign stable `#id`s.
4. **Topological emit** entities after dependencies are declared.
5. **Serialize** with strict value formatting:
   - enums like `.LENGTH_UNIT.` (with dots),
   - reals normalized (`1.0`, scientific okay),
   - strings quoted and escaped,
   - aggregates in `(a,b,c)` form,
   - refs as `#123`,
   - null `$`, unset `*`.
6. **Validate** by round-tripping:
   parse with an existing writer/reader, then compare geometric and metadata checks.

---

## 📐 C-style minimal emitter sketch (not production complete)

This is intentionally small, so it demonstrates mechanics only: entity allocation, formatting, ordering, and emission.

```c
// step_emitter.h (core idea only)
typedef enum { V_INT, V_REAL, V_BOOL, V_ENUM, V_STRING, V_REF, V_AGG, V_NULL } StepKind;

typedef struct StepValue {
    StepKind kind;
    union {
        int   i;
        double r;
        int   b;
        const char* s;   // enum/symbol/string
        int   ref_id;
        struct { struct StepValue* vals; int n; } agg;
    };
} StepValue;

typedef struct {
    int id;            // auto-generated, starts at 1
    const char* type;   // e.g. "CARTESIAN_POINT"
    const char** attrs; // attr type hints or names (optional in emission)
    StepValue* values;
    int n_values;
} StepEntity;

typedef struct {
    StepEntity* ents;
    int count, cap;
} StepModel;
```

```c
// serialize one value
void emit_value(FILE* out, const StepValue* v) {
    switch (v->kind) {
        case V_NULL: fputs("$", out); break;
        case V_BOOL: fputs(v->b ? ".T." : ".F.", out); break;
        case V_INT:  fprintf(out, "%d", v->i); break;
        case V_REAL: fprintf(out, "%.17g", v->r); break;
        case V_STRING: fprintf(out, "'%s'", escape_string(v->s)); break;
        case V_ENUM: fprintf(out, ".%s.", v->s); break;
        case V_REF: fprintf(out, "#%d", v->ref_id); break;
        case V_AGG:
            fputc('(', out);
            for (int i = 0; i < v->agg.n; i++) {
                if (i) fputc(',', out);
                emit_value(out, &v->agg.vals[i]);
            }
            fputc(')', out);
            break;
    }
}

void emit_entity(FILE* out, const StepEntity* e) {
    fprintf(out, "#%d = %s(", e->id, e->type);
    for (int i = 0; i < e->n_values; i++) {
        if (i) fputc(',', out);
        emit_value(out, &e->values[i]);
    }
    fprintf(out, ");\n");
}
```

What this sketch omits (important):

- string escaping for `'` and control characters,
- line-splitting policy for very long records,
- validation against EXPRESS definitions (required attributes, cardinality, supertypes),
- schema imports + entity inheritance,
- complex entity constraints and WHERE-rule enforcement,
- deterministic sort order for stable IDs across runs.

If your goal is real exchange quality, this minimal emitter is a start, not a done system.

---

## 🔄 Zig version shape (FFI-first strategy)

If you are writing in Zig, either:

1. **Use a C-bound API** from STEPcode/OCCT/SDAI and keep all heavy conformance in that layer.
2. Or keep your own Zig emitter with a C-compatible data model.

For option 1, avoid re-implementing schema logic in Zig:

```zig
const c = @cImport({
    @cInclude("STEPcontrol_Writer.hxx");
    @cInclude("Interface_Static.hxx");
});
```

Then feed geometry into the C++ layer and serialize there.

For option 2, mirror the C struct idea with `std.ArrayList(StepEntity)` and write
a `emit_value()` that outputs exactly the same token rules as C. Keep an explicit `id_counter`.

---

## ✅ Recommended "from scratch but not broken" pipeline

1. Pick AP and schema.
2. Emit a header that passes minimal parser checks.
3. Emit only entities you can prove valid for that AP.
4. Build regression fixtures:
   - write file -> parse with multiple readers -> check load success.
5. Add geometric checks:
   triangle counts, centroid/volume drift, units, face orientation, open edges.
6. Run schema-aware readers in CI:
   at least one strict reader from each major consumer you care about.

---

## ⚖️ If you really want in-code examples, the practical options

### Option A: STEPcode (C++)

- Build:
  - `git clone https://github.com/stepcode/stepcode`
  - `cmake .. -DSC_BUILD_SCHEMAS=ap214` (or point to your EXPRESS file path list)
  - generate + build schema objects
- Use `p21`/`exp2cxx` style APIs to create instances and write via `STEPfile`.

Minimal pattern (from upstream docs) is:

```cpp
Registry registry(SchemaInit);
InstMgr instance_mgr;
STEPfile file(registry, instance_mgr, "", false);
// create instances via generated API / schema constructors
file.WriteExchangeFile("out.step");
```

### Option B: Open CASCADE (C++)

- Construct your shape in OCCT.
- `STEPControl_Writer` writes AP214/AP242 based on static parameter.
- For AP242:

```cpp
Interface_Static::SetCVal("write.step.schema", "AP242DIS");
Interface_Static::SetIVal("write.step.assembly", 2);
STEPControl_Writer w;
w.Transfer(shape, STEPControl_AsIs); // or specific transfer mode
w.Write("out.step");
```

### Option C: SDAI C API (if you have it)

Name-based API avoids hardcoding entity classes:

```c
SdaiSession sess = sdaiOpenSession();
SdaiRep repo = sdaiOpenRepositoryBN(sess, "repo");
SdaiModel mod = sdaiCreateModelBN(repo, "model", "configuration_controlled_design");
SdaiAppInstance p = sdaiCreateInstanceBN(mod, "cartesian_point");
sdaiPutAttrBN(p, "x", sdaiREAL, 1.0);
sdaiPutAttrBN(p, "y", sdaiREAL, 0.0);
sdaiPutAttrBN(p, "z", sdaiREAL, 0.0);
```

Use the SDAI/ROSE docs for exact signatures and error handling flow; C API can differ per vendor and release.

---

## 🆚 In-depth format comparison for your CAD pipelines

| Format | Main purpose | Kernel semantics | Editability after import | Metadata fidelity | Typical failures |
|---|---|---|---:|---|---|
| STEP AP203 | Legacy mechanical exchange | Natively neutral | Usually editable | Low–medium | Missing newer fields, brittle assemblies |
| STEP AP214 | Legacy-to-moderate modern interoperability | Natively neutral | Usually editable | Medium | Nonuniform support in old/new tools |
| STEP AP242 | Neutral + MBE/PMI target | Natively neutral | Usually editable | High | Over-strict readers reject incomplete models |
| IGES | Legacy geometry transfer | Mostly geometry-only | Often weak/edit-lossy | Low | Assembly and topology drift |
| Parasolid x_t | Parasolid-native text transfer | Parasolid kernel objects | Excellent within Parasolid ecosystem | High | Hardly supported outside Parasolid |
| Parasolid x_b | Parasolid-native binary transfer | Parasolid kernel objects | Excellent within Parasolid ecosystem | High | Vendor/version lock-in, opaque diff |
| STL | Triangulated mesh transport | Mesh-only | Not solid-editable | Very low | Unit ambiguity, no topology metadata |

Use STL only when mesh is the deliverable. For editable CAD exchange or machining workflows, STEP is the correct neutral format.

---

## 🚩 Common production failure patterns

- Wrong AP/schema mismatch (header says AP242, payload entities look AP203-ish).
- Missing `FILE_SCHEMA` or wrong ordering in HEADER entities.
- Wrong units: model thinks mm, consumer interprets m.
- Broken reference graph (`#999` forward references where toolchain can't resolve).
- Not setting product definition structure, so import gets a geometry orphan.
- Optional fields omitted where required by your target AP variant.
- String escaping/locale issues in comments and names.
- Writing B-Rep entities without closed loops/topology validity.

---

## 🧭 Practical next steps

1. Start by validating with a known-good minimal model from your target library.
2. Use that as roundtrip baseline.
3. Build your C/Zig abstraction around a stable intermediate graph.
4. Only then, add more entity types and AP-specific metadata.

---

## 🔗 Related notes

- [[CAD APIs]]
- [[Parasolid]]
- [[Parasolid Data Format]]
- [[STEP]]

---

## 📚 Sources used for this note

- [ISO 10303-21 (STEP-file structure and sections)](https://wwws.loc.gov/preservation/digital/formats/fdd/fdd000448.shtml)
- [STEPcode build flow and schema generation (`SC_BUILD_SCHEMAS`, `exp2cxx`)](https://stepcode.github.io/docs/building/)
- [STEPcode executables reference (`exp2cxx`)](https://stepcode.github.io/docs/executables/)
- [STEPcode minimal Part 21 C++ example](https://stepcode.github.io/docs/p21_cpp_example/)
- [Open CASCADE `STEPControl_Writer` API](https://dev.opencascade.org/doc/refman/html/class_s_t_e_p_control___writer.html)
- [Open CASCADE STEP processor docs (`write.step.schema`, `write.step.assembly`)](https://dev.opencascade.org/doc/overview/html/occt_user_guides__step.html)
- [SDAI C programming API](https://www.steptools.com/docs/sdailib/programming.html)
- [SDAI C function reference (`sdaiCreateInstanceBN`, `sdaiPutAttrBN`)](https://www.steptools.com/docs/sdailib/functions.html)
