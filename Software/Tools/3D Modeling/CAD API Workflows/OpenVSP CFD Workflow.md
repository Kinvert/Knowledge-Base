# OpenVSP CFD Workflow

OpenVSP is an open-source aircraft-geometry workbench built for fast, repeatable, parametric shape creation. Its value is strongest when you want to generate many geometry variants quickly, then push those variants into higher-fidelity solvers and meshers for physics. NASA positions it as a design iteration core, with strong CAD interoperability and automation hooks that fit AI-in-the-loop loops well.

For our use case, OpenVSP is not the final solver. It is the geometry factory in a larger chain:

- Geometry design space definition
- Export or API handoff
- External meshing and solver
- Objective extraction + back-propagation into variables

That distinction keeps the tool stack honest and avoids over-trusting one system for everything.

---

## ✅ What OpenVSP is good at

OpenVSP has first-class support for aircraft-like component families:

- Wing and winglet families (planform, twist, sweep, dihedral, airfoils)
- Fuselage lofts and pods
- Nacelles, pylons, props, rotors, and engine-like arrangements
- Parametric design variable studies and batch variant generation
- Lightweight export workflows into external CAD/mesh/simulation ecosystems

The docs describe this as a conceptual to pre-CAD bridge rather than a replacement for full mechanical CAD kernels.

Useful links:
- OpenVSP source and release history: https://github.com/OpenVSP/OpenVSP
- Changelog and release notes: https://github.com/OpenVSP/OpenVSP/blob/main/CHANGELOG.md
- OpenVSP API reference landing page: https://openvsp.org/api_docs/latest/index.html

---

## ⚠️ What OpenVSP is not good at (and where errors sneak in)

OpenVSP is generally not the right first pick for:

- Full mechanical-detail modeling (fine fillets, clearance topology, production feature histories)
- Complex non-aerospace topology where CAD operations need explicit face/edge editability
- Standalone turbulence-resolved external aerodynamics with full wall-layer setup quality

The most common failure mode in external flow studies is assuming OpenVSP geometry fidelity is already enough for CFD-ready boundary layers. It is a geometry front-end, not the wall-model strategy, far-field domain logic, or turbulence numerics by itself.

---

## 📥📤 Import and export interface (practical map)

### Import entry points

The File I/O API includes explicit import primitives and format support for:

- OpenVSP model files and mesh-like geometry payloads
- Common triangulated and point-style inputs used in geometry interchange
- Airfoil profile-style data used by parametric workflows

OpenVSP import support is usually surfaced through:

- GUI import actions
- `ImportFile(...)` in C++/Python API

If an incoming geometry is a triangle soup (not parametric clean B-Rep), quality checks are usually needed before meshing:
- Normals and orientation
- Gaps and non-manifold edges
- Scale and axis orientation consistency

### Export entry points

OpenVSP exports are often split by downstream role:

- CFD-ready surface triangulation exports for meshing chains
- CAD-level shape exchange formats for higher-end cleanup/repair tools
- Airfoil export formats for profile libraries and design scripting

The official file API and enum docs list explicit format selectors, including:

- `EXPORT_STL`, `EXPORT_OBJ`, `EXPORT_PLOT3D`-family styles
- `EXPORT_GMSH`
- `EXPORT_IGES`, `EXPORT_STEP`

That means practical interoperability is broad, but consistency still depends on downstream readers.

Useful links:
- File I/O API group: https://openvsp.org/api_docs/latest/group___file_i_o.html
- API defines/enums: https://openvsp.org/api_docs/latest/_a_p_i_defines_8h_source.html
- Latest API by version: https://openvsp.org/api_docs/3.47.0/index.html
- 3.42 API index (older branch parity check): https://openvsp.org/api_docs/3.42.0/group___geom.html

---

## 🤖 Automation path (where OpenVSP gets powerful fast)

### Scripted execution

The workflow stack includes CLI and API integration:

- AngelScript entry: `.vspscript` (GUI + headless support patterns)
- `vspscript` launcher usage for scripted batch mutation/rendering paths
- Python API module (`openvsp`) for direct in-code geometry operations
- Python runtime config hooks for disabling graphics/facade when headless

Common automation patterns:
- load a `.vsp3`
- apply parametric tweaks via IDs
- run geometry updates
- export surface/CAD
- continue mesh stage in external solver tooling

### API mechanics that matter in pipelines

The API docs expose analysis and state APIs that are more important in automation than in manual use:

- `ImportFile` / `ExportFile` for deterministic file handoff
- `ComputeCFDMesh` for in-tool meshing stages
- control hooks around CFD mesh source and output naming
- scripted parameter updates and design-variable replay

Python config behavior is useful for CI/compute environments:
- disable graphics
- disable facade where needed

If you need to run `openvsp` in a separate process (facade mode) or run multiple concurrent instances, use `LOAD_FACADE`/`LOAD_MULTI_FACADE` patterns in [[OpenVSP Facade API]].

Key links:
- API overview: https://openvsp.org/api_docs/latest/group___geom.html
- API enumerations and types: https://openvsp.org/api_docs/latest/group___enumerations.html
- Python API: https://openvsp.org/pyapi_docs/latest/openvsp.html
- Python config: https://openvsp.org/pyapi_docs/latest/openvsp_config.html

Representative automation skeleton:

```python
import openvsp_config
openvsp_config.LOAD_GRAPHICS = False
import openvsp as vsp

vsp.ReadVSPFile("baseline.vsp3")
# mutate geometry params
vid = vsp.FindParm("WingGeom", "Wingspan")  # example pattern
vsp.SetParmVal(vid, 3.5, 0)
vsp.UpdateGeom()
vsp.ComputeCFDMesh()
vsp.ExportFile("wing_export.stl")
```

---

## 🌊 CFD integration styles with OpenVSP

### 1) Native [[VSPAERO]]

[[VSPAERO]] in OpenVSP is useful for very fast screening and trend direction. It is usually the first stage for sanity checks and low-cost ordering of designs before expensive meshing/solvers.

Good for:
- early lift/drag trend ranking
- geometry family debugging
- very fast loop resets

Limited for:
- detached flow and separated transonic or low-Re transitional regimes at production fidelity
- boundary-layer-sensitive ranking where RANS/LES or wall models matter

### 2) Built-in CFD mesh artifacts + external mesh refinement

OpenVSP’s internal CFDMesh export path is valuable as a deterministic first surface step, but often not the final production volume mesh. Typical flow:

1. Generate surface mesh/geometry from OpenVSP
2. Export to STL/triangulated or structured style
3. Refine with dedicated mesh system (snappyHexMesh, etc.)
4. Rebuild patches, boundary layers, wake control, solver regions

### 3) OpenFOAM-oriented external stacks

OpenFOAM integration is usually external by design. You pass geometry/meshes onward into OpenFOAM-focused preprocessors and solvers.

Most common practical paths:
- OpenVSP surface export -> STL/P3D -> `snappyHexMesh`
- OpenVSP workflows in DAFoam with `custom_vsp3` or converted intermediate mesh representations
- OpenVSP + `plot3dToFoam` routes with boundary patch reconstruction in OpenFOAM setup scripts

Important note:
OpenVSP does not replace OpenFOAM mesher/solver setup for final confidence-level results. It is geometry truth source, not a finished CFD stack.

### 4) Optional high-quality external mesher branch (free tools, CPU-first, GPU-aware)

For difficult geometry families, you can add a toggleable branch after OpenVSP export:

1. Export both `STEP` (downstream repair/rebuild) and `STL` (simple surface handoff).
2. Run geometry QA (manifoldness, normals, thin-gap checks, duplicate faces).
3. Mesh in an external free mesher:
   - `Gmsh` with scripted fields + boundary layer strategies for repeatable parametric sweeps.
   - `cfMesh`/SALOME route when your pipeline already requires their mesh conventions.
4. Convert/import into the solver stack (`convertToFoam`, `ideasUnvToFoam`, etc. as required).
5. Preserve only branch-internal mesh artifacts (`mesh_settings.json`, geometry hash, dict snapshots) so bad cases are reproducible.

Practical GPU reality check:

- There is no clearly production-ready, free and mature OpenVSP-specific GPU mesher with full BL control today.
- Free stack acceleration is typically CPU-parallel, not native CUDA meshing. In that context, Gmsh is a practical choice with OpenMP threading support.
- Use GPU acceleration primarily in later stages (solver kernels / staged offload experiments), not as the default mesh-creation gate.

Known integration examples:
- DAFoam example and docs: https://dafoam.github.io/
- DAFoam repository: https://github.com/mdolab/dafoam
- AI-loop and ADODG3 wing optimization docs: https://dafoam.github.io/ai-agent-aircraft.html
- OpenVSP and OpenFOAM worked example (community): https://flow.byu.edu/FLOWUnsteady/examples/openvsp-aircraft/
- Early community discussion around OpenVSP + OpenFOAM path: https://groups.google.com/g/openvsp/c/4TonJs5oO-Y

---

## 🛠️ What configuration classes OpenVSP can model well

| Class | Practical strengths | Common caveats | Best mesh target |
|---|---|---|---|
| Wings / wing sets | High parametric sweep quality and profile editing | Tip devices, flap gaps/clearances are simplified unless manually built | Use external mesher for local edge refinement |
| Fuselage / pods | Fast loft assembly and fast layout | Local surface micro-details are coarse compared with full CAD | Retopologize complex local features downstream |
| Empennage and tails | Good for configuration family studies | Joints and control-surface transitions need attention | Keep geometric continuity checks before CFD |
| Engine/pylon/prop assemblies | Useful for system-level aerodynamic placement studies | Blade/clevis-level detail typically abstracted | Export and re-resolve in solver preprocessing |
| Propellers | Parametric representation for speed/geometry studies | Nonlinear near-wall rotor wake setup still solver-side | Use dedicated rotor/rotorcraft mesher policy |

---

## 🧨 Limitations that cost time if ignored

- Parameter IDs are model/topology sensitive; design-variable replay should be version-gated.
- Importing low-quality triangulations can silently propagate non-manifold issues.
- STL/triangulation conversions can produce orientation surprises without consistent axis conventions.
- Patch naming/region naming should be standardized immediately after geometry export, not after meshing.
- Geometry-based design loops can overfit to solver artifacts if mesh policy changes before CAD design policy stabilizes.

Operational hard rule:
- Freeze geometry variable semantics before changing mesh-policy semantics.

---

## 📊 OpenVSP vs alternatives (comparison chart, 5+ entries)

| Stack | Geometry creation quality | CFD handoff path | Wall-layer control | Scriptability | Typical use |
|---|---|---|---|---|---|
| OpenVSP + snappyHexMesh (OpenFOAM) | High for aircraft families | STL/tri -> mesher | Strong (mesh/patch control in mesher + BC setup) | High | Production-like external flow loop |
| OpenVSP + DAFoam | High + design-var coupling | `custom_vsp3`, generated preprocessing chain | Strong if workflow includes consistent patch controls | High | Adjoint/optimization and AI-in-loop |
| OpenVSP + pyHyp + plot3dToFoam | Medium to high | OpenVSP surface -> pyHyp/plot3D | Medium (depends on conversion quality) | Medium | Research workflows with mixed structured/unstructured needs |
| OpenVSP + native VSPAERO | Good for rapid checks | Internal analysis path | Limited | Medium | Screening and quick ranking |
| OpenVSP + Salome/Gmsh | High but higher manual overhead | STL/P3D -> generic CAD/mesh pipeline | High if mesher is set properly | Medium | Non-OpenFOAM ecosystem integration |
| Pure OpenVSP (no external mesher) | Excellent rapid shapes | Internal only | Weak | High | Pre-optimization geometry ideation |
| XFOIL-only 2D chains | N/A for full 3D geometry | Airfoil profile only | Low | High | Preliminary 2D section trending |

---

## 🧭 Practical implementation pattern for your loop (RC + race-vehicle framing)

- Use OpenVSP as the source-of-truth for geometry parameterization.
- Keep a strict naming discipline:
  - `geom_id`, `variant_id`, `mesh_profile`, `run_tag`, `Re` or `U_ref`.
- Export from OpenVSP in deterministic filenames per variation.
- Immediately run pre-mesh geometry quality checks before solver launch.
- Mesh externally with a controller tuned for:
  - wall-normal resolution policy
  - farfield and wake growth
  - patch grouping for solver BCs
- For OpenFOAM solver choice:
  - use quick low-cost flow screening first
  - move to stronger transient/pressure-velocity coupling for separated/transient cases
- Parse solver outputs into a consistent score card before feeding the next design iteration.

This order has worked in practice because it prevents geometry and meshing from drifting separately.

---

## 🔗 Source anchors used in this note

- OpenVSP API docs: https://openvsp.org/api_docs/latest/index.html
- OpenVSP File I/O API: https://openvsp.org/api_docs/latest/group___file_i_o.html
- OpenVSP enumerations: https://openvsp.org/api_docs/latest/group___enumerations.html
- OpenVSP geometry API grouping: https://openvsp.org/api_docs/latest/group___geom.html
- OpenVSP Python docs: https://openvsp.org/pyapi_docs/latest/openvsp.html
- OpenVSP Python config docs: https://openvsp.org/pyapi_docs/latest/openvsp_config.html
- OpenVSP source code: https://github.com/OpenVSP/OpenVSP
- OpenVSP API changelog context: https://github.com/OpenVSP/OpenVSP/blob/main/CHANGELOG.md
- OpenFOAM geometry-to-mesh references:
  - https://doc.openfoam.com/2606/tools/pre-processing/mesh/generation/snappyhexmesh/geometry/
  - https://doc.openfoam.com/2306/tools/pre-processing/mesh/generation/snappyhexmesh/triSurfaceMeshes/trisurfacemesh/
  - https://www.openfoam.com/documentation/user-guide/4-mesh-generation-and-conversion/4.4-mesh-generation-with-the-snappyhexmesh-utility
- DAFoam pages:
  - https://dafoam.github.io/ai-agent-aircraft.html
  - https://dafoam.github.io/user-guide-adodg3.html
  - https://github.com/mdolab/dafoam
- Gmsh mesh references:
  - https://gmsh.info/dev/doc/texinfo/gmsh.html
- OpenFOAM meshing + offload context:
  - https://openfoam.org/release/2-3-0/foamyHexMesh/
- Community OpenVSP/CFD workflow example: https://flow.byu.edu/FLOWUnsteady/examples/openvsp-aircraft/
- OpenVSP community thread about OpenFOAM chaining: https://groups.google.com/g/openvsp/c/4TonJs5oO-Y
