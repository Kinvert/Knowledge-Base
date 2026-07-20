# paraFoam

**paraFoam** is an OpenFOAM helper command that launches a case directly into ParaView with metadata and mesh/field connectivity already prepared.

---

## 🧠 Overview

`paraFoam` is the standard OpenFOAM-to-ParaView bridge command. It reads a case directory (typically in the current working directory), exports a `.foam` proxy file when needed, and opens ParaView in the expected data context.

You use it when you want a quick visual sanity check after meshing or a solver run without manually converting everything first.

---

## 🧰 Typical workflow

1. Run a case in an OpenFOAM folder with populated time directories (e.g., `0/`, `<time>/`).
2. From case root, run:

```bash
paraFoam
```

3. In ParaView, apply:
   - mesh visualization filters,
   - patch/zone visibility filters,
   - wall-shear/pressure/velocity plots,
   - and animation for transient studies.

Common optional behavior:

- running from non-default time:
  - choose a specific time field when opening
  - then switch to `latest` as needed.
- using ParaView in distributed or script-driven workflows (less common than GUI use, but available through ParaView tooling).

---

## ✅ Why this is useful

- Fastest visual path for OpenFOAM case inspection.
- Keeps field/mesh names consistent with solver setup.
- No extra manual export step for first-pass checks.

Common use:

- quick convergence checks (`Residual fields`, vortical structures, pressure spikes),
- patch BC sanity checks (wrong type/region IDs show up quickly),
- and geometry or layer quality inspection before long runs.

---

## ⚠️ Notes and limitations

- It is primarily for **visualization**, not conversion quality reporting.
- For scripted/robust pipelines, `foamToVTK` + `paraview` can be more controllable.
- For very large datasets, opening directly may be heavy; consider reduced sampling and case time reduction first.

---

## 🔀 Comparison table

| Tool | Scope | OpenFOAM coupling | Best fit | Cost |
|---|---|---|---|---|
| paraFoam | OpenFOAM case launcher | Native | Fast in-case visualization | Free |
| ParaView | Full visualization platform | Indirect, flexible | Complex visualization pipelines | Free |
| foamToVTK | Data conversion | One-way export | Deterministic batch conversion workflows | Free |
| VisIt | High-performance visualization | External pipeline | Cross-solver workflows and HPC dashboards | Free |
| MeshLab | Mesh inspection/fix | Indirect | STL cleanup and shape checks | Free |
| ParaView Python (`pvpython`) | Automation layer | Script-driven | Headless batch rendering and scripted diagnostics | Free |

---

## 🔗 Related notes

- [[OpenFOAM]]
- [[ParaView]]
- [[CFD]]
- [[snappyHexMesh]]
- [[blockMesh]]
- [[Mesh Generation]]

## 📚 Further reading

- OpenFOAM and ParaView launch guide
- ParaView scripting docs
- OpenFOAM post-processing examples
