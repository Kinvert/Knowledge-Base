# StarCraft II Map Representation

`StarCraft II` map modeling for AI is usually split into three layers: static terrain topology, dynamic world state, and visibility/knowledge state.

---

## Overview

At runtime, bots usually need two families of data:

- **Static map layers** loaded at game start from the protocol `GameInfo`/`StartRaw` snapshot.
- **Dynamic world state** loaded from each observation via repeated units and map state.

This split is explicit in the SC2 protocol: `RequestGameInfo` for static setup and `RequestObservation` for per-tick state, with `GameInfo` fields for map topology and `ObservationRaw` containing units + map state [[GameInfo](https://blizzard.github.io/s2client-api/structsc2_1_1_game_info.html)], [[protocol](https://raw.githubusercontent.com/Blizzard/s2client-proto/master/s2clientprotocol/sc2api.proto)].

---

## Core protocol structs to mirror

### `GameInfo` / `StartRaw`

`StartRaw` gives map grid layers and start data in `GameInfo`:

- `width`/`height` / `playable_area` (`RectangleI`).
- `pathing_grid` (`1` bit bitmap).
- `terrain_height` (`1` byte bitmap).
- `placement_grid` (`1` bit bitmap).
- `start_locations` list and `local_map_path` (`.SC2Map` path).

These fields are documented both in the generated API struct docs and the protocol definition [[GameInfo](https://blizzard.github.io/s2client-api/structsc2_1_1_game_info.html)], [[raw.proto](https://raw.githubusercontent.com/Blizzard/s2client-proto/master/s2clientprotocol/raw.proto)], [[sc2api.proto](https://raw.githubusercontent.com/Blizzard/s2client-proto/master/s2clientprotocol/sc2api.proto)].

### Observation/units/message-level world model

`ObservationRaw` includes:

- `units` list (`repeated Unit`) with per-unit tags, positions, hp/shields/energy, owner, orders, etc.
- `map_state` with `visibility` and `creep` overlays.
- Other per-frame overlays/events (effects, radar, dead units list).

`Unit` message includes an `Alliance` enum and a `DisplayType` enum (`Visible`, `Snapshot`, `Hidden`, `Placeholder`) for unit presentation state [[protocol](https://raw.githubusercontent.com/Blizzard/s2client-proto/master/s2clientprotocol/raw.proto)].

---

## Bit-level byte handling: what the bytes mean

`ImageData` is generic across SC2 APIs and tells you exactly how to decode image planes:

- `bits_per_pixel`
- `size` as `Size2DI` (`x`, `y`)
- `data` payload as raw bytes, where buffer size is `width * height * bits_per_pixel / 8` [[common.proto](https://raw.githubusercontent.com/Blizzard/s2client-proto/master/s2clientprotocol/common.proto)].

`StartRaw` in SC2 protocol sets:

- `pathing_grid`: 1-bit passability.
- `terrain_height`: 1-byte terrain elevation.
- `placement_grid`: 1-bit buildability.

[[raw.proto](https://raw.githubusercontent.com/Blizzard/s2client-proto/master/s2clientprotocol/raw.proto)]

### Practical decode pattern (compact + cache-friendly)

For each map layer you want fast occupancy checks on:

```python
# pseudo-structure for a read-optimized snapshot
from dataclasses import dataclass

@dataclass
class Bitplane:
    w: int
    h: int
    bpp: int
    bytes_: bytes

def get_cell(lp: Bitplane, x: int, y: int) -> int:
    # map-local coordinates assumed.
    i = y * lp.w + x
    if lp.bpp == 1:
        by = lp.bytes_[i // 8]
        return (by >> (i % 8)) & 1
    off = i * lp.bpp // 8
    mask = (1 << lp.bpp) - 1
    return lp.bytes_[off] & mask
```

Why this matters:

- `bpp=1` packs 8 cells into one byte.
- A `H x W` map keeps memory low when used as `byte/bit` arrays.
- Converting to `uint8` occupancy grids lazily (only on demand) helps when you only need A* for a few regions.

---

## API-level querying patterns for search

`ObservationInterface` exposes the data you need to build AI world models:

- `GetUnits(alliance, filter)` and `GetUnits(filter)` for quick faction + predicate access.
- `GetUnit(tag)` for direct lookup.
- `IsPlacable(point)` / `TerrainHeight(point)` helper checks.
- `GetVisibility(point)` (visibility enum).

The interface docs also clarify visibility and when this interface is valid during callback timing [[observation interface](https://blizzard.github.io/s2client-api/classsc2_1_1_observation_interface.html)].

Query API (outside observation) provides synchronous-but-batch-friendly helpers:

- `PathingDistance(start, end)` (unit-aware, for movement differences).
- Batch pathing with `PathingDistance(vector<PathingQuery>)`.
- `Placement(ability, point, unit)` and batch version.

SC2 API docs explicitly call out batching because these queries are synchronous and effectively blocking [[query interface](https://blizzard.github.io/s2client-api/classsc2_1_1_query_interface.html)].

### Search behavior that works well in practice

This is implementation-level:

1. Use `pathing_grid` for static walkability mask.
2. Add dynamic overlays (temporary blockers from units, destruction state, vision assumptions).
3. Run local search from unit start to target when needed.
4. Cache successful paths by `(start region, end region, unit movement type)`.
5. For repeated large-scale path planning, build a second abstraction graph:
   - regions from connected walkable components,
   - weighted edges around choke points,
   - local refine with grid-level search.

This mirrors standard RTS map abstraction pipelines used in terrain-analysis literature and avoids full-grid search for every query.

---

## Map abstraction terms in literature and practical notes

Useful terms to use in map notes and AI designs:

- walkability mask
- buildability mask
- fog layer / visibility layer / last-seen state
- dynamic occupancy overlay
- connected components
- region decomposition
- choke points / choke graph
- contour tracing + Voronoi / medial axis as region-seed methods
- influence / density-style maps
- pathing cache

Sources that reinforce these terms include:

- Terrain analysis and choke-point decomposition in RTS literature (Perkins, 2010) [[AIIDE 2010](https://ojs.aaai.org/index.php/AIIDE/article/view/12405)].
- BWTA2 speed/accuracy claims and region/chokepoint pipeline [[BWTA2](https://cdn.aaai.org/ojs/12889/12889-52-16406-1-2-20201228.pdf)].
- SC2/Brood War terrain analysis as combinatorial optimization with connected components and map areas [[Taunt](https://arxiv.org/abs/2205.08683)].

---

## SC2 file/container view (how `.SC2Map` fits)

SC2 community docs describe `.SC2Map` as a zip-like archived map package and `.SC2Components` as the unarchived folder form [[SC2Mapster setup](https://sc2mapster.github.io/mkdocs/setup/)].

That gives three practical model layers:

- protocol `GameInfo`/`StartRaw` (runtime API)
- `.SC2Map` archive (editor + distribution)
- extracted `.SC2Components` (direct file access for scripts/assets)

---

## Comparison chart: map representation families

| Representation family | Representation unit | Pathability semantics | Buildability semantics | Visibility / fog model | Typical use |
|---|---|---|---|---|---|
| SC2 `StartRaw`/`GameInfo` | Per-cell bitmap layers | `pathing_grid` bit = passable | `placement_grid` bit = buildable | no fog (static) | bot startup map graph |
| SC2 `ObservationRaw` (`units`, `map_state`) | Repeated units + dynamic map overlays | dynamic occupancy from units + static pathing | dynamic checks via placement logic | map-state visibility + unit `DisplayType` | tactical replanning |
| SC2 feature layers (`Observation.feature_layer_data`) | Dense byte planes | `pathable` plane | `buildable` plane | `visibility_map` and creep/power planes | policy nets / GPU state features |
| SC2 query interface | On-demand pathing & placement tests | unit-aware pathing distance | ability-dependent placement checks | not a persistent map layer | path validation + feasibility checks |
| SC2 file/package (`.SC2Map`, `.SC2Components`) | Archive members (`.SC2Map`) and extracted file tree (`.SC2Components`) | engine interpreted from data | engine interpreted from data | engine interpreted from map + script data | map editing and reproducible pipelines |
| C&C CHK | Chunked binary sections | walkability encoded in map chunk structures | terrain/build flags in tile flags and chunks | engine-specific fog system | legacy AI/map tools |
| C&C mission format (`BIN` + `INI`) | terrain binary + mission descriptor script | terrain section + obstacle metadata | flags from mission terrain + doodad/build rules | engine-managed object/vision model | scripted single-player mission formats |

---

## Notes-to-write hints

- Add a dedicated note for `[[StarCraft II map file formats]]` with one subsection on `.SC2Map` vs `.SC2Components`.
- Add a dedicated note for `[[Command and Conquer map formats]]` covering CHK chunks, mission+INI, and map tilesets.
- Add one note for `[[RTS terrain analysis terms]]` if it is not already there.

---

## External references

- SC2 API game info: https://blizzard.github.io/s2client-api/structsc2_1_1_game_info.html
- SC2 API observation interface: https://blizzard.github.io/s2client-api/classsc2_1_1_observation_interface.html
- SC2 API query interface: https://blizzard.github.io/s2client-api/classsc2_1_1_query_interface.html
- SC2 protocol schema (`raw.proto`): https://raw.githubusercontent.com/Blizzard/s2client-proto/master/s2clientprotocol/raw.proto
- SC2 protocol schema (`sc2api.proto`): https://raw.githubusercontent.com/Blizzard/s2client-proto/master/s2clientprotocol/sc2api.proto
- ImageData encoding (`common.proto`): https://raw.githubusercontent.com/Blizzard/s2client-proto/master/s2clientprotocol/common.proto
- Feature layer channel layout: https://raw.githubusercontent.com/Blizzard/s2client-proto/master/s2clientprotocol/spatial.proto
- SC2Map as archive: https://sc2mapster.github.io/mkdocs/setup/
- SC2 export/components workflows: https://s2editor-guides.readthedocs.io/New_Tutorials/07_Lessons/083_Export_Game_Assets/
- CHK format (chunked SC1 map file): https://www.starcraftai.com/wiki/CHK_Format
- C&C mission format: https://moddingwiki.shikadi.net/wiki/Command_%26_Conquer_Mission_Format
- C&C map guide with tile structure: https://gamefaqs.gamespot.com/pc/196957-command-and-conquer/faqs/1836
- Terrain analysis and choke points (Perkins 2010): https://ojs.aaai.org/index.php/AIIDE/article/view/12405
- BWTA2 baseline: https://cdn.aaai.org/ojs/12889/12889-52-16406-1-2-20201228.pdf
- Taunt for SC1/SC2 map decomposition + connected components: https://arxiv.org/abs/2205.08683
