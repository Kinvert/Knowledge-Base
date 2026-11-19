# StarCraft map file formats

RTS tools typically encounter three map data layers:

- container format (archive/folder and distribution packaging),
- static spatial/semantic layers (tiles, regions, scripts, triggers),
- and runtime protocol fields extracted during play.

These layers are often all present in the same game ecosystem but live in different file paths and parsers.

---

## Why map-format choice matters

If you only need fast pathfinding on a fixed map, you can extract a static occupancy graph once and ignore the editor runtime representation.
If you need editor workflows, bot scripting, or deterministic replays, you usually need map-package and metadata formats too.
For ML/AI workflows (open-loop and re-play analysis), protocol-level formats often matter more than the package format itself.

---

## StarCraft II formats

### `.SC2Map` (published map archive)

Community documentation describes `.SC2Map` as an archive-like package:

- `.SC2Map` is treated as a packaged map for publishing and loading.
- `.SC2Components` is the unarchived folder-equivalent layout.

The protocol also treats local maps as `.SC2Map` paths or raw bytes:

- `CreateGame` supports `LocalMap` with `map_path` and `map_data`.
- `local_map_path` in game info and API metadata includes `.SC2Map`.
- `GameInfo` uses `StartRaw` for startup map layer data once the match is created.

Source claims to rely on:
- local map file/path selection in protocol (`LocalMap`), and
- `local_map_path` and `StartRaw` in game metadata.

### `.SC2Components` (unarchived working form)

`.SC2Components` is the “unzipped” file tree equivalent:

- map/project folders can be opened and edited as plain source-like files.
- SC2 editor export tools can produce component folders directly.
- common file types include `.SC2Layout`, `.SC2Lib`, `.SC2Lighting`, `.xml`, `.dds`, `.wav`, etc.

Typical SC2 map folders from editor docs include:

- `Base.SC2Data`, `Base.SC2Mod`, `Base.SC2Campaign` style roots
- `Base.SC2Data/GameData.xml` for catalog wiring
- trigger/lib assets and layout files

### SC2 map assets and scripts as file-level semantics

When you parse `.SC2Components`, you usually see a mix of:

- script entry points (for example `scripts/main.galaxy`),
- Data module catalogs and XML,
- UI XML/layout files,
- terrain and unit metadata files.

The advantage is diff-able, searchable source control and straightforward integration with custom tooling.

---

## StarCraft II runtime map layers (protocol side)

At runtime, AI frameworks usually use protocol structs rather than raw `.SC2Map` bytes:

- `StartRaw` (static):
  - `pathing_grid` (walkable bitplane),
  - `placement_grid` (buildability),
  - `terrain_height` (heightfield),
  - map extents and play areas,
  - unit start locations.
- `ObservationRaw` (dynamic):
  - unit list with per-frame attributes,
  - visibility map and creep-like overlays.

`ImageData` encoding is explicitly binary:

- `bits_per_pixel`,
- `size` (width/height),
- `data` bytes where payload size is `width * height * bits_per_pixel / 8`.

This is the most stable route for performant AI pipelines because it avoids editor-specific syntax and stores exactly what the simulator uses.

---

## StarCraft I / Brood War (`.SCM` / `.SCX` with CHK)

`CHK` is a classic binary chunk format:

- the map scenario is commonly stored at `staredit/scenario.chk` within older map containers,
- each chunk has an 8-byte header:
  - `u32` name tag (e.g., `"MTXM"`, `"MASK"`, `"UNIT"`),
  - `u32` payload size (bytes),
  - followed by payload bytes.

Common CHK chunks used by terrain/state parsers:

- `DIM`: map dimensions in tiles.
- `MTXM` / `TILE` / `ISOM`: terrain tile data.
- `MASK`: fog/visibility layer.
- `UNIT`: placed units.
- `MRGN`: named/rectangular map regions.
- `TRIG`: trigger/script-like behavior entries.

Important parser behavior from CHK specifications:

- the same section format is reused across many SC1/SCBW variants,
- invalid/unknown sections may be safely skipped by tolerant readers,
- duplicate sections can override earlier definitions.

---

## Command & Conquer map and mission formats

Command & Conquer variants often split map semantics across:

- map terrain data in binary mission/map container formats,
- unit/trigger logic in plain-text configuration sections.

### Mission `.BIN` + INI sections

Common mission metadata sections include:

- `[Structures]`, `[Units]`, `[Infantry]`,
- `[Triggers]`, `[CellTriggers]`, `[TeamTypes]`, `[Waypoints]`,
- optional `[Base]` AI rebuild order.

This is usually a text-intrusive layer where the parser can build:

- pre-placed map entities,
- pathing and team AI behavior scripts,
- event/action systems for dynamic gameplay flow.

### MIX-like terrain graphics containers

Territory graphics in older C&C ecosystems are often stored in asset containers with:

- file-count + body-length header,
- index records containing file ID, start offset, and size,
- tile graphics referenced by 2-byte terrain cell entries in maps.

Maps are frequently fixed-size tile grids (classically 64×64 squares), and each cell can encode compact terrain tile selectors.

---

## Practical decoding notes (bytes + indexing)

### CHK chunk parser sketch

```python
import struct

def parse_chk_chunks(buf: bytes):
    pos = 0
    while pos + 8 <= len(buf):
        tag = buf[pos : pos + 4].decode("ascii", errors="replace")
        size = struct.unpack_from("<I", buf, pos + 4)[0]
        body0 = pos + 8
        body1 = body0 + size
        if body1 > len(buf):
            break
        body = buf[body0:body1]
        yield tag, body
        # CHK can contain padding/alignment edge cases; real parsers usually validate
        pos = body1
```

### Fast map-space lookup strategy

For simulation loops:

- parse all static binary chunks into compact integer maps once at load,
- keep an occupancy cache keyed by `(cell_index, layer, variant)`,
- defer dynamic overlays to unit/event updates,
- convert to regional abstractions for low-latency high-level pathing.

---

## Comparison chart: SC maps by format family

| Format family | Container type | Static map state | Dynamic state capture | Parsing complexity | Best fit |
|---|---|---|---|---|---|
| SC2 `.SC2Map` | archive package (editor/publish format) | yes | yes (through protocol) | medium | sharing, deployment, custom map mods |
| SC2 `.SC2Components` | folder tree | high | medium (scripts/events present) | low | diffing, custom pipelines, tooling |
| SC2 `StartRaw` / observation | protocol messages | high | high | medium | bots and real-time AI |
| SC1/SCBW CHK (`.SCM`/`.SCX`) | chunked binary (`scenario.chk`) | very high | medium | high (chunk table decoding) | deterministic classic RTS parsing |
| C&C mission INI+BIN | text + binary split | high | high (triggers/events separate) | medium | rule-heavy map + AI scripting |
| C&C MIX + map grid | asset container + tile grid | high | low | high (offset/index traversal) | terrain analysis and tile editing |

---

## Use this for future notes

- Compare with `[[StarCraft II map representation]]` for runtime vs static-vs-visible state.
- Next likely note: `[[Command and Conquer map formats]]` to capture C&C-specific chunk/INI edge cases.
- Keep `[[Game terrain abstraction]]`-style notes aligned with:
  - walkability mask,
  - buildability mask,
  - visibility-state transitions,
  - connected components/choke-point graphs,
  - pathing cache and influence overlays.

---

## External references

- SC2Mapster Working With SC2 (SC2Map and SC2Components): `https://sc2mapster.github.io/mkdocs/setup/`
- SC2 Editor asset export and file format list: `https://s2editor-guides.readthedocs.io/New_Tutorials/07_Lessons/083_Export_Game_Assets/`
- SC2API protocol (`RequestCreateGame`, local maps, map info): `https://raw.githubusercontent.com/Blizzard/s2client-proto/master/s2clientprotocol/sc2api.proto`
- SC2 API `GameInfo` fields: `https://blizzard.github.io/s2client-api/structsc2_1_1_game_info.html`
- SC2 protocol image struct (`bits_per_pixel`, buffer size rule): `https://raw.githubusercontent.com/Blizzard/s2client-proto/master/s2clientprotocol/common.proto`
- SC2 protocol raw units and fields: `https://raw.githubusercontent.com/Blizzard/s2client-proto/master/s2clientprotocol/raw.proto`
- SC2 protocol query API (`Placement`/`Pathing`): `https://raw.githubusercontent.com/Blizzard/s2client-proto/master/s2clientprotocol/query.proto`
- CHK format (StarCraft): `https://www.starcraftai.com/wiki/CHK_Format`
- C&C mission format notes: `https://moddingwiki.shikadi.net/wiki/Command_%26_Conquer_Mission_Format`
- C&C map guide / MIX internals: `https://gamefaqs.gamespot.com/pc/196957-command-and-conquer/faqs/1836`
