# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Overview

This is an Obsidian knowledge base vault containing 842+ interconnected markdown notes on technical topics. The primary purpose is discovering connections between concepts via Obsidian's graph view - notes are LLM-generated summaries with comparisons and basic code snippets, not in-depth learning material.

## Local Semantic Search Workflow

- A separate LanceDB semantic-search tool venv lives at `.lance`; it is for searching this vault only.
- Use the GPU-backed path for indexing/searching:
  - `source .lance/bin/activate && lance-doctor --device cuda`
  - `source .lance/bin/activate && search --device cuda "vector database embeddings"`
- The `search` command queries the local LanceDB table at `.lancedb`, covering Markdown notes and local text files present at the last index build.
- If notes changed significantly or search results are stale, rebuild the index:
  - `source .lance/bin/activate && lance-reindex --root /home/claude/Documents/Knowledge-Base --device cuda`
- The `.lance` stack mirrors the known-good `/home/claude/sim2real` setup: Python 3.12.3, `torch 2.10.0+cu128`, `sentence-transformers 5.2.3`, `transformers 5.3.0`, `lancedb 0.29.2`, `numpy 2.4.2`, and `pyarrow 23.0.1`.
- The embedding model is `all-MiniLM-L6-v2`, vector dimension 384. The model cache lives in `.lance-cache/sentence-transformers`; the index lives in `.lancedb`. Both are ignored local artifacts.
- In the Codex command sandbox, GPU access may be hidden. Use an approved unsandboxed command or a normal shell for CUDA-backed indexing/search.

## Directory Structure

```
├── Algorithms/
│   ├── AI/
│   │   ├── Optimizers/
│   │   ├── Reinforcement Learning/    # PPO, DQN, SAC, TRPO, A2C, etc.
│   │   └── Tokenization/
│   ├── Clustering/                    # KMeans, KNN, PCA, Voronoi
│   ├── Control Theory/                # Kalman filters, SLAM, IK
│   ├── Dimensionality Reduction/
│   ├── Feature Detection/
│   ├── Graph Algorithms/
│   ├── Linear Algebra/
│   ├── Path Planning/
│   ├── Point Cloud/
│   │   └── Registration/
│   ├── Search/
│   │   └── Best First Search/
│   ├── Spatial Indexing/
│   ├── Statistical/
│   ├── Swarm/
│   └── Vision/
│
├── Hardware/
│   ├── Actuators/
│   │   └── Electric Motors/
│   │       └── Controllers/
│   ├── Cameras/
│   │   ├── Adapters/
│   │   ├── Industrial/
│   │   ├── Sensors/
│   │   ├── Stereo/
│   │   └── Thermal/
│   ├── CPUs/
│   ├── Debugging/
│   ├── Dev Boards/
│   ├── Drones/
│   ├── FPGA/
│   ├── GPUs/                          # Tensor Cores, DGX
│   ├── HDL/
│   ├── Memory/
│   ├── Microcontrollers/
│   │   └── Cellular/
│   ├── Motherboards/
│   ├── Networking/
│   ├── Pentest/
│   ├── Power/
│   ├── Radio Modules/
│   ├── Robots/
│   ├── SBCs/
│   ├── Sensors/
│   │   ├── GPS/
│   │   └── IMU/
│   │       └── Gyroscopes/
│   ├── Test Equipment/
│   └── Vehicle Computing Platforms/
│
├── Protocols/
│   ├── AI/
│   ├── Application/
│   ├── Automotive/
│   │   └── CAN/
│   │       └── Tools/                 # CANoe, etc.
│   ├── Cloud and Web/
│   ├── Data Formats/
│   ├── Distributed Systems/
│   ├── Email and Messaging/
│   ├── Embedded System/
│   ├── File Transfer/
│   ├── IoT/
│   ├── Media Streaming/
│   ├── Network/
│   ├── Real-Time Communication/
│   ├── Reference/
│   ├── Robotics and Industrial/
│   ├── Routing/
│   ├── RPC/
│   ├── Satellite Communication/
│   ├── Security/
│   ├── Serial/
│   ├── Serialization/
│   ├── Storage and Data Access/
│   ├── Time Synchronization/
│   ├── Transport/
│   └── Wireless/
│
├── Software/
│   ├── AI/
│   │   ├── Architectures/
│   │   ├── Datasets/
│   │   ├── Local/
│   │   └── Reinforcement Learning/
│   ├── Compilers/
│   ├── Compute APIs/
│   │   └── CUDA/
│   ├── Databases/
│   ├── Data Formats/
│   ├── Debuggers/
│   ├── Decompilers/
│   ├── Emulators/
│   ├── Geospatial/
│   ├── Graphics/
│   ├── Guidelines/
│   ├── Industrial/
│   ├── Libraries/
│   ├── Operating Systems/
│   │   └── Linux/
│   ├── Programming Languages/
│   │   ├── C/
│   │   ├── Elixir/
│   │   ├── Elm/
│   │   ├── Javascript/
│   │   ├── Python/
│   │   ├── Spin/
│   │   ├── WebAssembly/
│   │   └── Zig/
│   ├── Reinforcement Learning/
│   ├── Robotics/
│   │   └── ROS2/
│   │       └── Build Tools/
│   ├── Security/
│   ├── Simulation/
│   │   ├── CFD/
│   │   ├── Finite Element/
│   │   ├── Mesh Generation/
│   │   └── Visualization/
│   ├── Tools/
│   │   ├── 3D Modeling/
│   │   ├── Build Systems/
│   │   ├── Cloud Storage/
│   │   ├── Containerization/
│   │   ├── Data Labeling/
│   │   ├── Data Recovery/
│   │   ├── DevOps/
│   │   ├── Documents/
│   │   ├── Editors/
│   │   ├── File Transfer/
│   │   ├── Game Development/
│   │   ├── GUI Frameworks/
│   │   ├── Hardware Interface/
│   │   ├── Media/
│   │   ├── Networking/
│   │   │   └── Packet Analyzers/
│   │   ├── Package Management/
│   │   ├── Profilers/
│   │   ├── Server/
│   │   ├── Shell Utilities/
│   │   ├── Testing/
│   │   ├── TUI Frameworks/
│   │   ├── Visualization/
│   │   └── Web Development/
│   └── Vision/
│       └── Calibration/
│
├── Systems/
│   ├── ADAS/
│   └── Cloud/
│
├── Manufacturing/
│   └── CNC/
│
├── Companies/
│
└── zUnsorted/                         # Uncategorized notes
```

## Note Format

Notes use Obsidian-flavored markdown with:
- YAML frontmatter (aliases, tags, title)
- `[[WikiLinks]]` for internal connections to related concepts
- Typical sections: Overview, Core Concepts, Use Cases, Code Snippets (Python/C++/etc), Pros/Cons, Comparison Tables, Related Concepts, External Resources

## Writing Notes

**Purpose:** Notes show how topics inter-relate via Obsidian Graph View. Not in-depth teaching - concise summaries that reveal connections.

### Structure
- `# Main Title` with opening paragraph
- `## Sections` separated by `---` horizontal rules
- Section emojis for visual scanning (gears, checkmarks, books, brain, wrench, etc.)

### Common Sections (choose what's relevant)
- Overview / Summary
- Core Concepts
- Comparison Chart (**almost always include** - find 5+ comparable items)
- Use Cases
- Strengths / Weaknesses (or Pros / Cons)
- How It Works
- Key Features / Capabilities
- Compatible Items
- Variants
- Hardware Requirements (when applicable)
- Related Concepts/Notes (near end, bullet list of `[[WikiLinks]]`)
- External Resources / Further Reading

### Linking Rules
- **Format:** `[[DQN]] (Deep Q Network)` - acronym inside brackets, description outside
- **Density:** Not too sparse, not too dense. Links should make logical sense (Addition → Subtraction, not Addition → SLAM)
- **Acronyms as filenames** for popular terms; spell out full name if acronym conflicts with something more common
- **Valid filenames:** No special characters (use `CI-CD` not `CI/CD`)

### Comparison Charts
Almost always include a comparison table. Examples:
- RANSAC → compare to PROSAC, MLESAC, etc.
- RViz → compare to Gazebo, Ignition, Foxglove, etc.
- Ford → compare to Chevy, Toyota, etc.
Aim for 5+ comparable items when possible.

### Types of Notes
- **Folder-level topics:** `Algorithms.md` - links to many related specific topics
- **Specific topics:** `Bubble Sort.md` - detailed note on one concept

### Quality Guidelines
- **Target length:** ~150-250 lines generally, but not a hard limit. Concise and focused.
- Restrain link count - quality over quantity
- No trailing whitespace after bullet points
- Inline code for commands: `roslaunch pkg node.launch`
- Code blocks are fine when helpful

### Tool Note Thoroughness
When writing about a command-line tool, cover more than the obvious happy path. Check official docs, release notes, `--help` output when available, and common upstream issues/discussions for important flags, lifecycle behavior, and gotchas.

Example: when writing `[[usbipd]]`, covering `usbipd list`, `usbipd bind`, and `usbipd attach --wsl` was not thorough enough. The note also needed `--auto-attach`, because embedded boards often reset or briefly disconnect during flashing. For `usbipd`, expected coverage includes `usbipd attach --wsl --busid <BUSID> --auto-attach`, `--unplugged`, detach/unbind behavior, persistence differences (`bind` persists, attach does not), and the Linux-side relationship to `[[vhci_hcd]]`.


### Reference Examples
- Good: `Software/Programming Languages/Elixir/Elixir Hammer.md`
- Good: `Software/Programming Languages/Elixir/Oban.md`
- Good (could be slightly longer): `Hardware/Cameras/Sensors/Global Shutter.md`
- Too long/over-linked: `Software/AI/RAG.md`

### Creating New Notes
When creating a new note:
1. Propose 2-3 folder location options based on the topic
2. Ask the user which location they prefer before creating the file
3. Create the file directly in the chosen location

Over time, learn the folder structure patterns to make better suggestions.

### Useful Commands

Count WikiLinks per file (find over-linked notes):
```bash
find "/path/to/vault" -name "*.md" -exec grep -c '\[\[' {} \; -print 2>/dev/null | paste - - | sort -rn | head -20
```

Find orphan links (linked but no .md file exists):
```bash
find . -name "*.md" -exec basename {} .md \; | sort -u > /tmp/existing.txt
grep -roh '\[\[[^]]*\]\]' --include="*.md" | sed 's/\[\[//g; s/\]\]//g; s/|.*//' | sort | uniq -c | sort -rn > /tmp/links.txt
while read count link; do grep -qxF "$link" /tmp/existing.txt || echo "$count MISSING: $link"; done < /tmp/links.txt | head -30
```

### Known Orphan Links (as of Jan 2026)

**When you fix any of these, remove them from this list.**

**Quick fixes (alias or rename needed):**

| Links | Broken Link | Existing File | Fix |
|-------|-------------|---------------|-----|
| 37 | `[[Phoenix]]` | `Phoenix Framework.md` | Add alias |
| 30 | `[[CI-CD]]` | `CI-CD Pipelines.md` | Add alias |
| 21 | `[[Point Cloud]]` | `Point Cloud Algorithms.md` | Add alias |
| 19 | `[[Embedded Systems]]` | `Embedded System Protocols.md` | Add alias |
| 18 | `[[WebSockets]]` | `Websockets.md` | Case mismatch |

**Truly missing (high-value to create):**

| Links | Missing Topic | Notes |
|-------|---------------|-------|
| 20 | ROS | ROS1 - only `ROS2.md` exists |
| 19 | Motor Control Algorithms | Hardware/robotics |
| 19 | Encoder | Hardware fundamental |
| 17 | TensorRT | NVIDIA inference engine |
| 14 | JavaScript | Folder exists, no main note |
| 13 | Distributed Systems | Broad CS topic |
| 12 | PostgreSQL | Common database |

### Git Workflow
Keep it simple:
```
git add -A && git commit -m "Short message" && git push
```
- Short commit messages: "Moved ESP-IDF", "Added note on X", "Fixed typo"
- No elaborate multi-line messages needed
- CLAUDE.md is in .gitignore so `git add -A` is safe

### Maintenance
- When reading related notes to create links, if you notice mistakes or issues in existing files (broken links, outdated info, formatting problems), point them out and offer to fix them.
- Folder structure improvements are welcome - if you notice organizational issues, inconsistencies, or better ways to categorize topics, suggest them.
