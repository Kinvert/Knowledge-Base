# OSRM — Open Source Routing Machine 🗺️⚡

OSRM (Open Source Routing Machine) is a high-performance C++ routing engine designed for computing optimal paths across road networks based on OpenStreetMap (OSM) data. It is one of the fastest open-source shortest-path systems available, using contraction hierarchies and specialized preprocessing to perform real-time routing queries at millisecond latency. OSRM is extensively used in map applications, navigation systems, logistics planning, and robotics path planning where speed and scalability are essential.

---

## 🌐 Overview

OSRM transforms raw OSM map data into a graph representation, preprocesses that graph to compress the search space, and then answers routing queries via a REST API. It supports driving, cycling, walking, and arbitrary profiles through Lua-based configuration files. The fundamental design prioritizes extremely fast shortest-path discovery over slow updates (i.e., traffic data must be batch-updated).

OSRM is suitable for:

- Real-time navigation
- Logistics (delivery route optimization)
- Travel-time matrices
- Robotics path planning when global map data is needed
- Large-scale isochrone calculations
- Multi-agent planning depending on road networks

---

## 🧠 Core Concepts

- **OSM Data**: The raw map input. OSRM extracts roads, nodes, and metadata.
- **Profiles**: Lua definitions that tell OSRM what is considered drivable, walkable, penalized, etc.
- **Preprocessing**: OSRM runs steps such as `extract`, `partition`, and `contract`. These operations generate `.osrm` files that contain a compressed road graph.
- **Contraction Hierarchies (CH)**: A speed-up technique that shrinks the graph by collapsing nodes and adding shortcuts. Enables lightning-fast queries.
- **Query Server**: A REST server that takes coordinates and returns optimal routes.
- **Table Service**: Computes travel-time matrices (useful for clustering, CVRP, multi-agent planning).
- **Nearest Service**: Snaps coordinates to the closest drivable node.
- **Annotation**: Returns speed, turn penalties, way metadata, etc.
- **Alternatives**: Suggests multiple viable paths instead of a single route.

---

## 🧾 Comparison Chart — OSRM vs Similar Routing Engines 🧭

| Feature / Engine | OSRM | GraphHopper | Valhalla | HERE Routing (proprietary) | Mapbox Directions (proprietary) |
|---|---:|---:|---:|---:|---:|
| Primary language | C++ (BE) | Java | C++ | Multiple (proprietary) | Multiple (proprietary) |
| Routing algorithm | Contraction Hierarchies (CH) | CH, Landmarks, CH+ | Multi-modal (CH-like + PHAST), customizable | Proprietary (fast, commercial) | Proprietary (fast, optimized) |
| Query latency | Extremely low (ms) | Low (ms–tens ms) | Low (ms–tens ms) | Low | Low |
| Preprocessing cost | High (heavy contraction step) | Moderate–High | Moderate–High | Managed | Managed |
| Memory footprint | High for large regions | Tunable (less in some configs) | Moderate | Managed | Managed |
| Real-time traffic | Limited (batch/overlay) | Better support via GraphHopper Traffic | Has mechanisms for dynamic costing | Strong commercial support | Strong commercial support |
| Map matching | `match` API | Yes (map-matching module) | Yes | Yes | Yes |
| Isochrones / time surfaces | Via plugins / external | Built-in isochrones (paid/OSS modules) | Isochrones supported | Yes | Yes |
| Multi-modal (walk/bike/public transit) | Basic (profiles) | Good (public transit support via modules) | Strong multi-modal support | Strong | Strong |
| Flexibility of profiles | Lua-based, highly customizable | Java-based profiles (flexible) | JSON/profile rules | High | High |
| Scalability / QPS | Very high for point-to-point | High; sharding possible | High; tile-based | Scales (commercial) | Scales (commercial) |
| Matrix / many-to-many | `table` service, fast but memory-limited per request | Matrix support (tunable) | Matrix support | Yes | Yes |
| Ease of ops | Simple deployable binary + large RAM for big regions | JVM ops (heap tuning) | Binary + moderate ops | Managed SaaS or enterprise | Managed SaaS |
| Licensing | BSD-style (open) | Apache 2.0 (open) | MIT (open) | Commercial | Commercial |
| Best fit | Lightning-fast P2P routing, web backends, map matching | Flexible routing, lower memory kink, enterprise options | Multi-modal, transit-aware, routing + tiles | Enterprise, SLAs, global features | Fast integration, managed service |

---

## 🏗️ How to Build a Routing Service / Use OSRM in a Web App or RL Project — Practical Guide 🔧

### 1) High-level architecture (web app)
- **Preprocessing pipeline (one-time / batch)**:
  - Obtain OSM PBF for region (example: `region.osm.pbf`).
  - `osrm-extract` with a profile (Lua) — defines speeds, allowed ways.
  - `osrm-contract` (or `osrm-partition` + `osrm-customize`) to build CH files.
  - Store `.osrm` artifact set in object store or attached storage for deployment.
- **Routing service**:
  - Run `osrm-routed --algorithm mld` or `--algorithm ch` depending on build.
  - Expose REST endpoints: `route`, `table`, `nearest`, `match`, `trip`, `tile`.
  - Place behind an HTTP reverse proxy (Nginx) and autoscale multiple `osrm-routed` instances for QPS.
- **Frontend / API layer**:
  - Lightweight API gateway (Node/Python/Elixir) to:
    - Authenticate/limit QPS.
    - Compose route requests (snap points, optimize via `table` for many stops).
    - Cache responses for repeated queries (CDN for static tiles; Redis for route results).
  - Web UI uses Leaflet/Mapbox GL to render geometry and step instructions.
- **Data & analytics**:
  - Log requests to Kafka or job queue for metrics and replay.
  - Use `table` service to compute travel-time matrices offline; store results in ClickHouse/Parquet.
- **Monitoring & Ops**:
  - Export metrics (Prometheus) from `osrm-routed` and from preprocess jobs.
  - Use healthchecks, readiness probes, and alerting for memory pressure (CH is memory-heavy).
  - Automate rebuilds (daily/weekly) using CI if OSM updates are required.

### 2) Web-app usage patterns & optimizations
- **Caching**:
  - Cache `nearest` and `route` results for hot origin/destination pairs.
  - Cache `table` computations for common clusters (e.g., depot → candidates).
- **Batch vs Interactive**:
  - Use `table` or `trip` for bulk route optimization (precompute TSP cost matrix).
  - Use `route` for interactive, per-user directions.
- **Costing & profiles**:
  - Tweak Lua profile to penalize/unpenalize certain roads (truck restrictions, tolls).
  - Keep separate profiles for `car`, `truck`, `bike`, `foot`.
- **Scaling**:
  - Horizontal scale by running multiple `osrm-routed` instances per region (each requires loaded CH in RAM).
  - Partition large regions into tiles/regions and route using stitching logic when necessary.
  - Consider using GraphHopper/Valhalla for lower-memory footprints or better dynamic updates if OSRM memory is limiting.

### 3) Using OSRM in an RL project (environment + training)
- **Roles OSRM can play**:
  - **Global planner**: OSRM provides realistic road-based waypoints that an RL agent uses as high-level targets; local planner handles collision/kinematics.
  - **Reward shaping**: Use travel time / distance from OSRM to compute dense rewards or to normalize episode lengths.
  - **Environment realism**: Map-matched trajectories (via `match`) from real drivers provide expert demonstrations for imitation learning.
  - **Multi-agent coordination**: Use `table` to produce pairwise travel-time matrices for dispatch or coordination tasks.
- **Data flow for RL**:
  - Simulator / agents request waypoints from OSRM (`route`), follow them with a physics-based local policy, and emit experience tuples to Kafka or files.
  - Use OSRM `nearest` to convert noisy agent GPS to drivable nodes for observations.
  - For batch training, use precomputed `table` matrices as part of state features or adjacency heuristics.
- **Practical experiment design**:
  - **Offline phase**: Create datasets by sampling routes across the environment, compute features (distance, travel_time, road_type), store as Parquet.
  - **Online training**: RL workers query OSRM for path plans; limit QPS by local caching or embedding a lightweight pathfinder (e.g., A*/CH trimmed) for sub-steps.
  - **Domain randomization**: Vary profile speeds or penalties (simulate traffic conditions) to improve robustness.
- **Performance considerations**:
  - Avoid frequent re-preprocessing during training — OSRM expects static weights. For dynamic scenarios (changing speed profiles), keep a small in-memory modifier layer on top of OSRM outputs.
  - Use `table` to amortize cost: compute many-to-many once per training epoch and reuse.
- **Safety / reproducibility**:
  - Log OSRM version, profile Lua file, and `.osrm` metadata with each dataset to ensure reproducible experiments.
  - For deterministic replay, store full routes (coordinate sequences) rather than regenerating at runtime.

### 4) Building your own routing engine (overview)
- **Core components**:
  - **Graph builder**: parse OSM → directed graph with weights (speed, turn costs).
  - **Contraction / speed-up layer**: CH, ALT (A*, landmarks), PHAST, or transit node routing to accelerate queries.
  - **Index & storage**: efficient on-disk storage and memory mapping for quick startup.
  - **Routing API**: endpoints for `route`, `table`, `match`, `nearest`.
  - **Profiles & costing**: flexible policy layer to handle vehicle classes and restrictions.
  - **Map-matching**: HMM-based or simpler snapping to convert noisy GPS tracks to graph paths.
- **Design trade-offs**:
  - **Preprocessing vs Query time**: More preprocessing (CH) => faster queries but heavier builds and memory.
  - **Memory vs accuracy**: Shortcut edges increase memory but reduce search time.
  - **Dynamic updates**: If you need frequent edge-weight updates (traffic), prefer algorithms supporting fast updates (e.g., dynamic shortest paths, or hybrid where dynamic factors are applied post-hoc).
- **Useful components to reuse**:
  - Graph libraries (OSM parser components), CH implementations, HMM map-matching libraries, geometry libs (GEOS), spatial indexes (R-tree).
- **Testing & validation**:
  - Unit test for routing correctness (shortest path checks on small graphs).
  - Integration test against public datasets and baseline engines (compare durations/paths).
  - Regression tests for profile changes (ensure no unexpected road closures).

### 5) Operational checklist for production
- Ensure sufficient RAM per region (estimate via small-scale runs).
- Automate OSM updates: incremental vs full rebuild strategy.
- Use autoscaling for `osrm-routed` instances with warm-up strategies (RAM warm).
- Implement request throttling and per-user quotas.
- Monitor latencies, memory, and error rates; track distribution of route lengths and table sizes.
- Maintain versioned artifacts: keep copies of `.osrm` files for reproducibility.

---

## ✅ Quick Practical Tips (bullet list)
- Use `table` for routing many vehicles to many destinations — it’s faster than N×`route`.
- For route visualizations, send `geometry=polyline` (or `geometry=geojson`) from `route`.
- For noisy traces, run `match` first, then `route` between matched points for stable behavior.
- If OSRM memory is a blocker, try GraphHopper (Java) or Valhalla (C++) as alternatives with different trade-offs.
- Always store the Lua profile and OSRM version with datasets to reproduce numeric results for RL experiments.

---

## 🏗️ Architecture

OSRM’s architecture typically includes:

1. **Extraction step**  
   Reads OSM PBF/XML, interprets roads according to the chosen Lua profile.

2. **Preprocessing step**  
   The graph is partitioned, contracted, and shortcut edges are built.

3. **Routing engine (osrm-routed)**  
   The high-performance server that answers queries via JSON REST endpoints.

4. **Optional frontend/UI**  
   A Leaflet plugin called `osrm-frontend` visualizes routes for debugging.

The model is "precompute once, query infinitely" – ideal when maps change infrequently (daily or weekly updates).

---

## ⚡ Why OSRM Is So Fast

OSRM uses **Contraction Hierarchies (CH)** as its primary algorithm. This drastically reduces search time by:

- Ordering nodes by "importance"
- Removing low-importance nodes
- Adding shortcut edges to maintain correctness
- Shrinking the graph by orders of magnitude

This gives:

- Millisecond-range shortest path queries
- Sub-10ms travel-time matrix calculations
- Scalability to thousands of QPS

For robotics and RL, this performance allows OSRM to serve as a global planner feeding high-level waypoints or heuristics into local motion planners.

---

## 🔩 Routing Features

- Dijkstra, A*, Contraction Hierarchies (optimized)
- Turn-by-turn directions
- Turn penalties, u-turn handling
- Customizable speeds (via Lua profile)
- Traffic speeds (but batch-processed, not real-time)
- Distance, duration, annotations
- Alternative routes
- Snapping points to nearest drivable road

---

## 🧮 Services (REST API)

- **`route`**  
  Finds the optimal path between coordinate pairs.

- **`table`**  
  Computes matrix of travel times or distances.  
  Essential for clustering, TSP/VRP, multi-agent RL.

- **`nearest`**  
  Returns nearest road network node for a coordinate.

- **`match`**  
  Map-matching noisy GPS tracks.

- **`tile`**  
  Returns MVT vector tiles for display and visualization.

- **`trip`**  
  Solves a round-trip TSP-like problem (for deliveries).

Each service is stateless and designed for extremely high throughput.

---

## 🚗 Typical Use Cases

### 1) Navigation & Mapping
- Turn-by-turn driving instructions
- Map apps (Leaflet, QGIS plugins)
- Offline navigation for fleets

### 2) Logistics Optimization
- Delivery route planning
- CVRP/TSP/VRP feeding
- Real-time dispatch systems
- Travel-time matrices for hundreds of vehicles

### 3) Robotics & Reinforcement Learning
- Global path planning over large road networks
- Waypoint generation
- Reward shaping using distances and travel times
- Multi-agent navigation with realistic road constraints
- Simulation environments requiring human-like routes

### 4) Data Science & Urban Planning
- Isochrones (areas reachable within X minutes)
- Commuting analysis
- Traffic time estimation (batch)

---

## 🧩 Pros & Cons Summary

**Pros**
- Incredibly fast routing due to CH
- Highly flexible traffic models via Lua profiles
- Supports car, bike, foot, custom modes
- Open-source and battle-tested
- Perfect for real-time queries
- Ideal for TSP/VRP/multi-agent planning via `table`

**Cons**
- Slow preprocessing (heavy batch step)
- Real-time traffic updates difficult
- Requires full rebuild for map updates
- Not suited for indoor or off-road robotics
- Memory-heavy for large countries
- Cannot do weighted edge updates on the fly

---

## 🥇 When to Use OSRM

Choose OSRM when you need:

- Lightning-fast **point-to-point** routing
- **Massive-scale** routing queries per second
- Global/urban networks from OSM data
- Preplanned cost metrics (e.g., speed, congestion categories)
- High-quality road-based paths for agents or simulations

Avoid OSRM when:

- You need dynamic, second-by-second traffic changes
- You are doing off-road or indoor local planning
- You need mutable edge weights during runtime (try GraphHopper or Valhalla)

---

## 🧩 Integrations

- **Python** (`osrm`, `requests`) for routing inside ML pipelines  
- **C++** direct embedding (OSRM is a C++ library)  
- **Rust & Go** community bindings  
- **Leaflet / Mapbox** for visualization  
- **Docker** predefined images make deployment trivial  
- **Reinforcement Learning** environments needing large-scale graph-based navigation

---

## 🔗 Related Concepts

- [[OpenStreetMap]]  
- [[GraphHopper]]  
- [[Valhalla Routing Engine]]  
- [[Path Planning]]  
- [[Dijkstra]]  
- [[A*]]  
- [[Contraction Hierarchies]]  
- [[Routing Graphs]]  
- [[VRP]] and [[TSP]]  
- [[Multi-Agent RL]]  

---

## 🏁 Summary

OSRM is a high-speed routing engine built on OSM data, using contraction hierarchies for extremely fast queries. It is perfect for navigation systems, logistics optimization, and RL environments requiring global path planning on realistic road networks. While preprocessing is heavy and dynamic updates are limited, its runtime speed is unmatched among open-source routers.
