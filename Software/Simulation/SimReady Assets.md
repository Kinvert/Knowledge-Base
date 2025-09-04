# SimReady Assets (OpenUSD) for Robotics and Digital Twins
SimReady assets are standardized 3D models authored in OpenUSD that include physics, semantics, and metadata so they behave realistically in simulation—critical for robotics workflows (RL/IL), synthetic data, and digital twins. This note catalogs where to find SimReady assets, what “SimReady” actually guarantees, and how they compare to other USD assets. It also links tutorials and resources, with a special focus on sources you can download today.

---

## 🧭 Overview
SimReady (from NVIDIA Omniverse) defines how 3D assets should be packaged to be physically accurate, simulation-friendly, and semantically rich. Compared with generic USD or CAD, SimReady bundles:
- USDPhysics colliders, materials, and mass/inertia
- Joints/constraints where applicable (e.g., hinges, prismatic)
- Semantics/labels for perception tasks and synthetic data
- Consistent scale/axis/unit conventions and LOD variants

Key starting points:
- NVIDIA SimReady overview and rationale: https://developer.nvidia.com/omniverse/simready-assets
- SimReady documentation hub: https://docs.omniverse.nvidia.com/simready/latest/index.html
- Downloadable SimReady packs (warehouses, furniture, containers, materials): https://docs.omniverse.nvidia.com/usd/latest/usd_content_samples/downloadable_packs.html

---

## 🧩 Core Concepts
- **OpenUSD-first**: Assets are authored as `.usd/.usda/.usdc` with composition arcs (payloads, references) enabling modular assembly.
- **Physics-ready**: USDPhysics schemas define colliders, materials (static, dynamic), and joints; you typically add or toggle `RigidBody` on props you want to move.
- **Semantics**: Labels and categories (e.g., “pallet”, “IBC tank”) support synthetic data and downstream ML datasets.
- **Materials**: MDL and MaterialX/OpenPBR materials yield photoreal rendering and sim-consistent appearance; see SimReady materials set: https://github.com/NVIDIA-Omniverse/PhysicalAI-SimReady-Materials
- **Searchability at scale**: USD Search API indexes SimReady libraries to find assets by text/image and scene context: https://docs.omniverse.nvidia.com/services/latest/services/usd-search/overview.html

---

## 🔎 Comparison Chart (SimReady & Nearby Options)
| Source | What you get | Physics/Joints | Semantics/Labels | Licensing/Access | Pretrained Policies? | Tutorials/Docs |
|---|---|---|---|---|---|---|
| **NVIDIA SimReady (Developer page)** | Overview + links to packs; explains standard | Yes (USDPhysics) | Yes | Free downloads + docs | No (assets only) | https://developer.nvidia.com/omniverse/simready-assets |
| **Downloadable Packs (Omniverse)** | Warehouse, Furniture/Misc, Containers/Shipping, Materials | Colliders included; add `RigidBody` as needed | Yes (categories) | Free | No | https://docs.omniverse.nvidia.com/usd/latest/usd_content_samples/downloadable_packs.html |
| **Hugging Face: PhysicalAI SimReady Warehouse 01** | Prebuilt warehouse scene + assets | Colliders included; user adds `RigidBody` where needed | Yes | Free (dataset) | No | https://huggingface.co/datasets/nvidia/PhysicalAI-SimReady-Warehouse-01 |
| **SimReady Explorer (Omniverse Extension)** | In-app browser with 1,000+ SimReady assets ready to drop into scenes | Yes | Yes | Bundled with Omniverse | No | https://docs.omniverse.nvidia.com/extensions/latest/ext_core/ext_browser-extensions/simready-explorer.html |
| **Lightwheel Simready Assets** | Custom, research-oriented sim assets for manipulation/locomotion; some open-sourced | Yes | Typically | Site access (some free) | No | https://lightwheel.ai/home |
| **Isaac Sim Robot Assets (built-in)** | Ready-to-use robot USDs (e.g., Jackal, Dingo, Franka) | Full rigs/joints | N/A or basic | Included with Isaac Sim | **Yes via Isaac Lab examples** | https://docs.isaacsim.omniverse.nvidia.com/5.0.0/assets/usd_assets_robots.html |
| **Reallusion Rigged Characters Pack** | People/worker characters in USD (good for scenes) | Skeleton rigs (animation) | Limited | Free pack | No | See “Rigged Characters Asset Pack” on the Downloadable Packs page |
| **USD-compatible marketplaces (CGTrader/TurboSquid)** | Huge libraries; some USD | Often “mesh-only”; add physics yourself | Rare | Mixed licenses | No | General guidance: https://www.edgeimpulse.com/blog/how-to-use-usd-assets-for-any-edge-ai-use-case-with-nvidia-omniverse/ |

Notes:
- Downloadable Packs detail page lists contents and sizes; e.g., “SimReady Containers & Shipping 01 (316 models)” and more: https://docs.omniverse.nvidia.com/usd/latest/usd_content_samples/downloadable_packs.html
- SimReady docs (what’s included, e.g., semantics, physics conventions): https://docs.omniverse.nvidia.com/simready/latest/index.html

---

## 🧱 Asset Types (What exists today)
- **Passive props** (pallets, boxes, shelving, cones, furniture) ready for perception/synthetic data; many ship with colliders but require you to enable `RigidBody` for dynamics
  - Packs & dataset: Warehouse 01/02, Containers & Shipping, Furniture & Misc (see Downloadable Packs)
- **Articulated / jointed** (doors, drawers, robot joints)
  - USDPhysics joints (revolute/prismatic/fixed) defined where appropriate; robot USDs in Isaac Sim have full articulation
- **Rigged characters** (workers, delivery people) useful for realism and domain randomization
  - Rigged Characters Pack; compatible with USD animation workflows
- **Robots** (manipulators, mobiles, quadrupeds) in Isaac Sim
  - Built-in models with sensors and controllers starters: https://docs.isaacsim.omniverse.nvidia.com/5.0.0/assets/usd_assets_robots.html
- **Materials libraries** (OpenPBR/MaterialX, MDL) to standardize looks: https://github.com/NVIDIA-Omniverse/PhysicalAI-SimReady-Materials

---

## 🤖 “Pre-trained” Assets vs Policies
Assets themselves are not “pre-trained,” but **pretrained control policies** exist and can be deployed on SimReady scenes/robots:
- **Isaac Lab** provides tasks and example checkpoints for locomotion and manipulation (e.g., Unitree H1/Spot flat-terrain velocity policies; HOVER WBC tutorial)  
  - Isaac Lab site: https://isaac-sim.github.io/IsaacLab/  
  - Deploy policies to Isaac Sim: https://docs.isaacsim.omniverse.nvidia.com/latest/isaac_lab_tutorials/tutorial_policy_deployment.html  
  - HOVER policy tutorial (humanoid control): https://isaac-sim.github.io/IsaacLab/main/source/policy_deployment/00_hover/hover_policy.html
- **Environments for assembly/manipulation** (AutoMate tasks) that can evaluate trained checkpoints: https://isaac-sim.github.io/IsaacLab/main/source/overview/environments.html

Workflow: drop SimReady assets (warehouse props, fixtures) into Isaac Sim or USD Composer, then deploy Isaac Lab-trained policies to interact with them.

---

## 🛠️ Tools & Workflows (build, browse, search)
- **Authoring SimReady**: “Building Simulation-Ready USD 3D Assets in Omniverse” (what makes an asset SimReady + best practices)  
  https://developer.nvidia.com/blog/building-simulation-ready-usd-3d-assets-in-nvidia-omniverse/
- **SimReady Explorer** (browse/import 1,000+ assets in Omniverse)  
  https://docs.omniverse.nvidia.com/extensions/latest/ext_core/ext_browser-extensions/simready-explorer.html
- **USD Search API** (index and query SimReady libraries; NVIDIA API Catalog demo is pre-populated with a warehouse SimReady database)  
  Overview: https://docs.omniverse.nvidia.com/services/latest/services/usd-search/overview.html  
  Get started: https://docs.omniverse.nvidia.com/services/latest/services/usd-search/get-started.html  
  Samples: https://github.com/NVIDIA-Omniverse/usdsearch-samples
- **Physics reference** (USDPhysics schemas, joints, rigid bodies)  
  https://openusd.org/dev/api/usd_physics_page_front.html
- **Importing custom robots** (URDF→USD)  
  Video: https://www.youtube.com/watch?v=AMfEtZ4hyLY

---

## 🧰 Where to Download (Direct Links)
- **SimReady Packs (official)**  
  - Warehouse 01 / Warehouse 02 / Furniture & Misc / Containers & Shipping 01 / Containers & Shipping 02 / Data Center / vMaterials & Automotive MDL packs  
  - Catalog page: https://docs.omniverse.nvidia.com/usd/latest/usd_content_samples/downloadable_packs.html
- **Hugging Face (PhysicalAI SimReady Warehouse 01)**  
  - https://huggingface.co/datasets/nvidia/PhysicalAI-SimReady-Warehouse-01
- **Materials (OpenPBR/MaterialX)**  
  - https://github.com/NVIDIA-Omniverse/PhysicalAI-SimReady-Materials
- **Lightwheel** (custom sim-ready assets; some open)  
  - https://lightwheel.ai/home

---

## 🧪 Tutorials & Example Walkthroughs
- **Intro/overview of SimReady** (livestream)  
  https://www.youtube.com/watch?v=pB3DuFuHkr0
- **Creating SimReady assets** (step-by-step series)  
  https://www.youtube.com/watch?v=DEOqj_lndoQ
- **Composing scenes with SimReady**  
  https://www.youtube.com/watch?v=mJzq_OD8v0s
- **Simple scene interactions with pallets/boxes (Dell InfoHub)**  
  https://infohub.delltechnologies.com/en-us/l/digital-twin-journey-computer-vision-ai-model-enhancement-with-dell-technologies-solutions-nvidia-omniverse/3d-simulations/2/
- **Deploying Isaac Lab policies to Isaac Sim**  
  Docs: https://docs.isaacsim.omniverse.nvidia.com/latest/isaac_lab_tutorials/tutorial_policy_deployment.html  
  Video: https://www.youtube.com/watch?v=Cx9ss0v-DMs

---

## 🏷️ Revel (Robotics Data Infrastructure)
- **Revel Studios**: https://revelstudios.io/  
  Public, downloadable “Revel-branded SimReady” asset packs were not found at time of writing. Given Revel’s positioning around robotics data infrastructure and the SimReady/Physical AI ecosystem, reach out via their site for enterprise catalogs, custom USD/SimReady libraries, or integration with Omniverse/Isaac data pipelines.

---

## 🧠 Best Practices & Gotchas
- **Rigid body vs collider**: Many packs ship with colliders enabled but leave `RigidBody` off—so they won’t fall or move until you enable it (scene-dependent).
- **Units & scale**: Keep meters as the base unit; validate axis conventions (Z-up in Omniverse) when importing from CAD/URDF.
- **Articulation graphs**: For drawers/doors/robots, ensure joint limits, drive parameters, and damping are set for stable simulation.
- **Semantics**: Use consistent labels (e.g., “pallet_wood”, “box_cardboard_small”) for dataset generation and USD Search indexing.
- **Materials**: Prefer MaterialX/OpenPBR or MDL libraries consistent with your renderer/sensor simulation for better domain gap.

---

## 🧩 Compatible Tools / Ecosystem
- **Simulators**: [[Isaac Sim]] (OpenUSD-native), USD Composer; interoperability with Unreal/Unity via USD importers
- **Learning**: [[Isaac Lab]] (RL/IL), [[Reinforcement Learning]] frameworks (RSL-RL, PPO, SAC, etc.)
- **Data**: [[Synthetic Data]] generation, domain randomization in Omniverse
- **Standards**: [[OpenUSD]], [[USDPhysics]], [[MaterialX]], MDL

---

## ✅ Strengths / ⚠️ Limitations
**Strengths**
- Standardized physics + semantics; drop-in consistency across scenes
- Scales with USD composition and USD Search
- Free official packs cover common industrial objects

**Limitations**
- Some assets need final tuning (mass/inertia, friction) for your task
- Not all third-party USD assets meet SimReady conventions
- Public catalogs of “articulated industrial equipment” are growing but still limited; robots are mostly accessed via Isaac Sim libraries/vendors

---

## 🔗 External Resources
- SimReady home: https://developer.nvidia.com/omniverse/simready-assets
- SimReady docs hub: https://docs.omniverse.nvidia.com/simready/latest/index.html
- Downloadable packs: https://docs.omniverse.nvidia.com/usd/latest/usd_content_samples/downloadable_packs.html
- PhysicalAI SimReady Warehouse dataset: https://huggingface.co/datasets/nvidia/PhysicalAI-SimReady-Warehouse-01
- USD Search (overview + getting started): https://docs.omniverse.nvidia.com/services/latest/services/usd-search/overview.html • https://docs.omniverse.nvidia.com/services/latest/services/usd-search/get-started.html
- Lightwheel Simready assets: https://lightwheel.ai/home
- Materials (OpenPBR/MaterialX): https://github.com/NVIDIA-Omniverse/PhysicalAI-SimReady-Materials
- Dell InfoHub example: https://infohub.delltechnologies.com/en-us/l/digital-twin-journey-computer-vision-ai-model-enhancement-with-dell-technologies-solutions-nvidia-omniverse/3d-simulations/2/
- Isaac Sim robot assets: https://docs.isaacsim.omniverse.nvidia.com/5.0.0/assets/usd_assets_robots.html
- Isaac Lab (pretrained examples + policy deploy): https://isaac-sim.github.io/IsaacLab/ • https://docs.isaacsim.omniverse.nvidia.com/latest/isaac_lab_tutorials/tutorial_policy_deployment.html

---

## 🔗 Related Notes
- [[OpenUSD]] (Scene description and composition)
- [[USDPhysics]] (Physics schema: colliders, joints, rigid bodies)
- [[Isaac Sim]] (Simulator; asset browser; ROS/ROS 2)
- [[Isaac Lab]] (RL/IL framework for Isaac Sim)
- [[Synthetic Data]] (Dataset generation in Omniverse)
- [[MaterialX]] (OpenPBR materials)
- [[ROS2]] (Robot Operating System)
- [[Digital Twin]] (Industrial simulation)
