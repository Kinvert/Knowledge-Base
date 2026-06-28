# 🤖 Configuration Space

**Configuration space** (often written as **C-space**) is the set of all possible robot configurations (poses or states) represented as a state vector. In robotics, it is the core abstraction for motion planning because planning is done on robot-wide validity of movement, not just Cartesian coordinates of an end effector or position in the world.

---

## 🧠 Overview

- A configuration is a complete assignment of the robot’s DOF variables (for example, joint angles and base pose).
- A configuration space point represents one exact robot geometry layout in the world frame model.
- A motion plan becomes a continuous path in C-space, not a 2D/3D line in Euclidean map space.
- Obstacles become **configuration-space obstacles**: sets of configurations where the robot would collide.

### Why this matters

Robots with many DOF cannot be planned by only drawing a path in xy-plane; they must plan in the space where geometry constraints are explicit.

---

## 🧩 Definition terms

- **Configuration (q)**: The minimal coordinate set describing the robot pose (e.g., `(x, y, theta, q1, q2, ... qn)`).
- **Dimension**: Equals number of DOF in the chosen coordinates.
- **Free space (`C_free`)**: Set of collision-free configurations.
- **Obstacle region (`C_obs`)**: Set of configurations that cause collision.
- **Boundary of obstacles**: The configuration-space boundary where configurations touch constraints.
- **Roadmap/sample point**: A point in C-space used by planners such as PRM and RRT families.

---

## 🧭 Workspace vs C-space

Workspace coordinates can describe where the robot is; C-space coordinates describe how the robot is configured.

- Workspace planning: easier to visualize, but may hide joint limits and self-collision constraints.
- C-space planning: higher-dimensional and expensive, but precise for collision and feasibility.

---

## ⚙️ How planners use C-space

### 1) Discretized occupancy approaches
- Grid or lattice planners sample discretized subsets of C-space.
- Commonly used for low-to-moderate dimensions.

### 2) Sampling-based planning
- Planners like `[[RRT]]` and `[[Informed RRT*]]` query `C_free` with collision checks.
- Useful for higher-dimensional configuration spaces where grids explode.

### 3) Roadmaps and graphs
- `[[CHOMP]]`, `[[RRT-Connect]]`, and PRM-like systems use C-space samples as graph vertices.

### 4) Dynamics and kinodynamics
- Kinodynamic constraints embed velocity/acceleration into augmented C-space variants.

---

## 🧠 What C-space is useful for

- Detecting collisions that require full-body reasoning.
- Planning with articulation (arms, humanoid links, multi-DOF gimbals).
- Handling joint limits, joint coupling, and end-effector constraints consistently.
- Unifying obstacle avoidance and feasibility checks before command-level control.

---

## ⚠️ What C-space does not solve by itself

- It does not model controller stability or optimal feedback control by itself.
- It can become computationally intractable in very high dimensions.
- Real-time guarantees still depend on planner settings, heuristics, and collision-check speed.
- Perception latency and model drift remain separate issues in deployed robotics stacks.

---

## 📊 Comparison chart

| Concept | What it represents | Typical dimension | Used by |
|---|---|---:|---|
| `Configuration Space` | Full robot state (generalized coordinates) | #DOF of model | Motion planners (`[[RRT]]`, `[[Path Planning]]`, [[Motion Planning]]) |
| `Joint Space` | Joint variables only (joint angles/positions) | Arm/link DOF | Manipulator trajectory planning |
| `Task Space` | End-effector geometric pose/task coordinates | Usually 3D + orientation | IK/operational control and grasping |
| `State Space` | Full system state in dynamics/control context | Potentially includes velocity, force, battery state, etc. | Control and reinforcement learning systems |
| `Workspace` | Position/orientation of parts in world coordinates | Mostly 3D(+orientation) | Perception, mapping, navigation |
| `Configuration Space Obstacles (C_obs)` | Invalid configuration subsets | Same as C-space dimension | Collision-aware planners |

---

## 🧮 Practical implications

- **Sample density**: More DOF means exponentially more volume to cover.
- **Collision model quality**: `C_free` correctness depends directly on robot geometry and obstacle models.
- **Distance metric choice**: Euclidean in joint coordinates may fail to capture motion cost in joints with wraparound or constraints.
- **Steer function quality**: In local planning, interpolation in C-space strongly affects success in narrow passages.

---

## 🧱 Common variants and approximations

- **Embedded C-space**: Subset of C-space around current operating mode (e.g., fixed gait/pose mode).
- **Reduced-order C-space**: Uses only task-relevant DOF to make planning tractable.
- **Grid/Lattice C-space**: Discrete approximations for exactness at lower scale.
- **Continuous C-space samples**: RRT family and PRM methods.

---

## ✅ Pros and ❌ Cons

### ✅ Pros
- Faithful representation of collision feasibility.
- Naturally handles kinematic constraints and joint limits.
- Enables principled sampling and roadmap planning in robotic manipulators.

### ❌ Cons
- Can be high-dimensional and expensive.
- Collisions and connectivity checks are heavy in complex geometry.
- Harder for operators and logs to interpret than world-space maps.

---

## 🔧 Related notes

- [[Path Planning]]
- [[RRT]]
- [[RRT-Connect]]
- [[Informed RRT*]]
- [[CHOMP]]
- [[OMPL]]
- [[Motion Planning]]
- [[Robot Dynamics and Spatial Algebra]]
- [[Operational Space Control]]

---

## 🌐 External references

- [Configuration space (Wikipedia)](https://en.wikipedia.org/wiki/Configuration_space)
- [Motion planning overview (Wikipedia)](https://en.wikipedia.org/wiki/Motion_planning)
- [LaValle, Planning Algorithms — Configuration Space chapter](https://lavalle.pl/wordpress/wp-content/uploads/2014/12/Planning-Algorithms-2015-07.pdf)
- [ROS 2 moveit collision and planning concepts (for practical stack context)](https://moveit.ros.org/)

