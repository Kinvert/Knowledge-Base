# Genesis

**Genesis** is an open-source physics simulation platform for robotics and embodied AI, released December 2024 by a consortium of 20 institutions including MIT, Stanford, CMU, ETH Zurich, and UC Berkeley. It claims speeds of 43+ million FPS on a single RTX 4090—10-80x faster than [[Isaac Gym]], [[MuJoCo]], and other GPU-accelerated simulators. Genesis unifies multiple physics solvers (rigid body, MPM, SPH, FEM, PBD, fluids) in a single framework with native Python APIs and cross-platform support.

---

## Overview

Genesis aims to be the "foundation model" of physics simulation—a universal engine that handles everything from rigid robots to fluids, soft bodies, and granular materials. Unlike specialized simulators, Genesis integrates diverse physics solvers that can interact and couple, enabling simulation of complex multi-material scenarios.

The project is ambitious: beyond physics, Genesis includes a generative AI engine for creating environments, motions, and training data from natural language prompts (though generative features are still being rolled out).

- **GitHub**: https://github.com/Genesis-Embodied-AI/Genesis
- **Documentation**: https://genesis-world.readthedocs.io/
- **Website**: https://genesis-embodied-ai.github.io/
- **PyPI**: `pip install genesis-world`

---

## Why Is Genesis So Fast?

Genesis achieves extreme speeds through several architectural decisions:

### 1. Native GPU-Parallel Architecture

Genesis was built from scratch for GPU parallelism, not retrofitted. The entire physics pipeline runs on GPU with minimal CPU-GPU data transfer:

```python
# Creating 4096 parallel environments is one parameter
scene = gs.Scene(n_envs=4096, env_spacing=(2.0, 2.0))
```

All environment states are stored as contiguous GPU tensors, enabling SIMD-style parallel updates across thousands of environments simultaneously.

### 2. Simplified Contact Solver

Genesis uses a relatively simple constraint solver compared to engines like MuJoCo or PhysX. This trades some accuracy for speed:
- Fewer solver iterations per timestep
- Simpler contact models
- Faster convergence at cost of stability in edge cases

This explains both its speed advantage AND some reported manipulation accuracy issues.

### 3. Auto-Hibernation

Genesis detects when objects are stationary and "hibernates" them, skipping physics updates entirely. In benchmarks where objects settle quickly, this dramatically inflates FPS numbers. Real RL training with continuous actions sees lower (but still fast) performance.

### 4. Optimized Collision Detection

Genesis implements:
- Broad-phase spatial hashing
- Narrow-phase optimized for common robot geometries
- Optional self-collision skipping (disabled by default)

### 5. 100% Python with Taichi Backend

Despite being pure Python, Genesis achieves native speeds via Taichi—a domain-specific language that JIT-compiles Python to optimized GPU kernels:

```python
# This Python code compiles to CUDA/Metal/Vulkan
@ti.kernel
def physics_step():
    for i in range(n_bodies):
        # Parallel execution across all bodies
        update_velocity(i)
        update_position(i)
```

---

## Benchmark Deep Dive

### Headline Numbers (Genesis Team, Jan 2025)

| Configuration | FPS (RTX 4090) | vs Real-Time |
|--------------|----------------|--------------|
| Franka arm, self-collision ON | 43M+ | 430,000x |
| Franka arm + random actions | 27M | 270,000x |
| Quadruped locomotion (2 substeps) | ~5M | 50,000x |
| Manipulation (4 substeps) | ~2M | 20,000x |

### The Benchmark Controversy (Stone Tao, Jan 2025)

Independent researcher Stone Tao found significant issues with Genesis's original benchmarks:

| Issue | Impact on Benchmark |
|-------|---------------------|
| **Physics substeps = 1** | Real training uses 2-4 substeps |
| **90% idle time** | Robot takes one action then waits |
| **Self-collision disabled** | Most simulators enable by default |
| **Object hibernation** | Freezes settled objects |

**Corrected benchmark**: When Tao applied realistic settings, Franka arm performance dropped from 43M to **0.29M FPS**—a 150x reduction.

### Comparative Benchmarks (Manipulation)

| Simulator | Cube Picking FPS | Notes |
|-----------|------------------|-------|
| **ManiSkill** | ~100K | GPU-accelerated |
| **Genesis** | ~30K | After corrections |
| **Isaac Gym** | ~50-80K | Depends on config |
| **MuJoCo MJX** | ~40-60K | JAX-accelerated |

Genesis is competitive but not dramatically faster for realistic manipulation tasks.

### Locomotion Benchmarks

| Simulator | Quadruped FPS | Humanoid FPS |
|-----------|---------------|--------------|
| **Genesis** | ~5M | ~2M |
| **Isaac Gym** | ~500K | ~200K |
| **MuJoCo MJX** | ~1M | ~400K |
| **Brax** | ~2M | ~800K |

Genesis shows stronger advantages in locomotion (less contact-heavy) scenarios.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Genesis Platform                          │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐  │
│  │ Generative  │  │   Render    │  │   Physics Engine    │  │
│  │   Engine    │  │   Engine    │  │                     │  │
│  │ (LLM-based) │  │ (Ray-trace) │  │  ┌───┐ ┌───┐ ┌───┐ │  │
│  └─────────────┘  └─────────────┘  │  │RBD│ │MPM│ │SPH│ │  │
│                                     │  └───┘ └───┘ └───┘ │  │
│                                     │  ┌───┐ ┌───┐ ┌───┐ │  │
│                                     │  │FEM│ │PBD│ │SFL│ │  │
│                                     │  └───┘ └───┘ └───┘ │  │
│                                     └─────────────────────┘  │
├─────────────────────────────────────────────────────────────┤
│                    Taichi Backend                            │
│         (JIT compiles to CUDA/Metal/Vulkan/CPU)             │
└─────────────────────────────────────────────────────────────┘
```

### Physics Solvers

| Solver | Materials | Use Case |
|--------|-----------|----------|
| **Rigid Body (RBD)** | Robots, objects | Manipulation, locomotion |
| **MPM** | Soft bodies, granular | Deformables, sand, snow |
| **SPH** | Liquids | Fluid interaction |
| **FEM** | Elastic solids | Soft robotics |
| **PBD** | Cloth, ropes | Thin shells, cables |
| **Stable Fluid** | Gases, smoke | Aerodynamics |

Solvers can be coupled—e.g., rigid robot interacting with MPM soft object in SPH fluid.

---

## Comparison: Physics Simulators for Robotics RL

### Speed Comparison

| Simulator | Peak FPS | Typical RL FPS | GPU Required | Cross-Platform |
|-----------|----------|----------------|--------------|----------------|
| **Genesis** | 43M+ | 2-5M | Yes | Yes (NVIDIA/AMD/Apple) |
| [[Isaac Gym]] | ~1M | 200-500K | Yes (NVIDIA only) | No |
| [[Isaac Lab]] | ~1M | 200-500K | Yes (NVIDIA only) | No |
| [[MuJoCo]] MJX | ~2M | 500K-1M | Yes | Yes (JAX) |
| [[MuJoCo]] CPU | ~50K | 10-30K | No | Yes |
| [[PyBullet]] | ~10K | 5-10K | No | Yes |
| Brax | ~5M | 1-2M | Yes (JAX) | Yes |

### Feature Comparison

| Feature | Genesis | Isaac Gym | MuJoCo | PyBullet |
|---------|---------|-----------|--------|----------|
| **Soft Bodies** | MPM, FEM, PBD | Limited | Limited | No |
| **Fluids** | SPH, Stable Fluid | No | No | No |
| **Differentiable** | Partial (MPM only) | No | Yes (MJX) | No |
| **Ray Tracing** | Native | Via Sim | No | No |
| **MJCF Import** | Yes | Yes | Native | Partial |
| **URDF Import** | Yes | Yes | Via mujoco | Yes |
| **Multi-material** | Yes (coupled) | No | No | No |
| **Apple Silicon** | Yes | No | Yes | Yes |
| **AMD GPU** | Yes | No | Via ROCm | No |

### Accuracy vs Speed Trade-off

| Simulator | Contact Accuracy | Solver Stability | Speed |
|-----------|------------------|------------------|-------|
| **MuJoCo** | Excellent | Excellent | Medium |
| **Isaac Gym** (PhysX) | Good | Good | Fast |
| **Genesis** | Moderate | Moderate | Very Fast |
| **Brax** | Good | Good | Very Fast |
| **PyBullet** | Good | Good | Slow |

Genesis sacrifices some accuracy for speed—fine for locomotion, potentially problematic for dexterous manipulation.

---

## Limitations and Known Issues

### Simulation Accuracy

- **Contact solver simplicity**: Reports of objects falling through grippers
- **Constraint tuning required**: Default configs may not work for all tasks
- **Less battle-tested**: Younger than MuJoCo/Isaac, fewer edge cases addressed

### Differentiability

- **Partial support only**: Currently MPM and Tool Solver only
- **Rigid body gradients coming**: On roadmap but not yet released
- **Can't do gradient-based optimization** for most robotics tasks yet

### Generative Features

- **Not fully released**: Despite marketing, LLM-based generation still rolling out
- **Physics engine is open**: Generative components partially proprietary

### Installation Challenges

- **Taichi version sensitivity**: Requires specific Taichi 1.7.x
- **GPU driver issues**: Some systems have OpenGL/rendering problems
- **AWS/cloud headaches**: Headless rendering can be tricky

### Maturity

- **Rapidly evolving API**: Breaking changes between versions
- **Documentation gaps**: Some features underdocumented
- **Smaller community**: Fewer examples and Stack Overflow answers than MuJoCo

---

## When to Use Genesis

### Good Fit

| Use Case | Why Genesis |
|----------|-------------|
| **Locomotion training** | Speed advantage maximized |
| **Large-scale parallel training** | Excellent scaling |
| **Multi-material simulation** | Unique solver coupling |
| **Cross-platform development** | Apple/AMD support |
| **Rapid prototyping** | Pythonic API, fast iteration |

### Poor Fit

| Use Case | Why Not Genesis |
|----------|-----------------|
| **Dexterous manipulation** | Contact accuracy issues |
| **Gradient-based optimization** | Limited differentiability |
| **Production deployment** | Less mature than alternatives |
| **Existing Isaac/MuJoCo pipeline** | Migration cost |

---

## Integration with RL Libraries

### Native Genesis RL

Genesis includes built-in RL training examples:

```python
import genesis as gs

# Create parallel environments
scene = gs.Scene(n_envs=4096)
robot = scene.add_robot(gs.robots.Franka())

# Standard RL loop
for _ in range(1000):
    actions = policy(observations)
    observations, rewards, dones = scene.step(actions)
```

### With External Libraries

| Library | Integration | Notes |
|---------|-------------|-------|
| [[RSL-RL]] | Via genesis_lr project | Legged locomotion focus |
| [[Stable-Baselines3]] | Gym wrapper needed | CPU overhead possible |
| [[CleanRL]] | Direct tensor interface | Minimal wrapping |
| [[PufferLib]] | Not direct integration | PufferLib focuses on CPU envs |
| [[RL-GAMES]] | Community examples | Good for locomotion |

### genesis_lr Project

For [[Legged Gym]]-style locomotion training:

```bash
# Supports both Genesis and Isaac Gym with same codebase
git clone https://github.com/lupinjia/genesis_lr
```

---

## Relationship to PufferLib

[[PufferLib]] and Genesis serve complementary but different roles:

| Aspect | Genesis | PufferLib |
|--------|---------|-----------|
| **Focus** | GPU physics simulation | RL training infrastructure |
| **Environment type** | GPU-native | CPU (primarily) |
| **Speed source** | Parallel physics | Vectorized rollouts |
| **Typical use** | Robotics simulation | Game-like environments |

PufferLib explicitly notes it "omits libraries focused on control and robotics environments, most of which are simulated on the GPU." Genesis is one such GPU-native simulator.

**Potential integration**: A PufferLib wrapper for Genesis could provide:
- Standardized Gym interface
- Multi-backend policy support
- Experiment tracking integration

Currently no official integration exists—Genesis uses direct tensor interfaces.

---

## Getting Started

### Installation

```bash
# Requires Python 3.8-3.11, PyTorch
pip install genesis-world

# Or nightly for latest features
pip install genesis-world-nightly
```

### Minimal Example

```python
import genesis as gs

# Initialize
gs.init(backend=gs.cuda)  # or gs.cpu, gs.metal

# Create scene with 1000 parallel environments
scene = gs.Scene(n_envs=1000, env_spacing=(2.0, 2.0))

# Add ground and robot
scene.add_ground()
robot = scene.add_robot(
    gs.robots.Franka(),
    pos=(0, 0, 0)
)

# Build scene
scene.build()

# Simulation loop
for _ in range(1000):
    actions = torch.randn(1000, 7, device=gs.device)
    scene.step(actions)
```

### Benchmarking

```bash
# Run official benchmark
python -m genesis.benchmark.franka_benchmark
# Expected: 40M+ FPS on RTX 4090
```

---

## Contributing Institutions

| Institution | Focus Area |
|-------------|------------|
| MIT | Core development |
| Stanford | Generative AI |
| CMU | Robotics integration |
| ETH Zurich | Locomotion |
| UC Berkeley | RL integration |
| NVIDIA | GPU optimization |
| Tsinghua | Soft body physics |
| Peking University | Rendering |
| + 12 others | Various |

---

## Roadmap (as of 2025)

| Feature | Status |
|---------|--------|
| Rigid body differentiability | In progress |
| Full generative AI release | Gradual rollout |
| Improved contact solver | Active development |
| ROS2 integration | Planned |
| Cloud deployment tools | Planned |

---

## Related Notes

- [[Isaac Gym]] (Primary competitor, NVIDIA ecosystem)
- [[Isaac Lab]] (Modern Isaac successor)
- [[MuJoCo]] (Gold standard for accuracy)
- [[PyBullet]] (Free alternative)
- [[Legged Gym]] (Locomotion training framework)
- [[RSL-RL]] (RL library often used with simulators)
- [[PufferLib]] (RL infrastructure, CPU focus)
- [[Sim2Real]] (End goal of simulation training)
- [[Domain Randomization]] (Training technique)
- [[Reinforcement Learning]]

---

## External Resources

- [Genesis GitHub](https://github.com/Genesis-Embodied-AI/Genesis)
- [Documentation](https://genesis-world.readthedocs.io/)
- [Project Website](https://genesis-embodied-ai.github.io/)
- [Benchmark Controversy Analysis](https://stoneztao.substack.com/p/the-new-hyped-genesis-simulator-is)
- [genesis_lr (Legged Robot Training)](https://github.com/lupinjia/genesis_lr)
- [Taichi Programming Language](https://www.taichi-lang.org/)
- [Genesis Discord](https://discord.gg/genesis-sim)

---

## Summary

Genesis is a promising new entrant in robotics simulation with genuine speed advantages, especially for locomotion tasks. Its multi-solver architecture and cross-platform support are unique strengths. However, benchmark claims should be viewed critically—real-world RL training sees 10-100x lower FPS than headline numbers. For dexterous manipulation, mature alternatives like MuJoCo or Isaac may be safer choices until Genesis's contact solver matures.

**Bottom line**: Worth evaluating for new projects, especially locomotion. Don't migrate existing Isaac/MuJoCo pipelines without testing your specific use case.

---
