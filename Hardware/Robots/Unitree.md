# Unitree

**Unitree Robotics** is a Chinese robotics company that has become the dominant supplier of affordable legged robots for research and education. Their quadrupeds (Go1, Go2, A1, B2) and humanoids (G1, H1) are the most common platforms for [[Sim2Real]] locomotion and manipulation research, appearing in papers from MIT, Stanford, ETH Zurich, Berkeley, and dozens of other labs. Unitree's combination of low cost, open SDKs, and simulation support has made them the de facto standard for academic legged robotics.

---

## Company Overview

| Attribute | Details |
|-----------|---------|
| **Founded** | 2016 |
| **Headquarters** | Hangzhou, China |
| **Focus** | Legged robots (quadrupeds + humanoids) |
| **Notable For** | Making legged robots affordable for research |
| **Key Products** | Go2, G1, H1, B2, A1 |

Unitree disrupted the legged robotics market by offering robots at 1/10th to 1/100th the price of competitors like Boston Dynamics, while maintaining sufficient quality for serious research.

---

## Product Line Overview

### Quadrupeds

| Model | Class | Weight | Price | Target Use |
|-------|-------|--------|-------|------------|
| **Go2** | Consumer/Research | ~15kg | $1,600-$4,500+ | Education, research, hobbyist |
| **Go1** | Consumer/Research | ~12kg | Discontinued | Legacy research |
| **A1** | Research | ~12kg | ~$10,000 | Academic research |
| **B2** | Industrial | ~60kg | ~$100,000 | Industrial inspection |
| **Aliengo** | Research | ~20kg | ~$15,000 | Heavy-duty research |

### Humanoids

| Model | Height | Weight | DOF | Price | Target Use |
|-------|--------|--------|-----|-------|------------|
| **G1** | 127cm | ~35kg | 23-43 | $13,500+ | Research, education |
| **H1** | 180cm | ~47kg | 19 | ~$90,000 | Full-size humanoid research |

---

## Go2 Quadruped (Flagship)

The Go2 is Unitree's most popular robot, used extensively in university courses and RL research.

### Physical Specifications

| Spec | Value |
|------|-------|
| **Standing Dimensions** | 70cm × 31cm × 40cm |
| **Crouching Dimensions** | 76cm × 31cm × 20cm |
| **Weight** | ~15kg (with battery) |
| **Materials** | Aluminum alloy + engineering plastic |

### Performance

| Spec | Value |
|------|-------|
| **Max Speed** | 3.7 m/s (5 m/s lab tested) |
| **Max Climb Angle** | 30-40° |
| **Max Step Height** | 15-16cm |
| **Payload** | 7-8kg standard, 12kg max |

### Power System

| Spec | Value |
|------|-------|
| **Voltage** | 28V-33.6V |
| **Peak Power** | ~3000W |
| **Battery** | 8,000mAh (standard), 15,000mAh (EDU) |
| **Runtime** | 1-2h (standard), 2-4h (EDU) |

### Sensors

| Sensor | Specification |
|--------|---------------|
| **LiDAR** | 4D LiDAR L1, 360°×90° FOV |
| **Min Detection** | 0.05m |
| **Camera** | HD wide-angle |
| **Foot Force** | Force sensors (EDU only) |

### Computing

| Spec | Value |
|------|-------|
| **CPU** | 8-core high-performance |
| **GPU Option** | NVIDIA Jetson Orin (40-100 TOPS) |
| **Connectivity** | WiFi 6, Bluetooth 5.2, 4G+GPS |

### Go2 Variants

| Variant | Price | Key Features |
|---------|-------|--------------|
| **Air** | $1,600 | Basic, app-controlled |
| **Pro** | $2,800 | Higher speed/torque, tracking |
| **X** | $4,500 | Enhanced sensing |
| **EDU** | Contact | Full SDK, ROS2, Jetson Orin |

The **EDU variant** is essential for research—it unlocks:
- Full SDK access (unitree_sdk2)
- ROS2 support
- Extended I/O
- NVIDIA Jetson Orin compute
- Foot-end force sensors

---

## G1 Humanoid

The G1 is Unitree's compact, affordable humanoid designed for manipulation and locomotion research.

### Physical Specifications

| Spec | Value |
|------|-------|
| **Standing Height** | 132cm (4'4") |
| **Folded Dimensions** | 69cm × 45cm × 30cm |
| **Weight** | ~35kg |

### Degrees of Freedom

| Configuration | DOF | Details |
|---------------|-----|---------|
| **G1 Standard** | 23 | 6/leg, 5/arm, 1 waist |
| **G1 EDU** | 23-43 | Expandable waist (3 DOF), hands |

### Joint Specifications

| Joint | Torque | Range |
|-------|--------|-------|
| **Knee** | 90 N·m (G1), 120 N·m (EDU) | 0-165° |
| **Hip Pitch** | High | ±154° |
| **Hip Yaw** | High | ±158° |
| **Waist Z-axis** | Medium | ±155° |
| **Arm Capacity** | - | 2kg (G1), 3kg (EDU) |

### Motor Technology

| Feature | Specification |
|---------|---------------|
| **Type** | Low-inertia high-speed PMSM |
| **Bearings** | Industrial-grade crossed roller |
| **Encoders** | Dual per joint |
| **Cable Routing** | Hollow shaft design |

### Dex3-1 Dexterous Hand (Optional)

| Spec | Value |
|------|-------|
| **DOF** | 7 per hand |
| **Thumb** | 3 active DOF |
| **Index/Middle** | 2 DOF each |
| **Sensors** | Optional tactile arrays |
| **Wrist** | Optional +2 DOF |

### Power & Compute

| Spec | Value |
|------|-------|
| **Battery** | 13S lithium, 9000mAh |
| **Runtime** | ~2 hours |
| **Charger** | 54V/5A |
| **CPU** | 8-core (standard) |
| **GPU** | Jetson Orin (EDU) |

### Sensors

- Depth camera
- 3D LiDAR
- 4-microphone array
- 5W speaker

### G1 Pricing

| Variant | Price | Warranty | Features |
|---------|-------|----------|----------|
| **G1** | $13,500 | 8 months | Standard config |
| **G1 EDU** | Contact | 18 months | Full dev access, expandable DOF |

---

## H1 Humanoid

The H1 is Unitree's full-size humanoid, one of the fastest bipedal robots in existence.

### Physical Specifications

| Spec | Value |
|------|-------|
| **Height** | 180cm (5'11") |
| **Weight** | ~47kg |
| **Price** | ~$90,000 |

### Degrees of Freedom

| Region | DOF |
|--------|-----|
| **Legs** | 5 per leg (10 total) |
| **Arms** | 4 per arm (8 total) |
| **Waist** | 1 |
| **Total** | 19 |

### Performance Records

| Metric | Value | Note |
|--------|-------|------|
| **Walking Speed** | 1.5 m/s | Stable |
| **Running Speed** | 3.3+ m/s | Record-breaking for humanoids |

### M107 Joint Motor

The H1 uses Unitree's flagship M107 motor:

| Spec | Value |
|------|-------|
| **Max Torque (Knee)** | 360 N·m |
| **Max Torque (Hip)** | 220 N·m |
| **Max Torque (Ankle)** | 45 N·m |
| **Max Tension** | 10,000 N |
| **Torque/Weight** | 189 N·m/kg |
| **Dimensions** | 107mm × 74mm |

---

## Simulation Support

Unitree provides extensive simulation tools, making their robots ideal for [[Sim2Real]] research.

### Official Repositories

| Repository | Purpose | Robots |
|------------|---------|--------|
| **unitree_mujoco** | MuJoCo simulation | Go2, H1, G1 |
| **unitree_rl_gym** | Isaac Gym training | Go2, H1, H1_2, G1 |
| **unitree_rl_lab** | Isaac Lab training | Go2, H1, G1 |
| **unitree_ros2** | ROS2 interface | Go2, B2 |
| **unitree_sdk2** | Low-level control | All |

### unitree_rl_gym Workflow

```
Train (Isaac Gym) → Play → Sim2Sim (MuJoCo) → Sim2Real (Hardware)
```

Dependencies:
- [[Legged Gym]] / legged_gym
- [[RSL-RL]] / rsl_rl
- [[MuJoCo]]
- unitree_sdk2_python

### Supported Simulators

| Simulator | Support Level | Notes |
|-----------|---------------|-------|
| [[Isaac Gym]] | Official | unitree_rl_gym |
| [[Isaac Lab]] | Official | unitree_rl_lab |
| [[MuJoCo]] | Official | unitree_mujoco, Playground |
| [[Genesis]] | Community | genesis_lr project |
| [[PyBullet]] | Community | Various implementations |
| Gazebo/ROS2 | Official | unitree_ros2 |

### MuJoCo Playground Support

MuJoCo Playground includes official Unitree models:
- **Quadrupeds**: Go1
- **Humanoids**: H1, G1

All with validated sim2real locomotion policies.

---

## Research Applications

Unitree robots appear in hundreds of academic papers. Key research areas:

### Locomotion

| Paper/Project | Robot | Contribution |
|---------------|-------|--------------|
| **Walk-These-Ways** (MIT) | Go1 | Multi-gait locomotion |
| **Rapid Sim-to-Real** | G1 | 15-min training to deployment |
| **ExBody** | H1 | Expressive whole-body control |
| **HumanUP** | G1 | Getting-up from falls |
| **RMA** | A1 | Rapid motor adaptation |

### Manipulation

| Paper/Project | Robot | Contribution |
|---------------|-------|--------------|
| **HOMIE** | H1_2 | Humanoid loco-manipulation |
| **DreamControl** | G1 | Scene interaction |
| **VR-Robo** | Go2, G1 | Visual navigation |

### Key Techniques Validated on Unitree

| Technique | Description |
|-----------|-------------|
| **Domain Randomization** | Dynamics, terrain, perturbations |
| **Teacher-Student** | Privileged info distillation |
| **RMA** | Online adaptation modules |
| **Sim2Sim Validation** | Isaac → MuJoCo → Real |

---

## Comparison: Quadruped Robots

### Research Quadrupeds

| Robot | Manufacturer | Weight | Price | Simulation | Research Use |
|-------|--------------|--------|-------|------------|--------------|
| **Go2 EDU** | Unitree | 15kg | ~$5K+ | Excellent | Very High |
| **A1** | Unitree | 12kg | ~$10K | Excellent | High |
| **Spot** | Boston Dynamics | 32kg | ~$75K | Limited | Medium |
| **ANYmal** | ANYbotics | 50kg | ~$100K+ | Excellent | High |
| **Mini Cheetah** | MIT | 9kg | Custom | Good | Medium |
| **Laikago** | Unitree | 22kg | ~$20K | Good | Legacy |

### Feature Comparison

| Feature | Go2 EDU | Spot | ANYmal C |
|---------|---------|------|----------|
| **Open SDK** | Yes | Limited | Yes |
| **ROS2** | Yes | Yes | Yes |
| **Isaac Gym** | Official | Community | Official |
| **Price** | Low | High | Very High |
| **Payload** | 8kg | 14kg | 10kg |
| **Speed** | 5 m/s | 1.6 m/s | 1 m/s |

---

## Comparison: Humanoid Robots

### Research Humanoids

| Robot | Manufacturer | Height | DOF | Price | Availability |
|-------|--------------|--------|-----|-------|--------------|
| **G1** | Unitree | 127cm | 23-43 | $13.5K | Available |
| **H1** | Unitree | 180cm | 19 | ~$90K | Available |
| **Atlas** | Boston Dynamics | 150cm | 28 | N/A | Not sold |
| **Digit** | Agility | 175cm | 20+ | ~$150K | Limited |
| **Figure 01** | Figure | 170cm | 40+ | N/A | Not available |
| **Optimus** | Tesla | 173cm | 28 | TBD | Not available |

### Simulation Support Comparison

| Robot | Isaac Lab | MuJoCo | Official Models | Sim2Real Papers |
|-------|-----------|--------|-----------------|-----------------|
| **G1** | Yes | Yes | Yes | Many |
| **H1** | Yes | Yes | Yes | Many |
| **Atlas** | No | Community | No | Few (internal) |
| **Digit** | Limited | Community | No | Some |

---

## SDK and Development

### unitree_sdk2

The primary SDK for all modern Unitree robots:

```python
from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_

# Low-level motor control
publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
cmd = LowCmd_()
cmd.motor_cmd[0].q = target_position
cmd.motor_cmd[0].kp = 50.0
cmd.motor_cmd[0].kd = 1.0
publisher.write(cmd)
```

### Control Modes

| Mode | Description | Use Case |
|------|-------------|----------|
| **High-Level** | Velocity commands | Navigation, teleoperation |
| **Low-Level** | Joint position/torque | Custom controllers, RL |
| **Sport Mode** | Built-in gaits | Demos, basic locomotion |

### ROS2 Integration

```bash
# Install unitree_ros2
cd ~/ros2_ws/src
git clone https://github.com/unitreerobotics/unitree_ros2
colcon build

# Launch Go2 interface
ros2 launch unitree_ros2 go2.launch.py
```

---

## Training Pipeline (Typical)

### 1. Simulation Training

```python
# unitree_rl_gym example
python train.py --task=go2 --num_envs=4096
# Trains in Isaac Gym with RSL-RL
```

### 2. Sim2Sim Validation

```python
# Test in MuJoCo before hardware
python sim2sim.py --checkpoint=model.pt
# Validates policy transfers across simulators
```

### 3. Hardware Deployment

```python
# Deploy via unitree_sdk2
python deploy.py --robot=go2 --policy=model.pt
# Real-time control at 500Hz
```

### 4. Key Considerations

| Stage | Concern | Solution |
|-------|---------|----------|
| **Training** | Sim2real gap | Domain randomization |
| **Sim2Sim** | Physics differences | Motor modeling |
| **Deployment** | Latency | C++ inference (LibTorch) |
| **Safety** | Falls, collisions | Soft starts, kill switches |

---

## Strengths

| Strength | Details |
|----------|---------|
| **Price** | 10-100x cheaper than alternatives |
| **SDK Quality** | Well-documented, actively maintained |
| **Simulation** | Official Isaac/MuJoCo support |
| **Community** | Large research community |
| **Iteration** | Rapid hardware updates |
| **Availability** | Actually purchasable |

---

## Weaknesses

| Weakness | Details |
|----------|---------|
| **Build Quality** | Below Boston Dynamics |
| **Durability** | Not industrial-grade (except B2) |
| **Support** | Response times vary |
| **Documentation** | Some gaps, Chinese-first |
| **Waterproofing** | Limited (IP54 on some) |

---

## Purchasing Considerations

### For Research Labs

| Need | Recommendation |
|------|----------------|
| **Locomotion research** | Go2 EDU or G1 EDU |
| **Manipulation research** | G1 EDU with Dex3-1 |
| **Full humanoid** | H1 |
| **Teaching/courses** | Go2 Pro or EDU |
| **Budget-constrained** | Go2 Air + manual SDK unlock |

### EDU vs Standard

| Feature | Standard | EDU |
|---------|----------|-----|
| SDK Access | Limited | Full |
| ROS2 | No | Yes |
| Jetson Orin | No | Yes |
| Force Sensors | No | Yes |
| Warranty | 8 months | 18 months |
| Price | Lower | Higher |

---

## Related Notes

- [[Legged Gym]] (Training framework)
- [[RSL-RL]] (RL library for Unitree)
- [[Isaac Gym]] (Primary simulator)
- [[Isaac Lab]] (Modern simulator)
- [[MuJoCo]] (Sim2sim validation)
- [[Genesis]] (Alternative simulator)
- [[Sim2Real]] (Deployment goal)
- [[Domain Randomization]] (Training technique)
- [[PufferLib]] (RL infrastructure)
- [[Reinforcement Learning]]

---

## External Resources

### Official

- [Unitree Website](https://www.unitree.com/)
- [Unitree GitHub](https://github.com/unitreerobotics)
- [Go2 Product Page](https://www.unitree.com/go2/)
- [G1 Product Page](https://www.unitree.com/g1/)
- [H1 Product Page](https://www.unitree.com/h1/)

### Repositories

- [unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym) - Isaac Gym training
- [unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab) - Isaac Lab training
- [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco) - MuJoCo simulation
- [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) - ROS2 interface

### Community

- [MuJoCo Playground](https://github.com/google-deepmind/mujoco_playground) - DeepMind's Unitree models
- [genesis_lr](https://github.com/lupinjia/genesis_lr) - Genesis + Unitree
- [awesome-humanoid-learning](https://github.com/jonyzhang2023/awesome-humanoid-learning) - Humanoid resources
- [quadruped_ros2_control](https://github.com/legubiao/quadruped_ros2_control) - ROS2 control

### Research

- [Walk-These-Ways (MIT)](https://github.com/Improbable-AI/walk-these-ways)
- [Rapid Sim-to-Real Humanoid](https://arxiv.org/abs/2512.01996)
- [ExBody: Expressive Whole-Body Control](https://arxiv.org/abs/2402.16796)

---

## Summary

Unitree has democratized legged robotics research by making capable quadrupeds and humanoids accessible to academic labs worldwide. The Go2 EDU (~$5K) and G1 EDU (~$15K+) offer the best value for [[Sim2Real]] research, with official simulation support in [[Isaac Gym]], [[Isaac Lab]], and [[MuJoCo]]. For anyone starting in legged robotics or locomotion RL, Unitree robots are the default choice—they're what most papers use, most tutorials target, and most labs own.

**Bottom line**: If you're doing locomotion or humanoid research, you'll likely end up with a Unitree robot. Start with Go2 EDU for quadruped work or G1 EDU for humanoid manipulation.

---
