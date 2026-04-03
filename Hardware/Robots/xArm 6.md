# xArm 6

The **UFACTORY xArm 6** is a 6-DOF collaborative robotic arm that has become the go-to choice for developers, researchers, and small businesses needing an affordable cobot with excellent software support. At ~$8,900, it offers 5kg payload, ±0.1mm repeatability, and arguably the best ROS/Python/C++ ecosystem of any Chinese robot arm.

**Why it stands out**: While many Chinese cobots offer similar specs, UFACTORY prioritizes developers with comprehensive SDKs, active GitHub repositories, and genuine ROS2/MoveIt integration. The arm weighs only 12.2kg (half of competitors like Elfin E5), making it practical for mobile platforms and frequent repositioning.

---

## ⚙️ Technical Specifications

### Arm Performance

| Specification | Value |
|---------------|-------|
| Degrees of Freedom | 6 |
| Payload | 5 kg (11 lbs) |
| Reach | 700 mm (27.6 in) |
| Repeatability | ±0.1 mm |
| Max TCP Speed | 1000 mm/s |
| Max Joint Speed | 180°/s |

### Physical Properties

| Specification | Value |
|---------------|-------|
| Arm Weight | 12.2 kg (26.9 lbs) |
| Footprint | Ø 126 mm |
| Materials | Aluminum, Carbon Fiber |
| IP Rating | IP54 |
| ISO Cleanroom | Class 5 |
| Mounting | Any orientation (floor, ceiling, wall) |
| Tool Connector | M5 × 6 |

### Electrical

| Specification | Value |
|---------------|-------|
| Power Consumption | 8.4W min / 120W typical / 240W max |
| Input Power | 24V DC, 15A |
| Operating Temperature | 0-50°C |

### Joint Specifications

| Joint | Range | Max Speed | Max Torque |
|-------|-------|-----------|------------|
| J1 (Base) | ±360° | 180°/s | 56 Nm |
| J2 (Shoulder) | -118° to +120° | 180°/s | 56 Nm |
| J3 (Elbow) | -225° to +11° | 180°/s | 28 Nm |
| J4 (Wrist 1) | ±360° | 180°/s | 12 Nm |
| J5 (Wrist 2) | -97° to +180° | 180°/s | 12 Nm |
| J6 (Wrist 3) | ±360° | 180°/s | 12 Nm |

### Drive System

The xArm uses a compact powertrain with:
- Outer rotor [[BLDC]] motors (self-developed)
- [[Harmonic Drive]] reducers (built-in)
- 17-bit multi-turn absolute encoders
- Integrated motor drivers

---

## 📦 Control Box Options

### AC Control Box (Standard)

| Specification | Value |
|---------------|-------|
| Input | 100-240 VAC, 50/60 Hz |
| Output | 24 VDC, 16.5A |
| Dimensions | 280 × 200 × 116 mm |
| Weight | 3.5 kg |
| Digital I/O | 8 in, 8 out |
| Analog I/O | 2 in, 2 out |

### DC Control Box (Mobile/Battery Applications)

| Specification | Value |
|---------------|-------|
| Input | 24 VDC, 16.5A |
| Dimensions | 180 × 145 × 68 mm |
| Weight | 1.8 kg |

The DC control box is ideal for mobile robots, AGVs, and battery-powered applications where AC power isn't available.

---

## 💻 Software Ecosystem

### UFACTORY Studio (GUI)

The graphical interface for non-programmers:
- Live control with 3D visualization
- Blockly visual programming
- G-code support
- Point teaching and trajectory recording
- Integrated simulation
- Settings and calibration

### Python SDK

**Installation:**
```bash
pip install xarm-python-sdk
```

**Basic usage:**
```python
from xarm.wrapper import XArmAPI

# Connect to robot
arm = XArmAPI('192.168.1.221')

# Initialize
arm.motion_enable(enable=True)
arm.set_mode(0)  # Position mode
arm.set_state(0)  # Sport state

# Move to position (x, y, z, roll, pitch, yaw)
arm.set_position(x=300, y=0, z=200, roll=180, pitch=0, yaw=0, wait=True)

# Move joints (degrees)
arm.set_servo_angle(angle=[0, -45, 0, 0, 45, 0], wait=True)

# Close gripper
arm.set_gripper_position(0, wait=True)

# Disconnect
arm.disconnect()
```

**Key SDK Features:**
- Motion control (linear, arc, circular, joint)
- Velocity/acceleration control
- Trajectory recording and playback
- Gripper control
- GPIO access
- Force/torque sensor integration
- Real-time servo control (1ms loop)
- Collision detection configuration
- Load identification

**SDK Versions** (as of late 2025):
- Latest: 1.17.3
- Python: 3.5 - 3.13 supported
- License: BSD 3-Clause

### C++ SDK

**Linux build:**
```bash
git clone https://github.com/xArm-Developer/xArm-CPLUS-SDK.git
cd xArm-CPLUS-SDK
make xarm           # Build library
make test           # Build all examples
sudo make install   # Install library
./build/example/0002-get_property 192.168.1.221
```

**Windows:** Use Visual Studio 2015+ with C++ environment.

**Example structure:**
- 0001-0004: Basic operations (events, properties, servo)
- 1000s: Movement (line, arc, velocity control)
- 2000s: Joint control
- 3000s: Trajectory recording/playback
- 5000s: GPIO and gripper control
- 6000s: Safety features (reduced mode, fence)
- 7000s: Real-time servo control
- 8000s: Load identification

### ROS2 Integration

**Installation:**
```bash
# Clone for your ROS distro (humble, iron, jazzy)
git clone https://github.com/xArm-Developer/xarm_ros2.git --recursive -b $ROS_DISTRO
cd ~/catkin_ws
colcon build
```

**Features:**
- Full [[MoveIt]] integration
- Gazebo simulation
- RViz visualization
- Dual-arm control support
- Real robot and simulation parity
- xarm_planner for programmatic MoveIt access

**Network Recommendation:** Use a DIRECT Ethernet cable between controller and PC. Switches/routers can cause latency that impacts trajectory execution.

### Other Protocols

| Protocol | Use Case |
|----------|----------|
| Modbus TCP | PLC integration |
| Modbus RTU | RS-485 devices |
| WebSocket | Web interfaces |
| G-code | CNC-style programming |
| Private TCP | Low-level direct control |

---

## 📊 Comparison with Similar Arms

| Model | DOF | Payload | Reach | Repeatability | Weight | Price | ROS2 |
|-------|-----|---------|-------|---------------|--------|-------|------|
| **xArm 6** | 6 | 5 kg | 700 mm | ±0.1 mm | 12.2 kg | $8,900 | ✅ |
| xArm 5 Lite | 5 | 3 kg | 700 mm | ±0.1 mm | 11.2 kg | $4,000 | ✅ |
| xArm 7 | 7 | 3.5 kg | 700 mm | ±0.1 mm | 13.7 kg | $12,000 | ✅ |
| Dobot CR5 | 6 | 5 kg | 910 mm | ±0.02 mm | 17 kg | $8,500 | ✅ |
| AUBO i5 | 6 | 5 kg | 886 mm | ±0.02 mm | 24 kg | $7,500 | ✅ |
| JAKA Zu 5 | 6 | 5 kg | 800 mm | ±0.03 mm | 18 kg | $7,000 | ⚠️ |
| Han's Elfin E05 | 6 | 5 kg | 800 mm | ±0.02 mm | 23 kg | $8,000 | ⚠️ |
| UR5e | 6 | 5 kg | 850 mm | ±0.03 mm | 20.6 kg | $35,000 | ✅ |

**Key differentiator:** The xArm 6 is ~half the weight of competitors at the same payload class. Its 12.2 kg makes it practical for mobile platforms where weight matters.

---

## 🔧 End Effectors & Accessories

### Official UFACTORY Grippers

| Gripper | Payload | Stroke | Features | Price Est |
|---------|---------|--------|----------|-----------|
| xArm Gripper | 1.5 kg | 50 mm | Position/force control | $400-600 |
| xArm Vacuum Gripper | 4 kg | - | -90 kPa max | $500-700 |
| BIO Gripper | 500 g | 55 mm | Adaptive fingers | $600-800 |

### Third-Party Compatible

- Robotiq 2F-85/2F-140
- OnRobot RG2/RG6
- Custom end effectors via tool flange

### Other Accessories

| Accessory | Description |
|-----------|-------------|
| 6-Axis Force/Torque Sensor | For force-controlled applications |
| Intel RealSense D435 Mount | Camera mounting plate |
| 15m Cable Extensions | Longer power/signal runs |
| Linear Rail (7th axis) | Extended workspace |
| Dual-arm Mount | Two arms on one base |

---

## 🛠️ Setup & Installation

### Initial Setup Checklist

1. **Mounting**: Secure to surface that can withstand 10× base joint torque
   - Use all six M5 bolts through ∅5.5 holes
   - Floor, wall, or ceiling mount all supported

2. **Connections**:
   - Connect arm to control box via provided cable
   - Connect control box to power (AC or DC depending on model)
   - Connect Ethernet from control box to PC

3. **Network Configuration**:
   - Default IP: 192.168.1.xxx (check label on control box)
   - Set PC to same subnet (e.g., 192.168.1.100)
   - Direct connection recommended (no switches)

4. **Software**:
   - Download UFACTORY Studio from ufactory.cc
   - Or install Python SDK: `pip install xarm-python-sdk`

5. **First Power-On**:
   - Enable servos
   - Clear any faults
   - Home the robot (if required)
   - Test motion in slow mode first

### Safety Considerations

- Perform safety assessment before each installation/debug
- Keep personnel away during initial testing
- Configure collision detection and speed limits
- Use reduced mode for teaching/programming
- Emergency stop must be accessible

---

## ✅ Strengths

- **Best-in-class SDK**: Python, C++, ROS2 all actively maintained
- **Lightweight**: 12.2 kg is half of competitors
- **Developer community**: Active GitHub, forums, documentation
- **Direct sales**: No integrator markup at ufactory.cc
- **Mounting flexibility**: Any orientation
- **DC option**: Battery/mobile platform compatible
- **Price**: ~75% cheaper than UR5e at similar specs
- **Repeatability**: ±0.1mm is solid for the price

---

## ❌ Weaknesses

- **Reach**: 700mm is shorter than some competitors (AUBO i5 has 886mm)
- **Support**: Not as established as UR/FANUC for production issues
- **Repeatability vs. Accuracy**: ±0.1mm repeatability, but absolute accuracy is larger
- **Spare parts**: May take weeks from China
- **Documentation**: Some sections Chinese-first, English translations vary
- **No larger models**: UFACTORY caps at 5kg payload (xArm 7 is only 3.5kg)

---

## 🔧 Use Cases

- **Research & Education**: Best SDK for academic robotics labs
- **Pick and Place**: 5kg handles most small parts
- **Machine Tending**: CNC load/unload
- **Assembly**: Light assembly tasks
- **Testing/QA**: Automated test fixtures
- **Mobile Manipulation**: Lightweight for mobile bases
- **Vision-Guided Applications**: RealSense integration
- **Prototyping**: Rapid proof-of-concept development

---

## 💰 Pricing & Where to Buy

### Configurations

| Configuration | Price (USD) |
|---------------|-------------|
| xArm 6 + AC Control Box | ~$8,900 |
| xArm 6 + DC Control Box | ~$8,700 |
| xArm Gripper | ~$500 |
| Vacuum Gripper | ~$600 |
| 6-Axis F/T Sensor | ~$1,500 |

### Vendors

| Vendor | Region | Notes |
|--------|--------|-------|
| [UFACTORY Direct](https://www.ufactory.cc) | Global | Best price, direct support |
| [UFACTORY US](https://www.ufactory.us) | USA | US warehouse, faster shipping |
| [Top3DShop](https://top3dshop.com) | USA/EU | Sometimes has discounts |
| [RobotShop](https://www.robotshop.com) | Global | Established retailer |
| [Blue Sky Robotics](https://www.blueskyrobotics.ai) | USA | Integration services |

**Tip**: Buy direct from UFACTORY for best price and direct technical support access.

---

## 🔗 Related Concepts

- [[Chinese Robot Arms]] - Market overview
- [[SO-ARM100]] - Open source budget alternative
- [[MoveIt]] - Motion planning framework
- [[ROS2]] - Robot Operating System 2
- [[Harmonic Drive]] - Gearbox technology used
- [[BLDC]] - Motor type used
- [[FOC]] - Motor control method
- [[Inverse Kinematics]] - Motion calculation
- [[PID Controller]] - Control fundamentals
- [[Servo Motor]] - Actuator fundamentals
- [[Robot Kits]] - Other robot options

---

## 📚 External Resources

### Official

- [UFACTORY Website](https://www.ufactory.cc/)
- [UFACTORY Documentation](https://docs.ufactory.cc/)
- [UFACTORY Forum](https://forum.ufactory.cc/)
- [UFACTORY Downloads](https://www.ufactory.us/downloads)

### GitHub Repositories

- [xArm-Python-SDK](https://github.com/xArm-Developer/xArm-Python-SDK)
- [xArm-CPLUS-SDK](https://github.com/xArm-Developer/xArm-CPLUS-SDK)
- [xarm_ros2](https://github.com/xArm-Developer/xarm_ros2)
- [xarm_ros](https://github.com/xArm-Developer/xarm_ros)

### PyPI

- [xarm-python-sdk on PyPI](https://pypi.org/project/xarm-python-sdk/)

### Community

- [UFACTORY Forum](https://forum.ufactory.cc/) - Official support forum
- [r/robotics](https://reddit.com/r/robotics) - General robotics discussion
- [ROS Discourse](https://discourse.ros.org/) - ROS community

### Manuals

- [xArm User Manual (PDF)](https://www.ufactory.cc/wp-content/uploads/2023/05/xArm-User-Manual-V2.0.0.pdf)
- [Technical Specifications](https://docs.xarm.ufactory.cc/8.technical_specifications.html)
