# Inverse Kinematics

**Inverse Kinematics (IK)** is the process of determining the joint parameters (such as angles or positions) required to place the end-effector of a robot at a desired position and orientation in space. It is the inverse of forward kinematics, which calculates the end-effector pose from known joint values.

IK is a foundational concept in robotics, animation, biomechanics, and control systems, enabling goal-directed movement for articulated systems.

---

## 🧠 Core Concepts

- `Forward Kinematics`: Computes the end-effector pose from joint states  
- `Inverse Kinematics`: Computes joint states from desired end-effector pose  
- `Analytical IK`: Closed-form solution, fast but only works for specific geometries  
- `Numerical IK`: Iterative optimization methods, slower but more flexible  
- `Redundancy`: Robots with more DOF than task dimensions can have infinite solutions  
- `Singularities`: Problematic configurations where small output motion requires large joint changes  
- `Constraints`: Joint limits, preferred configurations, collision avoidance  

---

## 🧰 Use Cases

- Robotic arm manipulation and pick-and-place  
- Humanoid walking and balance control  
- Camera reorientation and gaze tracking  
- Animation of characters and avatars  
- Exoskeleton and prosthetic control  
- Human motion imitation and teleoperation  

---

## ✅ Pros

- Allows precise end-effector control in task space  
- Applicable across many types of robots and linkages  
- Can be used with optimization for constraint-aware behavior  
- Widely supported in simulators and motion planning libraries  

---

## ❌ Cons

- No solution may exist for certain goals (e.g. out of reach)  
- Multiple solutions can be ambiguous or unstable  
- Performance degrades near singularities  
- Requires accurate robot models (e.g. from `URDF`)  

---

## 📊 Comparison of IK Solvers

| Solver         | Type       | Speed    | Flexibility | Notes                                   |
|----------------|------------|----------|-------------|-----------------------------------------|
| IKFast         | Analytical | Very Fast| Low         | Precomputed, limited to known geometries |
| KDL            | Analytical | Fast     | Medium      | Built into many ROS systems             |
| TRAC-IK        | Hybrid     | Medium   | High        | Handles redundancy and joint limits     |
| MoveIt!        | Framework  | Varies   | Very High   | Can use TRAC-IK, KDL, or custom plugins |
| PyBullet IK    | Numerical  | Medium   | Medium      | Easy to use for simulation tasks        |
| Pinocchio      | Numerical  | High     | High        | Fast and used in advanced research      |

---

## 🤖 In Robotics Context

| Application              | Why IK Matters                             |
|--------------------------|--------------------------------------------|
| Robotic manipulation     | Enables end-effector positioning           |
| Grasp planning           | Determines joint poses for grasps          |
| Mobile manipulation      | Combines base motion with arm motion       |
| Gait generation          | Controls leg movement in walking robots    |
| Motion imitation         | Maps human poses to robot configurations   |

---

## 🔧 Compatible Items

- [[Forward Kinematics]] – Required for iterative IK solutions  
- [[URDF]] – Robot model format used in solvers like MoveIt and PyBullet  
- [[MoveIt!]] – Full motion planning framework with IK backends  
- [[TRAC-IK]] – Fast IK solver plugin for ROS  
- [[PyBullet]] – Includes built-in numerical IK solver  
- [[Pinocchio]] – Modern kinematics/dynamics library with IK support  

---

## 🔗 Related Concepts

- [[Forward Kinematics]] (Direct calculation of pose from joint values)  
- [[Robot Arm Kinematics]] (Includes IK as a subproblem)  
- [[Trajectory Planning]] (Often begins and ends with IK poses)  
- [[MoveIt]] (Framework for planning and IK solving)  
- [[URDF]] (Robot model needed for solving IK)  

---

## 📚 Further Reading

- [MoveIt Kinematics Tutorial](https://moveit.picknik.ai/main/doc/kinematics/kinematics_tutorial.html)  
- [TRAC-IK GitHub](https://github.com/ros-industrial-consortium/trac_ik)  
- [Pinocchio Library](https://stack-of-tasks.github.io/pinocchio/)  
- [PyBullet IK Examples](https://github.com/bulletphysics/bullet3/tree/master/examples/pybullet/examples)  
- [OpenRAVE IKFast Docs](http://openrave.org/docs/latest_stable/openravepy/ikfast/)  

---
