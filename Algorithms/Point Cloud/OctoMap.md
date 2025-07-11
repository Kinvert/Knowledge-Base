# OctoMap

**OctoMap** is a probabilistic 3D mapping framework that represents environments using an **octree-based occupancy grid**. It enables efficient storage, updating, and querying of volumetric information from 3D sensor data, such as LiDAR or depth cameras, and is widely used in robotics for navigation, obstacle avoidance, and SLAM.

---

## 📚 Overview

OctoMap compresses 3D space into a hierarchical tree of voxels (volumetric pixels), allowing memory-efficient representation of both occupied and free space. Each voxel stores a probability of occupancy, allowing the system to reason about sensor noise and uncertain observations. The resolution is adaptive: finer where needed, coarser elsewhere.

---

## 🧠 Core Concepts

- **Octree Structure**: Space is recursively divided into octants; nodes represent voxels  
- **Probabilistic Occupancy**: Each voxel holds a log-odds probability of being occupied  
- **Sensor Integration**: Point clouds from sensors are inserted into the map with raycasting  
- **Memory Efficiency**: Sparse data storage; only occupied or uncertain regions stored  
- **3D Querying**: Allows fast collision checks, nearest neighbor queries, and visibility testing  
- **Resolution Scaling**: Tree depth controls map resolution dynamically  

---

## 🧰 Use Cases

- 3D obstacle mapping for mobile robots and UAVs  
- Environment modeling for autonomous navigation  
- Collision checking in 3D space  
- Loop closure correction in SLAM pipelines  
- Autonomous exploration and path planning  

---

## ✅ Pros

- Scales well to large environments with adaptive resolution  
- Probabilistic reasoning handles sensor noise well  
- Efficient memory usage via sparse octree representation  
- Actively maintained with ROS/ROS2 integration  
- Useful for both online and offline mapping  

---

## ❌ Cons

- Assumes a static environment unless explicitly updated  
- May require tuning for sensor parameters and resolution  
- Not ideal for deformable or dynamic objects  
- Raycasting can be computationally intensive at high resolutions  

---

## 📊 Comparison Chart: OctoMap vs Other 3D Mapping Frameworks

| Feature            | OctoMap         | Voxblox         | Truncated Signed Distance Field (TSDF) | Elevation Maps    | Grid Maps (2D)   |
|--------------------|-----------------|-----------------|----------------------------------------|-------------------|------------------|
| Dimensionality     | 3D              | 3D              | 3D                                     | 2.5D              | 2D               |
| Representation     | Octree (Voxel)  | Voxels (ESDF)   | Signed Distance Function               | Grid + Height     | Grid             |
| Probabilistic      | Yes             | No              | No                                     | Often             | No               |
| Memory Usage       | Efficient       | Moderate        | High                                   | Moderate          | Low              |
| Dynamic Support    | Limited         | Some            | Yes (with updates)                     | No                | Yes              |
| ROS Integration    | Excellent       | Good            | Varies                                 | Good              | Excellent        |

---

## 🤖 In a Robotics Context

| Scenario                        | OctoMap Role                                  |
|--------------------------------|-----------------------------------------------|
| Autonomous drone navigation    | Build and query 3D occupancy maps             |
| SLAM systems                   | Store consistent volumetric map               |
| Collision avoidance            | Real-time checking for safe paths             |
| Loop closure adjustments       | Reconstruct local regions with updated scans  |
| Warehouse automation           | 3D map for stacking, shelving, and AGV routing|

---

## 🔧 Useful Commands / Interfaces

- `octomap::OcTree` – Core class for building the octree  
- `octomap_server` – ROS node for publishing 3D occupancy maps  
- `ros2 run octomap_server octomap_server_node` – Launch OctoMap server in ROS 2  
- `octomap_msgs/Octomap` – ROS message type for transmitting map data  
- `ros2 topic echo /octomap_full` – View full octree stream from OctoMap  

---

## 🔧 Compatible Items

- [[LiDAR]] and [[Depth Camera]] sensors for 3D point cloud input  
- [[ROS2]] and [[RViz]] for visualization and integration  
- [[ICP]] – Can align point clouds before insertion into OctoMap  
- [[Voxel Grid]] filters to pre-process data  
- [[Navigation Stack]] to use OctoMap for planning  

---

## 🔗 Related Concepts

- [[Voxel Grid]] (Basic building block of volumetric maps)  
- [[ICP]] (Alignment before insertion)  
- [[SLAM]] (Maps built using scan matching)  
- [[3D Mapping]] (General category OctoMap belongs to)  
- [[Collision Checking]] (OctoMap used for collision detection)  

---

## 📚 Further Reading

- [OctoMap.org](https://octomap.github.io/) – Official documentation and tutorials  
- [OctoMap GitHub Repository](https://github.com/OctoMap/octomap)  
- [ROS OctoMap Wiki](http://wiki.ros.org/octomap)  
- [OctoMap Paper (2010)](https://octomap.github.io/octomap.pdf)  
- [Efficient Probabilistic 3D Mapping Framework (Hornung et al.)](https://ieeexplore.ieee.org/document/6202556)  

---
