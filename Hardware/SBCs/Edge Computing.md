# 🌐 Edge Computing

**Edge Computing** refers to the practice of performing data processing close to the source of data — at the "edge" of the network — rather than in a centralized cloud or data center. It is especially useful in applications requiring **low latency**, **offline capabilities**, or **bandwidth conservation**.

---

## 🧠 Summary

- Processes data **locally** on edge devices (e.g., sensors, gateways, embedded computers)
- Reduces reliance on cloud or central servers
- Enables **real-time responses**, even with limited connectivity
- Common in **IoT**, **autonomous systems**, **smart manufacturing**, and **retail**

---

## 🧭 Core Characteristics

| Feature                 | Description                                                                 |
|-------------------------|-----------------------------------------------------------------------------|
| **Decentralization**    | Processing occurs near data sources, not centralized data centers           |
| **Low Latency**         | Near-instantaneous data handling for time-sensitive applications            |
| **Bandwidth Efficient** | Limits unnecessary transmission of raw data to the cloud                    |
| **Resilience**          | Systems can operate independently in the event of network disruptions       |
| **Context-Aware**       | Often tightly coupled with local sensors and actuators                      |

---

## ⚙️ Hardware Commonly Used

| Device                          | Notes                                                  |
|----------------------------------|---------------------------------------------------------|
| [[Jetson Nano]] / [[Jetson Family]] | Popular in AI inference at the edge                     |
| [[Raspberry Pi]]               | Affordable and highly flexible general-purpose SBC       |
| [[AMD SBCs with GPU]]          | x86-based, GPU-accelerated SBCs for edge workloads       |
| Google Coral Dev Board         | Edge TPU-powered device for ML workloads                 |
| NVIDIA Jetson Orin             | High-performance AI processing at the edge               |
| Intel NUC                      | Small form-factor x86 PC, often used in industrial edge  |
| [[ESP32]] / RP2040                 | Used for sensor-level edge processing in IoT             |
| Edge Gateways (e.g., Advantech, OnLogic) | Industrial-grade edge computers with I/O interfaces  |

---

## 🧰 Software & Frameworks

| Tool / Framework     | Description                                                                 |
|----------------------|-----------------------------------------------------------------------------|
| [[ROS2]]             | Common in robotics; edge nodes process sensor data or control actuators     |
| [[TensorFlow Lite]]  | Optimized for lightweight inference on embedded devices                     |
| [[ONNX Runtime]]     | Efficient cross-platform inference engine for edge AI models                |
| [[Docker]]           | Used to containerize applications at the edge                               |
| [[Kubernetes]] (K3s) | Lightweight cluster management for edge deployments                         |
| Azure IoT Edge       | Microsoft’s platform for cloud-connected edge devices                       |
| AWS Greengrass       | Amazon’s edge processing and messaging framework                            |
| OpenVINO             | Intel’s toolkit for deploying AI at the edge                                |

---

## 🚀 Use Cases

- **Autonomous Vehicles** – Perform perception and control onboard in real time  
- **Industrial Automation** – Edge devices monitor, analyze, and act on factory data  
- **Smart Cities** – Edge nodes on cameras/sensors process and respond without cloud delay  
- **Retail Analytics** – Edge-based vision systems for customer tracking or shelf monitoring  
- **Healthcare** – Patient monitoring and alerts with local computation  

---

## 📊 Comparison to Cloud Computing

| Feature               | Edge Computing                      | Cloud Computing                  |
|-----------------------|--------------------------------------|----------------------------------|
| Latency               | Very low (ms)                        | High (100ms–sec)                 |
| Internet Dependency   | Often optional                       | Required                         |
| Data Privacy          | High (local storage)                 | Variable                         |
| Compute Capacity      | Lower                                | Very high                        |
| Scalability           | Harder to scale physically           | Easy to scale horizontally       |
| Cost                  | Low for small deployments            | Can be high with data egress     |

---

## 🏆 Strengths

- Reduced latency, improved responsiveness
- More privacy and data control
- Bandwidth savings
- Operates in disconnected environments

---

## ⚠️ Weaknesses

- Requires maintenance of many physical devices
- Less raw compute power compared to cloud
- Hardware failures are harder to mitigate
- Software updates and deployment are more complex

---

## 🔗 Related Notes

- [[Jetson Family]]
- [[SBCs]]
- [[AMD GPU SBCs]]
- [[TensorFlow Lite]]
- [[ONNX Runtime]]
- [[Docker]]
- [[ROS2]]
- [[IoT]]
- [[Jetson Nano]]
- [[Industrial PCs]]

---
