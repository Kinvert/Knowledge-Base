---
title: eCALREC (eCAL Recording Format)
tags: [data-storage, file-format, ecal, ecalrec, hdf5, distributed-systems]
aliases: [eCALREC File Format, eCAL Recording, eCAL HDF5]
---

# 📂 eCALREC: eCAL Recording Format

## 🧭 Overview

**eCALREC** is the recording format used by **eCAL (Enhanced Communication Abstraction Layer)**, a middleware framework designed for high-performance data exchange in distributed systems. The `.ecalrec` file acts as a metadata file that organizes and references multiple **HDF5** files, which store the actual recorded data streams. This structure allows for efficient storage, replay, and analysis of large-scale, time-synchronized data.

The `.ecalrec` file is essential for managing distributed recordings, as it provides a centralized way to describe and access the individual HDF5 files that contain the recorded data.

---

## 🛠️ Key Features

1. **Centralized Metadata**:
   - The `.ecalrec` file serves as an index for all HDF5 files in a recording session, storing metadata about the recording.

2. **Time-Synchronized Data**:
   - Ensures all recorded data streams are synchronized with precise timestamps.

3. **Distributed Recording**:
   - Supports recording data from multiple nodes in a distributed system.

4. **Efficient Storage**:
   - Uses HDF5 files for compact and high-performance data storage.

5. **Replay Capability**:
   - Enables replaying recorded data streams for debugging, analysis, and simulation.

6. **Cross-Platform Compatibility**:
   - Works seamlessly on Windows and Linux systems.

---

## 📦 How eCALREC Works

1. **Recording Process**:
   - During a recording session, eCAL captures data streams (e.g., sensor data, telemetry) and stores them in multiple HDF5 files.
   - The `.ecalrec` file is created to act as a central metadata file that references these HDF5 files.

2. **File Structure**:
   - **`.ecalrec` File**: Contains metadata about the recording session, such as timestamps, topic names, and references to the associated HDF5 files.
   - **HDF5 Files**: Store the actual data streams, such as video, LiDAR, or telemetry data, in a compact and efficient format.

3. **Replay and Analysis**:
   - The `.ecalrec` file is used to load and replay the recorded data streams in eCAL Player.
   - Developers can analyze the data or export it to other formats for further processing.

---

## 📦 Common Use Cases

1. **Autonomous Vehicle Development**:
   - Recording sensor data (e.g., cameras, LiDAR, radar) during test drives.
   - Replaying data for debugging and improving perception algorithms.

2. **Robotics**:
   - Logging telemetry and sensor data for robot testing and validation.
   - Simulating real-world scenarios using recorded data.

3. **Industrial Automation**:
   - Capturing data from distributed systems for diagnostics and optimization.
   - Analyzing time-synchronized data from multiple machines.

4. **Data Analysis and Machine Learning**:
   - Storing large datasets for training and testing machine learning models.
   - Examples include object detection, path planning, and sensor fusion.

---

## ✅ Pros and ❌ Cons

### ✅ Advantages
- **Centralized Metadata**: Simplifies management of distributed recordings.
- **Time Synchronization**: Ensures precise alignment of data streams for accurate analysis.
- **High Performance**: Uses HDF5 for efficient storage and retrieval of large datasets.
- **Replay Functionality**: Enables debugging and simulation by replaying recorded data.

### ❌ Disadvantages
- **Proprietary Format**: Requires eCAL tools to access and manage recordings.
- **Learning Curve**: Requires familiarity with the eCAL ecosystem.
- **Limited Ecosystem**: Not as widely supported as open formats like ROS bag files.

---

## 🛠️ Tools for Working with eCALREC Files

### **1. eCAL Recorder**
- **Description**: Official tool for creating `.ecalrec` files and managing recordings.
- **Features**:
  - Start, stop, and monitor distributed recordings.
  - Manage multiple data streams across nodes.

### **2. eCAL Player**
- **Description**: Tool for replaying `.ecalrec` files and their associated HDF5 data.
- **Features**:
  - Replay data streams with precise time synchronization.
  - Adjust playback speed for debugging and analysis.

### **3. Export Tools**
- **Description**: Tools for converting `.ecalrec` recordings to other formats (e.g., CSV, ROS bag).
- **Features**:
  - Enable interoperability with other ecosystems.
  - Simplify data analysis workflows.

---

## ✅ eCALREC vs Other Formats

| Feature                | eCALREC           | ROS Bag           | CSV               | HDF5              |
|------------------------|-------------------|-------------------|-------------------|-------------------|
| **Time Synchronization** | ✅ Yes           | ✅ Yes           | ❌ No             | ✅ Yes           |
| **Performance**        | High              | Moderate          | Low               | High              |
| **Compression**        | ✅ Yes           | ✅ Yes           | ❌ No             | ✅ Yes           |
| **Human Readability**  | ❌ No             | ❌ No             | ✅ Yes            | ❌ No            |
| **Integration**        | eCAL Ecosystem    | ROS Ecosystem     | General Use       | General Use       |
| **Use Cases**          | Automotive, Robotics | Robotics         | Simple Data       | Scientific Data   |

---

## 🔗 Related Topics

- [[eCAL]] (Enhanced Communication Abstraction Layer)
- [[HDF5]] (Hierarchical Data Format)
- [[Data Storage Formats]]
- [[Automotive Data Logging]]

---

## 📚 Further Reading

- [eCAL Official Documentation](https://continental.github.io/ecal/)
- [eCAL GitHub Repository](https://github.com/continental/ecal)
- [HDF5 File Format](https://www.hdfgroup.org/solutions/hdf5/)
- [eCAL Recorder and Player](https://continental.github.io/ecal/tools/recorder/)

---

## 🧠 Summary

The `.ecalrec` file is a central component of eCAL's recording system, providing metadata and references to multiple HDF5 files that store the actual data streams. This structure enables efficient storage, replay, and analysis of large-scale, time-synchronized data, making it a powerful tool for applications like autonomous vehicle development, robotics, and industrial automation. While it is a proprietary format, its integration with the eCAL ecosystem makes it invaluable for managing distributed recordings.
