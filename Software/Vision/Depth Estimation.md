# 🧠 Depth Estimation

**Depth Estimation** is a fundamental task in computer vision that involves determining the distance of objects from a viewpoint, typically from images or video streams. It's crucial for 3D scene understanding in applications such as robotics, autonomous vehicles, augmented reality, and medical imaging.

---

## 🧰 What Is Depth Estimation?

Depth estimation refers to calculating a per-pixel depth map — an image where each pixel value corresponds to the distance between the camera and the object in the scene. This can be achieved via various techniques ranging from stereo vision to deep learning.

---

## 🎯 Use Cases

- [[SLAM]] and navigation for robotics
- 3D reconstruction
- Object tracking and recognition
- Augmented and mixed reality (AR/MR)
- Driver assistance systems
- Gesture recognition
- Medical imaging (e.g. endoscopy)

---

## 📚 Methods of Depth Estimation

### 🔹 Passive Methods

| Method                 | Description |
|------------------------|-------------|
| **Monocular**          | Estimates depth from a single image using learned priors or geometry |
| **Stereo Vision**      | Uses two or more cameras to triangulate distance by disparity |
| **Structure from Motion (SfM)** | Uses multiple images from different viewpoints to recover 3D structure |
| **Depth from Defocus** | Infers depth based on image blur across focus changes |

### 🔹 Active Methods

| Method                 | Description |
|------------------------|-------------|
| **Time-of-Flight (ToF)** | Measures the time light takes to reflect from objects |
| **Structured Light**     | Projects patterns and analyzes their deformation to infer depth |
| **LiDAR**                | Emits laser pulses to measure distances directly |

---

## 🔬 Monocular vs Binocular Depth Estimation

| Feature                   | Monocular                         | Binocular                          |
|---------------------------|-----------------------------------|------------------------------------|
| Input                     | Single image                      | Stereo image pair                  |
| Depth Accuracy            | Lower, relies on priors           | Higher, geometric triangulation    |
| Complexity                | Lower hardware, higher computation| Additional camera calibration       |
| Suitable For              | Mobile AR, image processing       | Robotics, driver assistance        |

See also: [[Monocular SLAM]], [[Binocular SLAM]]

---

## ⚙️ Algorithms and Frameworks

| Name                  | Category        | Notes |
|-----------------------|-----------------|-------|
| **MiDaS**             | Deep Learning   | High-quality monocular depth estimation |
| **DPT (Dense Prediction Transformer)** | Deep Learning | Monocular depth using transformers |
| **Semi-Global Matching (SGM)** | Stereo Matching | Common in stereo pipelines like OpenCV |
| **COLMAP**            | SfM / MVS       | Photogrammetry pipeline |
| **OpenCV**            | Stereo Block Matching, SGBM | Classical stereo algorithms |
| **PyTorch3D**         | 3D estimation   | Supports differentiable rendering and depth |

---

## 🛠️ Tools & Libraries

- [[OpenCV]]
- [[PCL]] (Point Cloud Library)
- [[Open3D]]
- [[ROS2]] (Robot Operating System)
- [MiDaS](https://github.com/intel-isl/MiDaS)
- [COLMAP](https://colmap.github.io/)
- [TartanAir](https://theairlab.org/tartanair-dataset/) (for datasets)

---

## 🔄 Output Format

- **Depth Map**: 2D grayscale image with distance values
- **Point Cloud**: Set of 3D coordinates derived from depth
- **Mesh**: 3D triangulated surface generated from depth data

---

## 📊 Comparison Table

| Method               | Hardware Needed    | Accuracy     | Suitable For         | Real-Time? |
|----------------------|--------------------|--------------|----------------------|------------|
| Monocular (MiDaS)    | Camera only        | Medium       | AR, offline analysis | ✅ With GPU |
| Stereo (SGM)         | Two cameras        | High         | Robotics, ADAS       | ✅          |
| LiDAR                | LiDAR sensor       | Very High    | ADAS, mapping        | ✅          |
| Structured Light     | IR projector + cam | High         | Face ID, industrial  | ⚠️ Depends |
| Time-of-Flight       | Specialized sensor | Medium–High  | Mobile devices       | ✅          |

---

## ✅ Strengths

- Enables 3D understanding from 2D images
- Crucial for autonomous navigation and perception
- Can be done with or without active sensors
- Wide variety of approaches

---

## ❌ Limitations

- Monocular estimation is less accurate and ambiguous
- Stereo requires calibration and good lighting
- Active methods need specialized (and sometimes expensive) hardware
- Performance varies by scene texture and motion

---

## 🔗 Related Notes

- [[Monocular SLAM]]
- [[Binocular SLAM]]
- [[OpenCV]]
- [[PCL]]
- [[Open3D]]
- [[Point Cloud]]
- [[Camera Calibration]]
- [[ROS2]]
- [[SLAM]]

---
