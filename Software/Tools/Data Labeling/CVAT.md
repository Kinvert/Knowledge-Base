# 🖼️ CVAT (Computer Vision Annotation Tool)

CVAT is an open-source annotation tool for labeling data used in computer vision applications. It was originally developed by Intel and is now maintained by the open-source community under the Apache 2.0 license.

---

## 🔎 Summary

- **Full Name**: Computer Vision Annotation Tool
- **Type**: Open Source Data Annotation Platform
- **License**: Apache 2.0
- **Main Use Case**: Labeling images, videos, and 3D data for supervised machine learning models.

CVAT is one of the most powerful self-hosted data labeling tools, particularly well-suited for computer vision tasks. It supports a wide variety of annotation types, integrates well with machine learning workflows, and has become a de facto standard in many industries and research projects.

---

## ⚙️ Supported Annotation Types

| Annotation Type | Description |
|------------------|-------------|
| [[Image Classification]] | Assign single or multiple class labels to entire images. |
| [[Object Detection]] | Draw bounding boxes around objects. |
| [[Semantic Segmentation]] | Pixel-wise class labels. |
| Instance Segmentation | Pixel-wise labels for each object instance. |
| Polygon Annotation | Freehand or point-to-point region marking. |
| Polyline Annotation | Useful for lanes, boundaries, etc. |
| Keypoint Annotation | Landmark detection (e.g., facial landmarks, pose estimation). |
| 3D Point Cloud Annotation | Annotate LIDAR or depth sensor data. |
| Video Annotation | Supports frame-by-frame object tracking. |

---

## 🚀 Key Features

- Web-based interface
- Team collaboration and user management
- API and automation integration (REST API)
- Supports semi-automatic annotation tools
- Pre-annotation with deep learning models
- Plugin system for custom extensions
- Supports large datasets
- Advanced interpolation and tracking for video annotation
- Multi-platform support via Docker containers

---

## 🖥️ Deployment Options

| Deployment Type | Details |
|------------------|---------|
| Self-Hosted | Install locally or on a server via Docker. |
| Cloud-Hosted | Host on any cloud provider (AWS, Azure, GCP, etc.). |
| Kubernetes | Supports container orchestration. |

---

## 🌐 Common Use Cases

| Industry | Use Case |
|----------|----------|
| Autonomous Vehicles | Object detection, segmentation, LIDAR annotation |
| Healthcare | Medical image labeling |
| Agriculture | Plant disease detection, crop monitoring |
| Industrial | Defect detection, visual inspection |
| Retail | Product recognition, inventory monitoring |
| Robotics | SLAM, object manipulation |

---

## 🔧 Strengths & Weaknesses

### Strengths

- Fully open source, actively maintained
- Supports nearly all major annotation types
- Highly customizable
- Scales well for teams and projects
- Integrates into ML pipelines
- REST API support for automation

### Weaknesses

- Setup requires some technical knowledge
- User interface can feel complex for beginners
- 3D annotation still maturing

---

## 🔄 Supported Formats

- Pascal VOC
- COCO
- YOLO
- LabelMe
- PASCAL VOC
- TFRecord
- MOT
- CVAT XML (native format)
- Others via plugins and converters

---

## 🔗 Related Tools & Comparisons

| Tool | License | Strength | Weakness |
|------|---------|----------|----------|
| [[CVAT]] | Apache 2.0 | Comprehensive, self-hosted | Setup complexity |
| [[Label Studio]] | Apache 2.0 | Multi-modal (text, audio, image) | Complex setups for big projects |
| [[Supervisely]] | Freemium | 3D support, team features | Cost for full features |
| [[Labelbox]] | Commercial | Enterprise tools | Subscription pricing |
| [[Diffgram]] | Open Source | Full ML workflow | Less mature than CVAT |

---

## 📊 Supported Protocols and Integrations

- REST API for integration with ML pipelines
- Webhooks for automation
- Integrations with:
  - TensorFlow
  - PyTorch
  - Detectron2
  - MMDetection
  - Custom models

---

## 🧪 Standards & Protocols Used

| Standard/Protocol | Purpose |
|--------------------|---------|
| HTTP/HTTPS | Web UI and API communication |
| REST API | Automation & integration |
| OAuth2 | User authentication (optional) |
| Docker Compose | Deployment |

---

## 📚 References

- [CVAT GitHub Repository](https://github.com/opencv/cvat)
- [Official Documentation](https://opencv.github.io/cvat/)
- [Docker Deployment Guide](https://opencv.github.io/cvat/docs/administration/basics/installation/)

---

## 🔗 Internal Links

- See also: [[Data Labeling]], [[Label Studio]], [[Supervisely]], [[AI Training Pipelines]], [[Computer Vision]]
