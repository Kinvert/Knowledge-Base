# Semantic Segmentation

Semantic Segmentation is a computer vision task where each pixel in an image is classified into a predefined category. It is a cornerstone technique in robotics, autonomous vehicles, medical imaging, and other domains requiring spatial understanding of visual data. Unlike object detection, semantic segmentation does not distinguish between separate instances of the same class—it simply assigns class labels to every pixel.

---

## 🧠 Core Concepts

- **Pixel-wise Classification**: Assigning a class label to each individual pixel.
- **Class Map**: The result is a dense map with each pixel labeled (e.g. road, person, tree).
- **Fully Convolutional Networks (FCNs)**: Core architecture replacing fully connected layers with convolutions.
- **Encoder-Decoder Structures**: Common in segmentation (e.g. U-Net, SegNet).
- **Upsampling Techniques**: Includes transpose convolutions, bilinear upsampling, and pixel shuffle.
- **Loss Functions**: Often use [[Cross Entropy Loss]], [[Dice Loss]], or [[IoU Loss]] depending on task imbalance and overlap needs.
- **Softmax Activation**: Typically used for multi-class outputs.

---

## ⚙️ Overview

Semantic segmentation enables a robot or system to understand *what* is in the image and *where* it is. By providing fine-grained spatial context, it helps improve navigation, manipulation, and decision-making. It is computationally expensive and requires significant data labeling effort.

---

## 📊 Comparison Chart

| Technique                 | Distinguishes Instances | Output Type     | Common Use Case                   | Complexity | Examples                       |
|---------------------------|--------------------------|------------------|------------------------------------|------------|--------------------------------|
| Semantic Segmentation     | ❌ No                   | Class per pixel | Road scenes, medical imaging      | Moderate   | U-Net, DeepLab                 |
| [[Instance Segmentation]]     | ✅ Yes                  | Mask per object | MS COCO, robotics grasping        | High       | Mask R-CNN                    |
| [[Panoptic Segmentation]]     | ✅ Yes (all classes)     | Unified labeling | AVs, robotics navigation          | Very High  | Panoptic-DeepLab              |
| [[Object Detection]]          | ✅ Yes (bounding boxes)  | BBoxes          | General object detection          | Moderate   | YOLO, Faster R-CNN            |
| [[Image Classification]]      | ❌ No                   | Single label     | Image tagging                     | Low        | ResNet, MobileNet             |

---

## 🧰 Use Cases

- Autonomous navigation (lane detection, drivable area)
- Agricultural robotics (crop vs weed detection)
- Industrial robotics (workspace awareness)
- Medical diagnostics (organ/tumor segmentation)
- Augmented reality (background separation)
- Aerial/satellite imagery interpretation

---

## ✅ Strengths

- Fine-grained pixel-level prediction
- Useful for spatial reasoning and environment modeling
- Compatible with real-time robotics systems (if optimized)

---

## ❌ Weaknesses

- Computationally intensive
- Struggles with overlapping instances of the same class
- Requires large, labeled datasets
- Sensitive to resolution and occlusion

---

## 🛠️ Key Features

- Dense spatial prediction
- Real-time capable with optimization (e.g. TensorRT, ONNX)
- Strong support in modern DL libraries (PyTorch, TensorFlow)
- Works with RGB, depth, LiDAR, or multi-modal input

---

## 🔧 Developer Tools

- `PyTorch` + `torchvision.models.segmentation`
- `TensorFlow/Keras` with custom decoder layers
- `SegFormer`, `DeepLabV3+`, `U-Net` implementations
- `Labelbox`, `CVAT`, or `SuperAnnotate` for labeling
- `ONNX` or `TensorRT` for deployment

---

## 🧠 Related Notes

- [[Instance Segmentation]] (Pixel-level prediction per object)
- [[Dice Loss]] (Overlap-sensitive loss function)
- [[U-Net]] (Common architecture)
- [[DeepLab]] (Atrous convolution-based network)
- [[IoU]] (Intersection over Union)
- [[Robotics Perception]] (Vision in robotic systems)
- [[Mask R-CNN]] (Instance-level segmentation)
- [[Fully Convolutional Network]] (Foundation for semantic segmentation)

---

## 🌐 External Resources

- https://paperswithcode.com/task/semantic-segmentation
- https://github.com/qubvel/segmentation_models.pytorch
- https://arxiv.org/abs/1505.04597 (Fully Convolutional Networks)
- https://arxiv.org/abs/1802.02611 (DeepLabV3+)
