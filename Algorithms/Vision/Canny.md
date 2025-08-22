# Canny Edge Detection

The **Canny Edge Detection** algorithm is a multi-stage method in computer vision designed to detect edges in images with high accuracy and minimal false detections. It was introduced by John F. Canny in 1986 and is widely used in robotics, image processing, and computer vision systems due to its balance of performance, efficiency, and robustness.

---

## 🔎 Overview

Canny is a **feature detection algorithm** specifically for identifying edges in images. It uses gradient-based techniques combined with filtering and non-maximum suppression to achieve thin, accurate edges. Its popularity stems from its ability to reduce noise while still preserving critical structural information in an image.

---

## 🧠 Core Concepts

- **Noise Reduction**: Uses a Gaussian filter to smooth the image and reduce noise sensitivity.
- **Gradient Calculation**: Finds the intensity gradient of the image to identify areas with strong intensity changes.
- **Non-Maximum Suppression**: Thins edges by removing pixels that are not local maxima along the gradient direction.
- **Double Thresholding**: Applies two thresholds (high and low) to classify strong and weak edges.
- **Edge Tracking by Hysteresis**: Links weak edges to strong ones if they are connected, improving continuity.

---

## 📊 Comparison Chart

| Algorithm              | Key Strength                | Noise Sensitivity | Typical Use Case                           |
|------------------------|-----------------------------|-------------------|--------------------------------------------|
| **Canny**              | Accurate, thin edges        | Low (Gaussian)    | General-purpose edge detection              |
| [[Sobel]]              | Simple, fast                | High              | Gradient estimation, basic edge detection   |
| [[Prewitt]]            | Computationally cheap       | High              | Edge detection with less precision          |
| [[Laplacian of Gaussian]] (LoG) | Detects edges & corners     | Medium            | Edge & blob detection                       |
| [[Roberts Cross]]      | Very simple, small kernels  | High              | Fast edge detection, low accuracy           |
| [[Scharr]]             | Improved gradient accuracy  | Medium            | Image gradient estimation in CV tasks       |
| [[Harris Corner Detector]] | Feature localization       | N/A               | Corner detection, feature tracking          |

---

## 🛠️ Use Cases

- Object detection and recognition
- Lane detection in autonomous vehicles
- SLAM (Simultaneous Localization and Mapping)
- Medical imaging (e.g., detecting structures in X-rays, MRI)
- Industrial robotics for defect detection
- Image preprocessing for feature extraction

---

## ✅ Strengths

- Produces **thin, well-localized edges**
- Reduces false positives via **non-maximum suppression**
- **Noise-resistant** due to Gaussian smoothing
- Parameterized with thresholds for flexibility

---

## ❌ Weaknesses

- Computationally heavier than simpler detectors (e.g., Sobel, Prewitt)
- Sensitive to threshold selection (requires tuning)
- Gaussian smoothing can remove fine details

---

## 🔗 Compatible Items

- Works with most image formats (grayscale preferred)
- Commonly implemented in [[OpenCV]] and [[scikit-image]]
- Runs efficiently on modern GPUs and [[CUDA]] environments

---

## 🧪 Variants

- **Canny-Deriche**: Recursive filtering for speed
- **Adaptive Canny**: Automatic threshold calculation
- **Multi-Scale Canny**: Uses different Gaussian kernel sizes for edge detection at multiple resolutions

---

## 📚 External Resources

- Original paper: "A Computational Approach to Edge Detection" by John F. Canny (1986)
- OpenCV documentation: https://docs.opencv.org/4.x/da/d22/tutorial_py_canny.html
- scikit-image documentation: https://scikit-image.org/docs/stable/auto_examples/edges/plot_canny.html

---

## 📝 Summary

Canny is one of the most reliable and widely used edge detection algorithms in computer vision. While more computationally expensive than simpler methods, its accuracy and noise robustness make it the go-to choice in robotics, medical imaging, and industrial inspection. 

---

## 🧩 Related Concepts

- [[Feature Detection]] (broad category of techniques)
- [[Sobel]] (Gradient operator)
- [[Prewitt]] (Gradient operator)
- [[Roberts Cross]] (Simple edge operator)
- [[Hough Transform]] (Line detection post-edge detection)
- [[Gaussian Blur]] (used for noise reduction step)
- [[OpenCV]] (Library implementation)
- [[SLAM]] (Edges often used in mapping and localization)
