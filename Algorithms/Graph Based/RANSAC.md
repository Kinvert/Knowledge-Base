# RANSAC (Random Sample Consensus)

RANSAC is a robust statistical method used to estimate parameters of a mathematical model from a dataset that contains outliers. It is especially prevalent in computer vision and robotics, where sensor noise and mismatches in feature correspondences are common. RANSAC excels at identifying the best model that fits the inliers while discarding outliers.

---

## 🧠 Overview

RANSAC is an iterative method designed to handle a high percentage of outliers in the input data. It works by repeatedly selecting a random subset of the data, fitting a model to that subset, and determining how many elements in the entire dataset fit the model within a predefined tolerance.

---

## ⚙️ How It Works

1. Randomly select a minimal subset of the data needed to estimate the model parameters.
2. Fit the model to this subset.
3. Determine the number of inliers — points that fit the model well within a tolerance.
4. Repeat steps 1–3 for a fixed number of iterations.
5. Choose the model with the largest number of inliers.
6. Optionally, re-estimate the model using all inliers from the best model.

---

## 🧩 Core Concepts

- **Inliers vs Outliers**: RANSAC separates data into inliers (fit the model) and outliers (do not).
- **Consensus Set**: A subset of data that agrees with a particular model.
- **Robust Estimation**: The key strength of RANSAC is its ability to tolerate outliers.

---

## 🧪 Use Cases

- Feature matching in stereo vision
- Estimating homography or fundamental matrix in image processing
- Plane fitting in 3D point clouds (e.g., LiDAR)
- Motion model estimation in SLAM
- Fitting lines or circles in noisy sensor data

---

## ✅ Strengths

- Extremely robust to outliers
- Works well in high-noise environments
- Easy to implement and adapt to different models

---

## ❌ Weaknesses

- No guarantee of optimal result
- Computationally expensive with large datasets
- Requires tuning of thresholds and iteration count
- May fail if the fraction of inliers is too small

---

## 📊 Comparison Chart

| Method     | Robust to Outliers | Uses Prior Probabilities | Adaptive Sampling | Common in SLAM | Notes |
|------------|--------------------|---------------------------|-------------------|----------------|-------|
| **RANSAC** | ✅ High             | ❌ No                     | ❌ No             | ✅ Yes         | Simple, generic |
| **PROSAC** | ✅ High             | ✅ Yes (ranking)         | ✅ Yes            | ✅ Yes         | Prioritized sampling |
| **MLESAC** | ✅ High             | ✅ Yes                   | ❌ No             | ✅ Yes         | Maximizes likelihood |
| **LMedS**  | ✅ High             | ❌ No                    | ❌ No             | ❌ Rare        | Minimizes median of residuals |
| **LO-RANSAC** | ✅ Very High     | ❌ No                    | ✅ Local Opt.     | ✅ Yes         | Includes local optimization |

---

## 🛠 Compatible Items

- [[OpenCV]] (Computer vision library with built-in RANSAC functions)
- [[PCL]] (Point Cloud Library with RANSAC-based model fitting)
- [[Eigen]] (Used for matrix math in model fitting)
- [[Sophus]] (Pose representation in SE(3), often used with RANSAC in SLAM)
- [[g2o]] (Used in optimization after model estimation)

---

## 📚 Related Concepts

- [[SLAM]] (Simultaneous Localization and Mapping)
- [[ICP]] (Iterative Closest Point)
- [[Feature Matching]] (Keypoint-based matching often filtered by RANSAC)
- [[Homography]] (Model often estimated using RANSAC in vision pipelines)
- [[Point Cloud Segmentation]] (RANSAC used to identify planes, cylinders, etc.)
- [[Model Fitting]] (General category of techniques to which RANSAC belongs)

---

## 🔧 Developer Tools

- `cv2.findHomography(..., method=cv2.RANSAC)` in OpenCV
- `pcl::SampleConsensusModelPlane` in PCL
- MATLAB `ransac` function (if using toolboxes)

---

## 🔎 Further Reading

- Fischler, M. A., & Bolles, R. C. (1981). *Random Sample Consensus: A Paradigm for Model Fitting with Applications to Image Analysis and Automated Cartography.*
- Hartley & Zisserman. *Multiple View Geometry in Computer Vision*
- PCL RANSAC tutorials and examples

---
