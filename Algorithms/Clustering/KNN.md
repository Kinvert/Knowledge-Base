# KNN (K Nearest Neighbors)

K Nearest Neighbors (KNN) is a simple, non-parametric, and instance-based learning algorithm used in both classification and regression tasks. It makes predictions by comparing new data points to stored examples and using the "majority vote" (classification) or "average" (regression) from the closest neighbors in feature space.

---

## 🧭 Overview

KNN works by finding the *k* closest data points to a query sample and inferring its output from those neighbors. The distance metric (e.g., Euclidean, Manhattan, cosine similarity) determines which points are "closest."  

It is widely used in robotics, computer vision, anomaly detection, and recommendation systems due to its simplicity and interpretability.

---

## 🧩 Core Concepts

- **Instance-based learning**: Stores training data rather than building an explicit model.  
- **Distance metric**: Choice of metric strongly impacts performance.  
- **k-value selection**: Small *k* → sensitive to noise, large *k* → more generalized.  
- **Decision boundaries**: KNN creates non-linear decision surfaces depending on data distribution.  
- **Weighted neighbors**: Option to weight neighbors by distance (closer points matter more).  

---

## 📊 Comparison Chart

| Algorithm                  | Type              | Parametric | Training Cost | Inference Cost | Robust to Noise | Common Use Cases          |
|-----------------------------|------------------|------------|---------------|----------------|-----------------|---------------------------|
| **KNN**                    | Instance-based   | No         | Low           | High (O(n))    | Moderate        | Classification, regression |
| [[SVM]] (Support Vector Machine) | Model-based      | Yes        | High          | Moderate       | High            | Classification, margins   |
| [[Decision Tree]]           | Model-based      | No         | Moderate      | Fast           | Low (pruning needed) | Classification, regression |
| [[Random Forest]]           | Ensemble         | No         | High          | Moderate       | High            | Classification, regression |
| [[Naive Bayes]]             | Probabilistic    | Yes        | Low           | Fast           | Low (independence assumption) | Text classification |
| [[Logistic Regression]]     | Model-based      | Yes        | Low           | Fast           | Moderate        | Binary classification |

---

## 🛠️ Use Cases

- **Robotics**: Sensor fusion, object recognition, fault detection.  
- **Computer Vision**: Image classification, handwriting recognition.  
- **Recommendation Systems**: Finding similar items/users.  
- **Healthcare**: Disease diagnosis via patient similarity.  
- **Anomaly Detection**: Identifying outliers in sensor data.  

---

## ✅ Strengths

- Easy to understand and implement.  
- No explicit training phase.  
- Works well with non-linear decision boundaries.  
- Naturally handles multi-class problems.  

---

## ❌ Weaknesses

- Computationally expensive at inference time (`O(n)` comparisons).  
- Sensitive to irrelevant features and feature scaling.  
- Performance degrades with high-dimensional data (curse of dimensionality).  
- Requires careful selection of *k* and distance metric.  

---

## 🔧 Variants

- **Weighted KNN**: Neighbors closer to the query point have higher influence.  
- **k-d Tree / Ball Tree KNN**: Efficient search structures to speed up nearest neighbor queries.  
- **Approximate Nearest Neighbors (ANN)**: Faster methods for large-scale datasets.  
- **Fuzzy KNN**: Assigns membership degrees instead of hard labels.  

---

## 🐍 Python / Scikit-learn

Most common library: `scikit-learn` (`sklearn.neighbors.KNeighborsClassifier` / `KNeighborsRegressor`).

Typical workflow:
1. Load data into a `pandas.DataFrame`
2. Split into train/test (`sklearn.model_selection.train_test_split`)
3. Scale features (`sklearn.preprocessing.StandardScaler`)
4. Fit/predict with `KNeighborsClassifier`

Example cheatsheet (not full code here, just quick reference):

- Import: `from sklearn.neighbors import KNeighborsClassifier`
- Init: `model = KNeighborsClassifier(n_neighbors=5, metric='euclidean')`
- Train: `model.fit(X_train, y_train)`
- Predict: `y_pred = model.predict(X_test)`
- Evaluate: `accuracy_score(y_test, y_pred)`

---

## 🖥️ Other Implementations

- **Python**  
  - `scikit-learn` (most common)
  - `scipy.spatial.KDTree` / `BallTree` (fast neighbor search)
  - `faiss` (Facebook, large-scale ANN)
  - `annoy`, `hnswlib` (approximate neighbors)

- **C / C++**  
  - OpenCV: `cv::ml::KNearest`
  - FLANN (Fast Library for Approximate Nearest Neighbors)
  - nanoflann (header-only C++ library for KD-trees)
  - mlpack (C++ machine learning library with KNN support)

- **Other Languages**  
  - R: `class::knn()`
  - MATLAB: `fitcknn()`

---

## 📊 Cheatsheet

- **Preprocessing**: Normalize data → `StandardScaler` or `MinMaxScaler`
- **Classifier**: `KNeighborsClassifier(n_neighbors=k, metric='minkowski', p=2)`
- **Regressor**: `KNeighborsRegressor(n_neighbors=k, weights='distance')`
- **Cross-validation**: `GridSearchCV` for optimal k and distance metric
- **Efficient queries**: Use `algorithm='kd_tree'` or `algorithm='ball_tree'` for large datasets

---

## 📚 Related Concepts

- [[SVM]] (Support Vector Machine)  
- [[Decision Tree]]
- [[Random Forest]]
- [[Naive Bayes]]
- [[Logistic Regression]]
- [[Euclidean Distance]]
- [[Curse of Dimensionality]]
- [[Dimensionality Reduction]]
- [[Clustering]]

---

## 🌐 External Resources

- [Scikit-learn KNN Documentation](https://scikit-learn.org/stable/modules/neighbors.html)  
- [Stanford CS229 Notes on KNN](https://cs229.stanford.edu/)  
- [Approximate Nearest Neighbors Libraries (FAISS, Annoy, HNSW)](https://github.com/facebookresearch/faiss)  

---
