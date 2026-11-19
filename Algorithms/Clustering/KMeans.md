# 📌 K-Means Clustering

**K-Means** is one of the most widely used unsupervised machine learning algorithms for partitioning data into **K distinct clusters**. It works by iteratively assigning points to clusters and updating cluster centroids to minimize within-cluster variance (sum of squared distances to the centroid).

---

## 🧠 Summary

- **Type**: Partitional clustering algorithm
- **Goal**: Divide data into K clusters minimizing intra-cluster distances
- **Distance Metric**: Typically Euclidean distance (but can be modified)

---

## ⚙️ How It Works

1️⃣ **Initialization**: Choose K initial centroids (randomly or using heuristics like k-means++).

2️⃣ **Assignment Step**: Assign each data point to the nearest centroid (creating Voronoi-like partitions).

3️⃣ **Update Step**: Recalculate centroids as the mean of assigned points.

4️⃣ **Repeat**: Continue assignment and update steps until convergence (centroids stabilize or max iterations reached).

---

## 🎯 Applications

- Image segmentation (e.g., color quantization)
- Customer segmentation in marketing
- Document clustering (text mining)
- Sensor fusion (grouping detections)
- [[Point Cloud]] downsampling or segmentation
- Pattern recognition and compression

---

## 🛠️ Common Variants

| Variant          | Description |
|------------------|-------------|
| **k-means++**    | Smarter centroid initialization to improve convergence |
| **MiniBatchKMeans** | Processes data in small batches for scalability |
| **[[e-means]]** | EMA mini-batches, scheduled steps, cluster revival and automatic batch/step scaling laws |
| **Fuzzy c-means** | Allows soft clustering (points can belong to multiple clusters) |

---

## 📊 Comparison Table

| Feature                | K-Means             | DBSCAN               | Agglomerative Clustering | [[Voronoi Clustering]] |
|------------------------|--------------------|----------------------|-------------------------|-----------------------|
| Requires number of clusters | ✅ Yes            | ❌ No                 | ❌ No                    | ✅ Yes (via seeds)     |
| Handles arbitrary shapes | ❌ No              | ✅ Yes                | ✅ Yes                   | ❌ No                  |
| Density-aware          | ❌ No               | ✅ Yes                | ⚠️ Depends on linkage    | ❌ No                  |
| Robust to outliers      | ❌ No               | ✅ Yes                | ⚠️ Medium                | ❌ No                  |
| Scalability             | ✅ High             | ⚠️ Medium             | ❌ Low                   | ⚠️ Medium              |

---

## ✅ Strengths

- Simple and fast, especially for large datasets
- Easy to implement and understand
- Works well when clusters are spherical and evenly sized
- Scales well with dimensionality and data size

---

## ❌ Weaknesses

- Requires choosing K (number of clusters)
- Sensitive to initialization (can converge to local minima)
- Poor performance on non-convex or varying-size clusters
- Struggles with outliers
- Assumes equal variance in all directions

---

## 🛠️ Libraries / Tools

- **scikit-learn** (`sklearn.cluster.KMeans`)
- **OpenCV**
- **MATLAB**
- **TensorFlow / PyTorch** (for custom implementations)

---

## 🔗 Related Topics

- [[Voronoi Clustering]]
- [[DBSCAN]]
- [[Agglomerative Clustering]]
- [[Point Cloud]]
- [[OpenCV]]

---

## 🌐 External References

- [scikit-learn: KMeans](https://scikit-learn.org/stable/modules/generated/sklearn.cluster.KMeans.html)
- [Wikipedia: K-means clustering](https://en.wikipedia.org/wiki/K-means_clustering)

---
