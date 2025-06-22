# 🔎 Clustering

**Clustering** is an unsupervised machine learning technique that groups data points into clusters based on similarity. The goal is to ensure that data points within the same cluster are more similar to each other than to those in other clusters. Clustering is widely used in data exploration, pattern recognition, image analysis, and anomaly detection.

---

## 🧠 Summary

- **Type**: Unsupervised learning
- **Goal**: Find natural groupings (clusters) in data
- **Common applications**: Data exploration, customer segmentation, anomaly detection, computer vision

---

## 🛠️ Common Clustering Algorithms

| Algorithm        | Type                  | Shape of Clusters | Strengths                             | Weaknesses                          |
|------------------|-----------------------|------------------|---------------------------------------|--------------------------------------|
| [[K-Means]]       | Centroid-based         | Spherical         | Simple, scalable                     | Struggles with non-spherical clusters |
| DBSCAN           | Density-based          | Arbitrary         | Finds arbitrarily shaped clusters     | Sensitive to parameters, struggles with varying densities |
| Hierarchical     | Tree-based (agglomerative or divisive) | Arbitrary         | Dendrogram provides hierarchy insight | Can be slow on large datasets       |
| Mean Shift       | Density-based          | Arbitrary         | No need to specify # clusters         | Computationally intensive           |
| [[Gaussian Mixture Models (GMM)]] | Probabilistic mixture | Elliptical        | Soft clustering (probabilities)       | Sensitive to initialization         |
| Spectral         | Graph-based            | Arbitrary         | Handles complex cluster shapes        | Computationally expensive for large data |

---

## 🎯 Common Use Cases

- Customer segmentation in marketing
- Document clustering (e.g. news articles)
- Image segmentation
- Grouping similar sensor data (e.g. in robotics)
- Anomaly and outlier detection

---

## ✅ Strengths

- Reveals hidden structure in unlabeled data
- Can handle large, high-dimensional datasets (e.g. K-Means)
- Provides insights for further supervised learning

---

## ❌ Weaknesses

- Results depend on choice of algorithm and hyperparameters
- Some methods (e.g. K-Means) assume specific cluster shapes
- Sensitive to noise and outliers in some cases

---

## 🌐 External References

- [Scikit-learn clustering overview](https://scikit-learn.org/stable/modules/clustering.html)
- [Clustering algorithms comparison (scikit-learn examples)](https://scikit-learn.org/stable/auto_examples/cluster/plot_cluster_comparison.html)

---

## 🔗 Related Notes

- [[KMeans]]
- [[Algorithms]]
- [[Dimensionality Reduction]]
- [[t-SNE]]
- [[UMAP]]
- [[DBSCAN]]
- [[Gaussian Mixture Models (GMM)]]

---
