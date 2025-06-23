# 🌌 UMAP (Uniform Manifold Approximation and Projection)

**UMAP** is a non-linear dimensionality reduction technique designed to embed high-dimensional data into low-dimensional space (typically 2D or 3D) while preserving as much of the underlying structure as possible. It is widely used for visualization, clustering, and exploratory data analysis.

---

## 🧠 Summary

- Developed as an alternative to t-SNE, with faster performance and better preservation of global data structure.
- Based on manifold learning and topological data analysis.
- Suitable for large datasets and high-dimensional data.

---

## ⚙️ How It Works (High-Level)

- Constructs a weighted k-nearest neighbor (k-NN) graph of the data.
- Estimates the local manifold structure in high-dimensional space.
- Optimizes a low-dimensional embedding that preserves the high-dimensional relationships.

---

## 🚀 Applications

- Visualizing clusters in high-dimensional datasets (e.g., image features, gene expression).
- Preprocessing for machine learning tasks.
- Understanding structure in word embeddings or document representations.
- Reducing noise while preserving essential data patterns.

---

## 🏆 Strengths

- Preserves both local and global structure better than t-SNE in many cases.
- Scales well to large datasets.
- Faster than t-SNE on most tasks.
- Can serve as a preprocessor for clustering.

---

## ⚠️ Weaknesses

- Still non-deterministic unless a random seed is set.
- Choice of hyperparameters (e.g., n_neighbors, min_dist) can greatly affect results.
- May overemphasize certain structures if parameters are poorly tuned.

---

## 🔗 Related Notes

- [[Dimensionality Reduction]]
- [[t-SNE]]
- [[PCA]]
- [[Clustering]]
- [[K-Means]]
- [[DBSCAN]]

---

## 🌐 External References

- [UMAP GitHub](https://github.com/lmcinnes/umap)
- [UMAP Documentation](https://umap-learn.readthedocs.io/en/latest/)

---

## 📊 Comparison with Similar Methods

| Method     | Preserves Local Structure | Preserves Global Structure | Speed | Typical Use |
|------------|--------------------------|---------------------------|-------|-------------|
| UMAP       | ✅                         | ✅ (better than t-SNE)      | Fast  | Large data visualization |
| t-SNE      | ✅                         | ❌                          | Slow  | Small to medium data visualization |
| PCA        | ❌                         | ✅                          | Very fast | Linear structure, variance analysis |

---
