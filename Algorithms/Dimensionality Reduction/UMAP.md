# 🌐 UMAP (Uniform Manifold Approximation and Projection)

**UMAP** is a non-linear dimensionality reduction technique designed to project high-dimensional data into lower dimensions (typically 2D or 3D) while preserving both local and some global structure. It is widely used for visualization, clustering, and as a faster alternative to [[t-SNE]].

---

## 🧠 Summary

- **Full name**: Uniform Manifold Approximation and Projection
- **Type**: Non-linear dimensionality reduction
- **Invented by**: Leland McInnes, John Healy, James Melville (2018)
- **Purpose**: Visualize and analyze high-dimensional data

---

## 🎯 Key Features

- Preserves more global structure than [[t-SNE]]
- Much faster than [[t-SNE]] on large datasets
- Supports supervised and unsupervised dimensionality reduction
- Can be used as a general-purpose embedding for machine learning

---

## 📊 Comparison to Other Methods

| Algorithm      | Type      | Preserves Local Structure | Preserves Global Structure | Speed | Typical Use                      |
|----------------|-----------|--------------------------|---------------------------|--------|-----------------------------------|
| UMAP           | Non-linear | ✅ Yes                     | ⚠️ Some                    | 🚀 Fast | Visualization, clustering        |
| [[t-SNE]]      | Non-linear | ✅ Yes                     | ❌ No                       | 🐢 Slow | Cluster visualization            |
| [[PCA]]        | Linear     | ⚠️ Limited                 | ✅ Yes                      | 🚀 Fast | Variance capture, preprocessing  |
| [[Isomap]]     | Non-linear | ✅ Yes                     | ✅ Yes                      | ⚠️ Moderate | Manifold learning               |

---

## 🔬 Common Use Cases

- Visualizing latent spaces of neural networks
- Large-scale clustering exploration
- Preprocessing for clustering algorithms
- Data embedding for downstream tasks

---

## ✅ Strengths

- Scales well to large datasets
- Produces more consistent embeddings than [[t-SNE]]
- Maintains both local detail and some global structure
- Easy to integrate with Python ML libraries (e.g. scikit-learn API)

---

## ❌ Weaknesses

- Still non-deterministic unless a seed is fixed
- Global structure preservation is better than t-SNE but imperfect
- Hyperparameter tuning (e.g. `n_neighbors`, `min_dist`) can significantly affect results

---

## 🌐 External References

- [Official UMAP documentation](https://umap-learn.readthedocs.io/en/latest/)
- [Original UMAP paper](https://arxiv.org/abs/1802.03426)

---

## 🔗 Related Notes

- [[Dimensionality Reduction]]
- [[t-SNE]]
- [[PCA]]
- [[Isomap]]
- [[Cluster Analysis]]

---
