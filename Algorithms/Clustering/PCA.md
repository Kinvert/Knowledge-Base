# 📈 PCA (Principal Component Analysis)

**Principal Component Analysis (PCA)** is a linear dimensionality reduction technique that transforms high-dimensional data into a lower-dimensional form by projecting it onto the directions (principal components) of maximum variance. It is one of the most widely used methods for data compression, visualization, and noise reduction.

---

## 🧠 Summary

- PCA finds orthogonal axes (principal components) that capture the most variance in the data.
- The first principal component captures the highest variance; subsequent components capture decreasing amounts of variance.
- It's a linear method — suitable when the data lies roughly in a linear subspace.

---

## ⚙️ How It Works (High-Level)

1. Center the data (subtract the mean).
2. Compute the covariance matrix.
3. Perform eigenvalue decomposition (or singular value decomposition).
4. Project data onto the top principal components (eigenvectors with largest eigenvalues).

---

## 🚀 Applications

- Data visualization (reducing to 2D/3D)
- Noise filtering and compression
- Preprocessing for machine learning
- Gene expression analysis
- Finance (e.g., risk factor modeling)

---

## 🏆 Strengths

- Fast and efficient for large, sparse datasets.
- Captures variance and simplifies datasets.
- Provides orthogonal, uncorrelated components.

---

## ⚠️ Weaknesses

- Assumes linear relationships; not effective for non-linear manifolds.
- Sensitive to scaling; variables should be standardized.
- Principal components may lack clear interpretability.

---

## 🔗 Related Notes

- [[Dimensionality Reduction]]
- [[UMAP]]
- [[t-SNE]]
- [[Clustering]]

---

## 🌐 External References

- [Wikipedia - PCA](https://en.wikipedia.org/wiki/Principal_component_analysis)
- [scikit-learn PCA Documentation](https://scikit-learn.org/stable/modules/generated/sklearn.decomposition.PCA.html)

---

## 📊 Comparison with Similar Methods

| Method     | Preserves Local Structure | Preserves Global Structure | Speed       | Typical Use |
|------------|--------------------------|---------------------------|-------------|-------------|
| PCA        | ❌                         | ✅                          | Very fast   | Linear structure, variance analysis |
| UMAP       | ✅                         | ✅ (better than t-SNE)      | Fast        | Large data visualization |
| t-SNE      | ✅                         | ❌                          | Slower      | Small to medium data visualization |

---
