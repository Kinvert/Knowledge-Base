# 📉 Dimensionality Reduction

**Dimensionality reduction** refers to techniques that transform high-dimensional data into a lower-dimensional form, while preserving as much important structure (e.g., variance, distance, neighborhood relationships) as possible. It is widely used in machine learning, data visualization, and data preprocessing.

---

## 🧠 Summary

- **Purpose**: Simplify data, reduce noise, improve computational efficiency, aid visualization
- **Types**: Linear and non-linear methods
- **Key applications**: Visualization, preprocessing, compression, noise reduction

---

## 🎯 Why Use Dimensionality Reduction?

- Easier to visualize data (e.g. 2D, 3D plots)
- Reduce computational cost and storage
- Remove multicollinearity and noise
- Improve model generalization
- Extract latent features

---

## 🛠️ Common Methods

| Algorithm      | Type      | Preserves Local Structure | Preserves Global Structure | Typical Use                      |
|----------------|-----------|--------------------------|---------------------------|-----------------------------------|
| [[PCA]]        | Linear     | ⚠️ Limited                 | ✅ Yes                      | Preprocessing, variance analysis |
| [[t-SNE]]      | Non-linear | ✅ Yes                     | ❌ No                       | Visualization of clusters        |
| [[UMAP]]       | Non-linear | ✅ Yes                     | ⚠️ Some                    | Faster t-SNE alternative         |
| [[Isomap]]     | Non-linear | ✅ Yes                     | ✅ Yes                      | Manifold learning                |
| Autoencoders   | Non-linear | ✅ Yes                     | ⚠️ Depends on architecture | Deep learning-based reduction    |

---

## ✅ Strengths

- Reveals hidden structure in high-dimensional data
- Helps prevent overfitting by removing redundant features
- Can dramatically reduce memory and compute needs

---

## ❌ Weaknesses

- Risk of losing important information
- Some methods (e.g. [[t-SNE]]) do not preserve global geometry
- Results can be hard to interpret

---

## 🔬 Common Use Cases

- Data visualization (e.g. cluster exploration)
- Feature extraction for machine learning models
- Noise reduction in sensor data
- Compression of large datasets
- Preprocessing step in pipelines

---

## 🌐 External References

- [Dimensionality reduction overview - scikit-learn](https://scikit-learn.org/stable/modules/manifold.html)
- [PCA explained](https://towardsdatascience.com/a-one-stop-shop-for-principal-component-analysis-5582fb7e0a9c)
- [UMAP documentation](https://umap-learn.readthedocs.io/en/latest/)

---

## 🔗 Related Notes

- [[PCA]]
- [[t-SNE]]
- [[UMAP]]
- [[Isomap]]
- [[Algorithms]]
- [[Autoencoder]]
- [[Cluster Analysis]]
- [[Feature Engineering]]

---
