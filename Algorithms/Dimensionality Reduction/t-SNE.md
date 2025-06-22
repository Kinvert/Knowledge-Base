# 🔍 t-SNE (t-distributed Stochastic Neighbor Embedding)

**t-SNE** is a dimensionality reduction algorithm primarily used for visualizing high-dimensional data in 2 or 3 dimensions. It is particularly popular in machine learning and data science for exploring patterns in complex datasets.

---

## 🧠 Summary

- **Type**: Non-linear dimensionality reduction
- **Invented by**: Laurens van der Maaten and Geoffrey Hinton (2008)
- **Purpose**: Embedding high-dimensional data for human-interpretable visualization

---

## 🎯 Main Features

- Converts similarities between data points to joint probabilities
- Minimizes the Kullback–Leibler divergence between high-dimensional and low-dimensional distributions
- Preserves local structure (neighborhoods) in the data
- Often used for visualizing clusters in machine learning outputs

---

## 🔬 Common Use Cases

- Visualizing latent space of neural networks
- Exploring high-dimensional embeddings (e.g., word vectors, image features)
- Clustering analysis
- Anomaly detection (visual exploration)

---

## 📊 Comparison to Similar Algorithms

| Algorithm       | Type                       | Preserves Local Structure | Preserves Global Structure | Common Use                     |
|-----------------|----------------------------|--------------------------|---------------------------|---------------------------------|
| t-SNE           | Non-linear, probabilistic   | ✅ Yes                     | ❌ No                       | Visualization, cluster insight |
| [[PCA]]         | Linear                      | ⚠️ Limited                 | ✅ Yes                      | Preprocessing, variance analysis|
| UMAP            | Non-linear, manifold        | ✅ Yes                     | ⚠️ Some                    | Faster alternative to t-SNE    |
| Isomap          | Non-linear, manifold        | ✅ Yes                     | ✅ Yes                      | Manifold learning              |

---

## ✅ Strengths

- Excellent for revealing local clusters and patterns
- Intuitive 2D/3D plots for exploration
- Widely supported in ML toolkits (e.g. scikit-learn, TensorFlow)

---

## ❌ Weaknesses

- Slow on large datasets
- Poor at preserving global relationships
- Stochastic → results can vary between runs unless seed is fixed
- Harder to interpret quantitatively (good for visualization, not for further computation)

---

## 🌐 External References

- [Original paper (2008)](https://www.jmlr.org/papers/volume9/vandermaaten08a/vandermaaten08a.pdf)
- [Scikit-learn t-SNE docs](https://scikit-learn.org/stable/modules/generated/sklearn.manifold.TSNE.html)

---

## 🔗 Related Notes

- [[PCA]]
- [[UMAP]]
- [[Dimensionality Reduction]]
- [[Cluster Analysis]]
- [[Machine Learning]]

---
