# Sinkhorn's Algorithm

## Overview

Sinkhorn's Algorithm, also known as the Sinkhorn-Knopp algorithm, is an iterative matrix scaling method used to transform a non-negative matrix into a *doubly stochastic matrix*—a matrix where all rows and columns sum to 1. It plays a key role in optimal transport problems, probabilistic modeling, computer vision, and machine learning, particularly in scenarios involving matching, permutations, and structured prediction.

---

## Motivation and Applications

### Optimal Transport
Sinkhorn's Algorithm regularizes the optimal transport problem with an entropic penalty, making it computationally efficient and differentiable—suitable for modern deep learning applications.

### Use Cases
- Matching tasks in unsupervised learning (e.g., clustering, alignment)
- Solving the assignment problem (e.g., in tracking or attention mechanisms)
- Comparing distributions in generative models (e.g., Wasserstein GANs)
- Permutation matrices in neural architectures (e.g., SortNet, Deep Perm Set)
- Graph matching and structured prediction

---

## Mathematical Formulation

Given a cost matrix \( C \in \mathbb{R}^{n \times n} \), the entropically regularized optimal transport problem minimizes:

**Objective:**
Minimize ⟨P, C⟩ − ε * H(P)

Subject to:
- \( P \mathbf{1} = r \)
- \( P^\top \mathbf{1} = c \)

Where:
- \( P \): transport matrix (to be solved)
- \( r \), \( c \): target row and column sums (often uniform)
- \( H(P) \): entropy of \( P \)
- \( \epsilon \): regularization strength

The solution can be obtained by alternating row and column normalization:

1. Initialize \( K = \exp(-C / \epsilon) \)
2. Iteratively compute scaling vectors \( u \), \( v \) such that:

   \( u = r / (K v) \)  
   \( v = c / (K^T u) \)

3. Then compute:  
   \( P = \text{diag}(u) K \text{diag}(v) \)

This process converges to a matrix close to doubly stochastic when \( \epsilon > 0 \).

---

## Properties

| Property                     | Description                                                                 |
|-----------------------------|-----------------------------------------------------------------------------|
| Convergence                 | Guaranteed under mild conditions for strictly positive matrices             |
| Differentiability           | Fully differentiable; allows end-to-end training in neural networks         |
| Regularization              | The entropy term smooths the solution and improves numerical stability      |
| Speed                       | Much faster than traditional LP-based optimal transport solvers             |
| Scaling behavior            | Works well for moderate-sized matrices; large-scale variants exist          |

---

## Sinkhorn vs Hungarian vs Auction Algorithm

| Feature                      | Sinkhorn                 | [[Hungarian Algorithm]]    | [[Auction Algorithm]]       |
|-----------------------------|--------------------------|-----------------------------|-----------------------------|
| Output                      | Soft (approximate)       | Hard (exact)                | Hard (approximate)          |
| Differentiable              | ✅                       | ❌                          | ❌                          |
| Use in Deep Learning        | Very common              | Rare                        | Rare                        |
| Complexity                  | \(O(n^2)\) per iteration | \(O(n^3)\)                  | \(O(n^2 \log n)\)           |
| Regularization              | Entropic (ε)             | None                        | None                        |
| Output Format               | Doubly stochastic matrix | Permutation matrix          | Permutation matrix          |

---

## Variants and Extensions

- **Sinkhorn-Attention**: Used in transformer-like architectures for structured outputs.
- **Gumbel-Sinkhorn Networks**: Add Gumbel noise for sampling permutations.
- **Unbalanced Optimal Transport**: Handles mass mismatches using additional divergence penalties.
- **Online Sinkhorn**: Scales the algorithm to very large datasets using stochastic updates.

---

## When to Use Sinkhorn's Algorithm

✅ Use when:
- You need soft assignments or a differentiable approximation.
- You're dealing with distributions, not hard sets.
- You need scalability or GPU-accelerated computations.

🚫 Avoid when:
- You need an exact permutation or assignment.
- You're constrained to binary decision outputs.
- High accuracy on small matrices is critical (Hungarian may be better).

---

## Software and Libraries

| Library         | Language | Sinkhorn Support | Notes                                    |
|----------------|----------|------------------|------------------------------------------|
| POT (Python Optimal Transport) | Python   | ✅               | High-level API for OT problems           |
| GeomLoss        | Python   | ✅               | GPU-accelerated, integrates with PyTorch |
| ott-jax         | Python   | ✅               | Google’s fast JAX-based OT library       |
| SciPy           | Python   | ❌               | Has traditional assignment algorithms    |
| KeOps           | Python/C++ | ✅             | Efficient for large-scale kernel ops     |

---

## Summary

Sinkhorn’s Algorithm is a cornerstone tool for modern machine learning problems involving optimal transport and soft assignment. Its differentiability, scalability, and entropic smoothing make it ideal for integration into neural models and structured prediction systems.

For robotics and perception systems, it’s particularly useful in matching sensor observations, point correspondences, and track association—often providing robust, probabilistic alternatives to classical hard-matching algorithms.
