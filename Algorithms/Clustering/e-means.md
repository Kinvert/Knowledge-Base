# e-means

**e-means** is a mini-batch solver for the same fixed-$k$, squared-Euclidean clustering objective as [[KMeans]]. Introduced in a November 2026 preprint by researchers at Meta FAIR and École des Ponts, it combines **exponential moving averages (EMA)** with scheduled updates, dead-cluster revival, finalization and empirical scaling laws for parameter selection.

The **e** refers to exponential averaging. It is not a method for automatically discovering the number of clusters, nor simply another name for expectation-maximization with Gaussian mixtures.

---

## 🎯 Same Problem, Different Optimizer

For vectors $x_i\in\mathbb{R}^d$, choose $k$ centroids to minimize:

$$
\operatorname{MSE}(X,C)=\frac{1}{n}\sum_{i=1}^{n}\min_{j=1,\ldots,k}\|x_i-c_j\|_2^2.
$$

- **Lloyd's algorithm:** assign the entire dataset, then replace every nonempty centroid with its assigned points' mean.
- **e-means:** make many smaller updates during a shuffled data pass, forgetting older assignment statistics progressively.
- Both require a chosen $k$, a suitable representation and a distance objective that fits the application.
- Better optimization does not make squared Euclidean distance robust to outliers or appropriate for arbitrary cluster geometry.

Their nearest-centroid partitions connect directly to [[Voronoi Clustering]]. Lower quantization MSE is not automatically a better semantic grouping or better downstream retrieval.

---

## ⚙️ EMA Counts and Sums

For each cluster, maintain a running mass $N_j$ and vector sum $S_j$. A mini-batch contributes count $n_j$ and sum $s_j$ after nearest-centroid assignment:

$$
N_j\leftarrow(1-\alpha)N_j+\alpha n_j,\qquad
S_j\leftarrow(1-\alpha)S_j+\alpha s_j,\qquad
c_j=\frac{S_j}{N_j}.
$$

Here $\alpha=1-\gamma$ if the implementation/paper expresses the rule using retention factor $\gamma$.

This is **not generally an EMA of batch means with a fixed centroid mixing weight**. Counts influence how far the centroid moves.

For example, start with $N=10$, $S=40$, then observe two assigned points with sum $24$, using $\alpha=0.25$: the new mass is $8$, sum $36$, and centroid $4.5$. Directly mixing the old mean and batch mean would instead give $6$.

If a batch assigns no points to a cluster, its mass and sum decay together; its position stays unchanged before any revival. With a full-dataset batch and $\alpha=1$, the **update rule** reduces to a Lloyd update, subject to corresponding empty-cluster handling. That does not make every component of the full e-means recipe identical to Lloyd.

---

## 🧠 More Than an EMA Update

| Component | Purpose | Important qualification |
|---|---|---|
| Small initial statistical mass | Prevent starting seeds from dominating accumulated statistics | Seeds and randomness still affect the run |
| Cosine step schedule | Move from more responsive updates toward stabilization | The paper schedules toward one-tenth of the initial step, not zero |
| Dead-cluster revival | Reuse underoccupied centroids by splitting active donor clusters | Not a requirement that clusters have equal size |
| Empirical batch/step laws | Choose batch size and initial step from dataset size, $k$ and epoch budget | Fitted heuristics, not universal optimality guarantees |
| Last-epoch-mean finalization | Recompute means from assignments collected during the final pass | Does not perform a new nearest-centroid assignment |
| Optional slow-EMA finalization in the paper | Improve held-out quantization error | A separate train-versus-test tradeoff; check implementation support |
| Specialized assignment kernels | Translate algorithmic savings into CPU/GPU runtime gains | Hardware, precision and backend selection matter |

The paper reports improved escape from poor local configurations, but e-means is still a nonconvex stochastic optimization method—not a global-optimum guarantee. Its loss need not decrease after every mini-batch as in idealized Lloyd iterations.

---

## 📊 Comparison with k-means and Other Methods

Rows include both optimizers for the same problem and algorithms with genuinely different objectives.

| Method | Objective / update idea | When it fits | Main limitation or distinction |
|---|---|---|---|
| **e-means** | Fixed-$k$ squared-distance objective; EMA mini-batches and a coordinated training recipe | Large static embedding sets and codebooks | Same geometric/outlier limitations as the objective; empirical scaling laws |
| **Lloyd's k-means** | Full assignments followed by exact means | Simple baseline; small/moderate dense problems | Full-data updates can spend many passes in a poor local solution |
| **k-means++ + Lloyd** | Better initial seeds, then Lloyd updates | Initialization-sensitive clustering | k-means++ is a seeding procedure, not a different clustering objective; seeding costs time |
| **MiniBatchKMeans / MacQueen-style updates** | Accumulate assignment statistics with decreasing effective centroid steps | Fast incremental/scikit-learn workflows | Old assignments can retain stale influence; a different update recipe from e-means |
| **Elkan / Flash-KMeans** | Accelerate Lloyd with distance bounds or optimized exact GPU assignment | Keep Lloyd-style optimization while improving its implementation | Faster kernels or skipped distances do not themselves change the optimizer |
| **Gaussian mixture model with EM** | Fit mixture likelihood, means, covariances and soft responsibilities | Probabilities, overlapping/elliptical components and density modeling | More parameters and covariance/regularization choices; not the same MSE problem |
| **DBSCAN / HDBSCAN** | Density connectivity/hierarchy with noise handling | Nonconvex groups, noise and no prescribed final $k$ | Need density-scale settings; high-dimensional distances and density variation still matter |
| **Agglomerative clustering** | Merge groups according to linkage until a selected cut | Hierarchy or linkage-specific structure | Linkage and stopping rule matter; many formulations become expensive at scale |

**Practical choice:** use e-means when the desired result is a fixed-size vector codebook. Choose a different clustering model when the real requirement is outlier rejection, density connectivity, covariance structure or a hierarchy.

---

## 📐 Scaling Laws and Low-Dimensional Data

The authors fit power laws of the form $B\propto k^{b_k}n^{b_n}T^{b_T}$ for batch size and a related law for $\alpha_0$. This reduces the need for a separate batch/step hyperparameter sweep.

It does **not** remove choices about $k$, epoch budget, normalization, embedding quality, hardware or evaluation criteria. Quality-oriented and throughput-oriented batches also trade convergence quality against execution efficiency.

The paper explicitly warns that its laws were tuned for relatively high-dimensional data and are not optimal for very small or low-dimensional problems. The inspected Faiss implementation adjusts defaults:

- `batch_law="auto"` chooses the quality law for $d\le16$, throughput otherwise.
- Default cluster revival is disabled for $d\le2$.
- Manual `batch_size` and `alpha0` overrides are available.
- Current implementation coefficients, caps and initialization mass differ from the paper; use the selected version's recipe rather than mixing constants from both.

For 2D/3D robotics points, benchmark the actual workload instead of importing high-dimensional embedding speedups as a guaranteed result. Feature scaling or [[PCA]] also changes the clustering problem and should be applied consistently across comparisons.

---

## ⏱️ What the Speed Claims Mean

The original preprint reports the following against the evaluated Lloyd baselines:

| Measurement | Authors' reported result | Scope |
|---|---|---|
| Passes to match reference test MSE | e-means: 1.4–2.6; reference: 25 | Evaluated embedding datasets/configurations in paper Table 4 |
| GPU wall-clock speedup at that quality | 9.5–45.5× | Reported comparison methodology; single GPU through 100M rows, multi-GPU at 1B |

An epoch is a data pass, **not one centroid update**. e-means performs multiple updates per pass. Compare wall time at matched quality rather than assuming a fixed ratio from epoch counts.

The paper uses H100 GPUs for its GPU experiments. Dataset size, dimension, $k$, initialization, data transfer, precision, compilation and the exact baseline implementation affect the result. These are authors' measurements, not locally reproduced speedups.

The largest-scale experiments additionally use approximate IVF assignment to avoid exhaustive comparison with millions of centroids. Keep that search approximation separate from the default exact-assignment solver when comparing quality or computational cost.

---

## 🛠️ Implementation and API Gotchas

Upstream code lives in `faiss.contrib.e_means`:

- `NumpyEmeans`: NumPy/CPU path using Faiss nearest-centroid assignment.
- `Emeans`: PyTorch tensor path, including GPU backends.
- Current upstream also provides quantized, distributed and centroid-sharded variants.

A typical API sequence is `NumpyEmeans(d, k, nepochs=T, seed=0)`, then `centroids, last_labels = em.train(x)`. For a NumPy Euclidean model, **`distances, nearest_labels = em.assign(x)`** computes squared distances and nearest labels against the final centroids.

Important boundaries:

- Returned training labels are **last-pass assignments** made while centroids were moving; they need not be nearest to the final centroids.
- `em.obj` describes moving-pass error, not a fresh final-centroid evaluation.
- A new `assign` pass has a real computation cost; include it when comparing end-to-end pipelines.
- Training-set subsampling via `max_points_per_centroid` can leave `-1` labels for excluded rows. These are not discovered noise clusters.
- Tensor data must already be on the intended device; passing NumPy data does not automatically use a GPU.
- GPU input needs an explicit finite-value check; reduced-precision assignment can change clustering outcomes.
- Current backends have hardware/dependency restrictions and first-run compilation costs; inspect `em.recipe` rather than assume an H100-specific kernel runs on every GPU.
- `spherical=True` changes to normalized centroid directions/similarity semantics. Do not interpret those scores as Euclidean squared distances.

This is a recent contribution: verify the chosen Faiss build contains the module. Do not assume every installed/released wheel exposes the same API as repository `main`.

---

## 🔎 Evaluation and Applications

Useful applications include embedding codebooks, image-feature quantization, data curation, coarse partitions for vector search and visual vocabularies such as [[BoW]]. [[FLANN]] provides a related example of clustering structures used for nearest-neighbor search, not an e-means implementation.

For a meaningful comparison:

1. Use the same vectors, scaling/normalization, $k$, precision policy and train/held-out split.
2. Report fresh nearest-final-centroid train and held-out MSE; do not compare one method's moving-pass objective with another's final inertia.
3. Use multiple seeds and record wall time, memory, initialization and assignment costs.
4. Record empty/underoccupied clusters and whether revival or subsampling was enabled.
5. Measure downstream behavior: reconstruction, retrieval recall or semantic utility may matter more than a small MSE change.

The core EMA arithmetic and full-batch Lloyd equivalence were checked with small numerical fixtures for this note. No Faiss e-means training run or paper-scale speed benchmark is claimed.

---

## 🔗 Related Notes

- [[Clustering]] — method families and model-selection tradeoffs.
- [[KMeans]] — objective and Lloyd baseline.
- [[Voronoi Clustering]] — nearest-centroid partition geometry.
- [[Embeddings]] — representations being quantized/grouped.
- [[PCA]] — preprocessing changes distances and dimensionality.

---

## 📚 External Resources

- [Original e-means paper, arXiv:2611.09501v1](https://arxiv.org/html/2611.09501v1) — November 6, 2026 preprint; algorithms, scaling-law limitations and benchmark methodology.
- [Authors' overview and visualizations](https://e-means.github.io/) — EMA versus accumulated statistics and dead-cluster revival.
- [Faiss implementation README, pinned revision](https://github.com/facebookresearch/faiss/blob/2b648f0605f2ef3d14c4705350f8a0b214463878/contrib/e_means/README.md) — API, labels, devices, precision and distributed-memory constraints.
- [Pinned shared recipes](https://github.com/facebookresearch/faiss/blob/2b648f0605f2ef3d14c4705350f8a0b214463878/contrib/e_means/base.py) and [NumPy implementation](https://github.com/facebookresearch/faiss/blob/2b648f0605f2ef3d14c4705350f8a0b214463878/contrib/e_means/e_means.py) — implementation-specific defaults.
- [scikit-learn clustering guide](https://scikit-learn.org/stable/modules/clustering.html) and [Gaussian mixtures guide](https://scikit-learn.org/stable/modules/mixture.html) — comparison objectives, geometry and API behavior.
