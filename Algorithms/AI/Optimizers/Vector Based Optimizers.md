# Vector Based Optimizers

Vector-based optimizers — in the ML/NN context — refers to optimization algorithms that reason about parameter *vectors* (or matrices) with structure-aware updates, as opposed to purely scalar/elementwise updates. This note surveys the landscape: classic per-parameter methods (SGD, Momentum), per-parameter adaptive methods (AdaGrad, RMSProp, Adam, AdamW), sign/momentum hybrids (Lion), structured / matrix-aware optimizers (K-FAC, natural gradient, Muon and related orthonormal/matrix-updating schemes), and other modern variants. Where useful I call out compute & memory tradeoffs, typical hyperparameters, and when to prefer each family.

---

## 🧭 Overview

- **What “vector-based” means here:** algorithms that operate on whole parameter vectors or matrices, exploit correlations between parameters, or use geometry-aware transforms (preconditioners, second-order approximations, matrix orthogonalizations).  
- **Contrast:** scalar updates (SGD with per-weight updates) vs. adaptive per-parameter scalings (Adam family) vs. structured/matrix-level updates (K-FAC, Muon).  
- **Why it matters:** structured updates can converge faster or require fewer tokens/epochs for the same generalization, but often at added implementation/compute complexity and communication cost in distributed training.

---

## 🧩 Core Concepts

- **First-order vs. second-order:** First-order uses gradients only (SGD, Adam). Second-order uses curvature information (Newton, quasi-Newton, K-FAC) or approximations to it.  
- **Per-parameter adaptivity:** Methods like AdaGrad/RMSProp/Adam adapt the step per parameter (elementwise statistics). AdamW fixes a regularization coupling issue by decoupling weight-decay.
- **Momentum & sign-based updates:** Momentum accumulates a velocity; sign-based methods (e.g., Lion) use a sign step, often cheaper in memory.
- **Matrix / layer-aware preconditioning:** K-FAC and Muon are examples where updates consider parameter matrices or layers as structured objects; Muon uses matrix orthogonalization techniques and per-layer transforms to produce efficient updates that can significantly speed training in some settings.

---

## 🔬 How the Major Families Work

### SGD (Stochastic Gradient Descent)
- Update: `θ ← θ - η * g` (possibly with momentum).  
- Pros: Simple, low memory, good generalization with proper tuning.  
- Cons: Sensitive to learning rate; per-parameter geometry ignored.

### Momentum / Nesterov
- Accumulate `v ← β v + g` then update. Nesterov looks ahead.  
- Pros: Faster on ill-conditioned problems than plain SGD.  
- Cons: Still global learning-rate tuning.

### AdaGrad / RMSProp / Adadelta
- Keep running scale per parameter (`s_t`), divide gradient by `sqrt(s_t)`.  
- Pros: Good for sparse gradients (AdaGrad).  
- Cons: AdaGrad’s learning rate decays; RMSProp/Adadelta fix some issues.

### Adam / AMSGrad / AdamW
- Adam: maintains per-parameter `m` (first moment) and `v` (second moment) and corrects bias; widely used.  
- AdamW: decouples weight decay from Adam’s update rule — shown to improve generalization vs. naive L2-as-weight-decay in adaptive optimizers.
- AMSGrad: tries to fix Adam's convergence pathologies by using the max of second-moment estimates.  
- Pros: Fast convergence, low tuning for many problems.  
- Cons: Can generalize worse than SGD in some regimes; requires extra memory for `m` and `v`.

### Sign / Low-Memory Methods (Lion, etc.)
- Lion uses sign-based updates with momentum-like accumulation — less memory than Adam (only one momentum) and performs competitively on many large-model tasks. Discovered via symbolic search.

### Second-order and Approximate Natural Gradient (Newton, L-BFGS, K-FAC)
- Use curvature approximations to build a preconditioner (approximate inverse Hessian) to scale gradient steps directionally.  
- K-FAC approximates Fisher blocks for layer-wise preconditioning — faster per-step progress but heavier compute & memory; used when curvature structure is important.

### Matrix / Layer-aware Optimizers (Muon, Dion, etc.)
- **Muon:** a structured optimizer that treats hidden layers (weight matrices) with matrix-orthonormalization-inspired updates and layer-level preconditioning. It has been used in small-scale record training runs and there is active work demonstrating how to scale it (adding weight decay, tuning per-parameter update scale) for large models. Implementations and follow-up engineering work (Flash-Muon, Microsoft / Dion repos) exist. Muon’s core idea reframes updates in matrix terms rather than elementwise adjustments.
- Pros: Can yield large-step, geometry-aware updates that accelerate training (especially for matrix-structured layers).  
- Cons: More complex kernels (matrix ops, Newton–Schulz iterations in some impls), communication costs in distributed setups, and needing extra implementation effort / specialized CUDA kernels.

---

## 📊 Comparison Chart (select algorithms)

| Algorithm | Memory | Per-step Cost | Structure-Aware? | Typical Use Cases | Key Strength |
|---|---:|---:|:---:|---|---|
| SGD (+momentum) | low | cheap | no | large-scale training, best generalization | simplicity, generalization |
| Adam | medium (m,v) | cheap | elementwise adaptive | quick convergence, NLP/vision pretraining | robust default for many tasks |
| AdamW | medium | cheap | elementwise + decoupled WD | same as Adam but better generalization when WD used | decoupled weight decay improves results. |
| Lion | low-medium | cheap | sign-based momentum | large LLMs, memory-constrained training | memory-efficient, competitive to Adam. |
| K-FAC | high | expensive | yes (layer-block) | models where curvature matters (small/medium models, research) | second-order-like speedups |
| Muon | medium-high | matrix ops, can be heavy | yes (matrix orthonormalization) | fast small-model training, being scaled to LLMs | structural, layer-matrix-aware updates; scaling tweaks required. |

*(This chart is a concise guide — real costs depend on implementation, batch size, hardware, and distributed strategy.)*

---

## ✅ Strengths (Vector / Matrix-aware Optimizers)

- Capture correlations across parameters (especially within a weight matrix or layer).  
- Can provide faster convergence in wall-clock time for some workloads (fewer epochs/tokens to target).  
- Potential to reduce the need for extensive LR scheduling or learning-rate grids when designed carefully.

---

## ❗ Weaknesses / Tradeoffs

- Higher implementation complexity — custom kernels or expensive matrix ops.  
- More memory, compute, and communication cost — can be a blocker for very large distributed runs without engineering investment.  
- Hyperparameter sensitivity: matrix-aware methods (e.g., Muon) often need careful scaling of per-parameter update magnitude and may require added weight decay to stabilize at large scale. Recent work on scaling Muon highlights these adjustments.

---

## 🧪 Practical Guidance & Hyperparameters

- **If you need simplicity & broad success:** start with `AdamW` (`β1=0.9, β2=0.999, eps=1e-8`) and tune LR & weight decay. AdamW is the better default than Adam when using weight decay.
- **If memory is constrained:** try `Lion` (fewer accumulators than Adam) or tuned SGD with momentum.
- **For research on faster convergence or smaller compute budgets per token:** consider Muon or K-FAC — but budget for implementation (Flash-Muon, specialized kernels) and tune per-layer scaling / decay.
- **Distributed training:** structured optimizers may require new communication patterns (synchronizing matrices or preconditioner state), so check implementations like `microsoft/dion` and community repos for practical approaches.

---

## 🧾 Implementations & Resources

- **Muon blog + writeups / implementation:** initial exposition and repo (Keller Jordan): “Muon: An optimizer for the hidden layers of neural networks.” Implementation available.
- **Scaling and evaluation studies:** early arXiv follow-ups investigate making Muon scale to LLM training (adding weight decay, tuning per-parameter scales).
- **Flash-Muon:** specialized CUDA implementation optimizing Muon’s matrix ops / Newton–Schulz steps. Useful if you plan to experiment at scale.
- **Microsoft / Dion repo:** includes implementations and distributed strategies for Dion & Muon variants. Useful engineering reference.
- **AdamW paper:** Loshchilov & Hutter (Decoupled Weight Decay Regularization). Essential reading for weight-decay best-practices with Adam.
- **Lion discovery:** Symbolic discovery paper that introduced Lion — a memory-efficient sign-momentum optimizer. Good to study as an alternative family.

---

## 🔧 Developer Tools & Implementation Notes

- Fast Muon-like updates often need custom CUDA kernels or batched matrix primitives (see Flash-Muon). Expect to use `cuda`, `cutlass`-style kernels, or efficient batched BLAS.
- For research experiments, implement Muon/K-FAC in a framework-friendly way (PyTorch/TF) using `torch.autograd` hooks and tested distributed reduction primitives. Reference Microsoft/Dion for distributed patterns.
- For Adam/AdamW/Lion, standard framework optimizers exist (`torch.optim.AdamW`, 3rd-party Lion implementations). Start from those unless you need the structural matrix updates.

---

## 🔗 Related Concepts / Notes

- [[Adam]] (Adaptive Moment Estimation)  
- [[AdamW]] (Decoupled Weight Decay)  
- [[Lion]] (Sign-momentum optimizer)  
- [[K-FAC]] (Kronecker-factored Approximate Curvature)  
- [[Natural Gradient]] (information-geometric preconditioning)  
- [[Second-Order Methods]] (Newton, L-BFGS)  
- [[Muon]] (Matrix / layer-aware optimizer)  
- [[Distributed Training]]  
- [[CUDA Kernels]] (for Flash-Muon style optimization)
- [[Optimizers]]

---

## 🏁 Summary / Takeaways

- “Vector-based” optimizers broadly capture methods that exploit structure beyond elementwise updates.  
- For practical large-scale work, `AdamW` remains a robust baseline; `Lion` is a compelling low-memory alternative; Muon and K-FAC represent structured approaches that can accelerate training but at engineering and compute cost.
- If you’re experimenting: prototype with existing `AdamW`/`Lion` in your training pipeline, then evaluate Muon/K-FAC only if you need the extra convergence/efficiency and can invest in optimized kernels and distributed engineering.
