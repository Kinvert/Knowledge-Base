# Horovod

`Horovod` is a distributed training framework developed to make multi-GPU and multi-node deep learning scaling easier, especially for TensorFlow, Keras, PyTorch, and Apache MXNet models.

---

## Overview

`Horovod` was created to remove a lot of the boilerplate around distributed training and let teams scale training jobs with less framework-specific complexity.

Core goal: train faster by splitting work across many devices while preserving the same model semantics and converging to similar accuracy.

Main idea:

- Run the same training code on multiple workers.
- Replicate model/optimizer state.
- Synchronize gradients each step so all workers stay consistent.

---

## Why it was created

It started as an attempt to bring efficient distributed training to deep learning users who were hitting scaling friction in multi-GPU setups.

The framework became known for:

- simple API (`hvd.init()`, `DistributedOptimizer`, `broadcast_parameters`).
- reduced setup complexity versus hand-rolled MPI code.
- strong scaling patterns for data-parallel training.

---

## Core architecture

Horovod is primarily MPI-oriented distributed data parallel training.

- **Communication primitive**: usually allreduce.
- **Backend support**: NCCL, Gloo, and optional cloud/host communication plugins depending on install.
- **Execution model**: typically one process per GPU.
- **Integration point**: wraps the existing optimizer so users can minimize changes in training scripts.

Key flow:

1. Each worker computes gradients from its local mini-batch.
2. Gradients are reduced across workers.
3. Updated gradients are applied by each worker's optimizer locally.
4. Model parameters stay in sync because the allreduce and broadcast steps coordinate state.

---

## Typical setup steps

At a high level:

1. Install Horovod with framework and CUDA-compatible dependencies.
2. Ensure launcher command includes host/rank metadata.
3. Add one-time Horovod init and optimizer wrapping in training loop.
4. Add optional learning-rate scaling logic.

A simple PyTorch style pattern:

```python
import torch
import torch.nn as nn
import torch.optim as optim
import horovod.torch as hvd

hvd.init()

torch.cuda.set_device(hvd.local_rank())
model = nn.Linear(128, 10).cuda()
optimizer = optim.SGD(model.parameters(), lr=0.01 * hvd.size())

optimizer = hvd.DistributedOptimizer(
    optimizer,
    named_parameters=model.named_parameters(),
)

hvd.broadcast_parameters(model.state_dict(), root_rank=0)
```

Notes:

- Use a scaled learning rate with `hvd.size()` as a starting point.
- Use collective communication correctly for network topology and GPU affinity.

---

## Performance behavior

Horovod is mostly effective when:

- model compute dominates per-step communication,
- minibatch sizes are sufficiently large,
- network topology is fast enough (especially for frequent allreduce).

It can be bottlenecked when:

- models are tiny and communication dominates,
- bandwidth is weak,
- parameter count is huge with small local batch sizes.

Tuning levers:

- gradient fusion (`HOROVOD_FUSION_THRESHOLD` and related options),
- adjusting batch size and accumulation,
- selecting launch affinity and NIC/GPU placement,
- choosing NCCL/Gloo correctly per hardware.

---

## Comparison chart

| Framework | Scaling model | Setup model | Communication style | Framework coverage | Best fit |
|---|---|---|---|---|---|
| Horovod | Data-parallel, MPI-style worker-per-GPU | moderate | Allreduce (NCCL/Gloo) | TF, PyTorch, MXNet (varies by version) | Existing multi-framework teams wanting similar code path |
| PyTorch DDP | Data-parallel built into PyTorch | moderate | Allreduce via torch.distributed backend | PyTorch only | PyTorch-first teams, native integration |
| DeepSpeed | Data-parallel + optimizer/state partitioning + ZeRO | high | Allreduce + optimizer/offload flows | PyTorch | Large LLM and memory-intensive training |
| FairScale/FSDP | Sharded model/optimizer states | high | Allreduce + sharding collectives | PyTorch | Memory scaling for very large models |
| DeepSpeed/FSDP mixed | Varies | high | Advanced sharding/compression/partition | PyTorch | Extreme scale and parameter-efficient memory patterns |

---

## When to use Horovod

Use Horovod when:

- you need to scale existing code quickly across many GPUs/nodes,
- you value a consistent abstraction across TensorFlow/PyTorch at the project level,
- you need straightforward launch and consistent training behavior.

Prefer alternatives if:

- you are deeply in PyTorch-only advanced optimization,
- you need advanced sharding and optimizer state partitioning as a primary feature,
- your model and input pipeline are already tuned for newer PyTorch-native distributed ecosystems.

---

## Pros and cons

- Pros
  - simpler migration path from single-GPU code,
  - broad framework compatibility,
  - good community examples for multi-node launch.
- Cons
  - less native deep PyTorch optimizer-sharding sophistication than newer stacks,
  - tuning can still be required for irregular network topologies,
  - one more runtime dependency layer in some environments.

---

## Practical caveats

- Determinism can vary with non-deterministic ops and fused kernels.
- If you see divergence, check:
  - rank ordering,
  - learning-rate scaling,
  - dataset shuffling consistency,
  - mixed-precision path assumptions.
- Be explicit about environment variable ordering and launch command in cluster docs to prevent accidental mismatched ranks.

---

## Related notes

- [[PyTorch]]
- [[TensorFlow]]
- [[TensorRT]]
- [[DeepSpeed]] (if present in your local notes)

---

## Official sources

- Horovod GitHub: https://github.com/horovod/horovod
- Horovod docs: https://horovod.readthedocs.io/
- API reference: https://horovod.readthedocs.io/en/stable/api.html
- RFC/paper context: https://arxiv.org/abs/1802.05799
