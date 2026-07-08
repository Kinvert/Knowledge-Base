# Safetensors

`Safetensors` is a file format for saving/loading machine learning model tensors with a **`.safetensors`** extension (for example: `model.safetensors` or `pytorch_model-00001-of-00003.safetensors`).

---

## Overview

- Purpose: store model tensor weights safely and efficiently.
- It is designed to replace unsafe Python pickle-based checkpoint loading.
- Commonly used on Hugging Face for model checkpoints instead of `.bin` in modern uploads.

---

## Why `safetensors` exists

Traditional checkpoint formats like `.bin`/`pickle` can execute arbitrary code during load because pickling is Python object deserialization.
`Safetensors` avoids this by storing only raw tensor data and metadata, with no executable payload.

Hugging Face docs describe it as a safer tensor format and recommend converting from pickle-based checkpoints to `.safetensors`.

---

## File extension

- The extension is **`.safetensors`**
- It is usually single-file or sharded:
  - `model.safetensors`
  - `model-00001-of-00004.safetensors`

---

## Core format idea

At a high level, a `.safetensors` file has:

1. a metadata header (JSON) with tensor names, shapes, dtypes, and offsets
2. a raw binary buffer with tensor bytes

This gives:

- safe parsing (no code execution path on load)
- faster loads in many cases due to zero-copy / memory-mapped behavior

---

## Security properties

- No arbitrary code execution during load by design
- Safer for untrusted checkpoints than pickle-based formats
- Header bounds checks reduce malformed-file parsing risk

---

## Performance & loading behavior

- Fast loading and partial reads
- Useful for large checkpoints when full eager deserialization is not needed
- Helpful for multi-GPU workflows and selective tensor loading

---

## What a `.safetensors` checkpoint contains

A checkpoint typically stores:

- model tensors
- tensor metadata in the header

It does **not** store arbitrary Python objects for execution.

---

## Practical usage

```python
# Install:
# pip install safetensors

import torch
from safetensors.torch import save_file, load_file

# Save

tensors = {
    "weight": torch.randn(256, 256),
    "bias": torch.randn(256),
}
save_file(tensors, "model.safetensors")

# Load
loaded = load_file("model.safetensors")
print(loaded["weight"].shape)
```

Load selected tensors only:

```python
from safetensors import safe_open

with safe_open("model.safetensors", framework="pt", device=0) as f:
    keys = list(f.keys())
    subset = f.get_tensor("weight")
```

---

## Hugging Face and ecosystem context

- HF docs position it as a safe tensor format and publish conversion workflows from pickle-based formats.
- The PyTorch project page also describes it as a secure, fast model-weight format.

Useful references:

- https://huggingface.co/docs/safetensors/index
- https://huggingface.co/docs/safetensors/en/convert-weights
- https://pytorch.org/projects/safetensors/

---

## Comparison chart

| Format | Extension | Safe from code execution on load | Typical speed | Ecosystem fit | Typical use |
|---|---|---|---|---|---|
| Safetensors | `.safetensors` | Yes | High / zero-copy friendly | HF, PyTorch, multi-framework | Model weight distribution + secure loading |
| Pickle checkpoint | `.bin` / `.pkl` | No | Varies | Legacy PyTorch/legacy scripts | Generic Python object checkpoints |
| Torch binary | `.pt` / `.pth` | No guaranteed for untrusted sources | Varies | PyTorch workflows | State dict / checkpoint persistence |
| GGUF | `.gguf` | Safer inference runtime format | Optimized for inference | Popular in quantized runtime stacks | LLM inference quantized deployment |

---

## When to use safetensors

Use `safetensors` when:

- distributing model weights publicly
- wanting safer loading semantics than pickle
- using Hugging Face loading flows
- needing fast partial tensor loading

Avoid if:

- you rely on serialized arbitrary Python objects in checkpoints
- you only need very narrow legacy tooling that expects pickle-only formats

---

## Related notes

- [[PyTorch]]
- [[Hugging Face]]
- [[Pickle]]
- [[Model Quantization]]
- [[GGUF]]
