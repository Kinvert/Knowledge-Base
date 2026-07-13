# Miril-Drone-2B-1

`Miril-Drone-2B-1` is an open-weight aerial vision-language model (VLM) published by Miril.ai on Hugging Face.

---

## Overview

- Goal: turn drone-view imagery into structured, machine-readable answers for operator review and autonomy tooling.
- Core use style: image + text prompt -> JSON response.
- The main model card describes it as a **2B-class aerial VLM** for civilian drone workloads.
- It is intentionally positioned as a **review/assistance layer**, not a full flight stack.

---

## Official links

- Hugging Face model card: https://huggingface.co/MirilAI/Miril-Drone-2B-1
- Miril home: https://miril.ai/
- Demo space: https://huggingface.co/spaces/MirilAI/Miril-Drone-2B-1-Demo
- Variant: 4-bit CUDA: https://huggingface.co/MirilAI/Miril-Drone-2B-1-bnb4
- Variant: 8-bit CUDA: https://huggingface.co/MirilAI/Miril-Drone-2B-1-bnb8
- Variant: 4-bit MLX: https://huggingface.co/MirilAI/Miril-Drone-2B-1-MLX-4bit
- Variant: 8-bit MLX: https://huggingface.co/MirilAI/Miril-Drone-2B-1-MLX-8bit
- Lineage model reference (WALDO30): https://huggingface.co/StephanST/WALDO30

---

## What it does

From the model card and examples:

- Scene captions
- Aerial visual question answering
- Ground/context review for inspection and first-response workflows
- Coarse landing/delivery area pointing cues
- Structured JSON outputs for downstream tools

Typical response patterns are strict JSON keys such as:

- `caption_v1` -> `{ "caption": string }`
- `simple_answer_v1` -> `{ "answer": string }`
- `operational_coordinate_v2` -> `{ "coordinate_system", "description", "point_2d", "point_semantics", "status", "x", "y" }`

---

## Core architecture and assumptions

- The public card says the checkpoint is based on Gemma4 components and tuned for aerial-vision tasks.
- In practice, the release is described as a **V1** version with a strict schema-first output style.
- The same card emphasizes the model is primarily for image-plus-text tasks and **not** for native temporal video reasoning.

---

## Important limits (from official docs)

- Not a flight controller or autopilot.
- Not a certified safety system.
- V1 operational coordinates are rough review cues, not centimeter-accurate control points.
- Not guaranteed to detect small distant objects consistently (e.g., people at altitude).
- Must keep the safety model outside the model itself; use independent checks and human/verified planner loops.

From the card’s “Safety Notes” and “Intended Use” guidance, this is treated as a perception/review component, not a closed-loop control authority.

---

## Deployment options

The main repository is deployment-ready via multiple local serving paths:

- Transformers API
- vLLM
- SGLang
- Docker / docker model runner

The variants are mainly quantization/target-platform drops for practical edge deployment.

| Variant | Intended target |
|---|---|
| `Miril-Drone-2B-1` | Standard checkpoint |
| `Miril-Drone-2B-1-bnb4` | NVIDIA CUDA 4-bit quantized |
| `Miril-Drone-2B-1-bnb8` | NVIDIA CUDA 8-bit quantized |
| `Miril-Drone-2B-1-MLX-4bit` | Apple Silicon MLX 4-bit |
| `Miril-Drone-2B-1-MLX-8bit` | Apple Silicon MLX 8-bit |

---

## Practical integration pattern

1. Run a vision front-end on each frame.
2. Prompt with explicit JSON contract.
3. Parse strict JSON keys and schema version.
4. Feed into a validation layer (geofence, obstacle checks, occupancy context, confidence thresholds).
5. Forward only safe/validated outputs into navigation logic.

Keep the model outputs as *advisory signals* unless you add independent safety proof.

---

## For this note

This is mostly a domain-specific perception layer for aerial workflows, not an autonomy proof-of-safety component on its own.

Use this model when you need:

- fast structured drone imagery parsing,
- quick operator review automation,
- lightweight edge-compatible VLM behavior with constrained JSON output.

Prefer it for:

- triage and inspection workflows,
- human-in-the-loop landing/approach review,
- structured annotation capture.

Prefer not to use it for:

- direct closed-loop flight control,
- safety-critical decision making without independent runtime validation.

---

## Comparable models from other vendors

Models that are usually discussed in the same bucket as aerial/robotic vision-language systems.

| Model | Vendor | What to compare with | Official link | Why it may fit drone workflows |
|---|---|---|---|---|
| `Miril-Drone-2B-1` | Miril.ai | JSON-structured drone-view Q&A and grounding | https://huggingface.co/MirilAI/Miril-Drone-2B-1 | Strong baseline for structured aerial observations and schema-first parsing. |
| `Qwen/Qwen2.5-VL-7B-Instruct` | Alibaba Qwen | General VLM baseline and map-like visual reasoning | https://huggingface.co/Qwen/Qwen2.5-VL-7B-Instruct | Good for broad VLM tasks (charts, text reading, grounding, multimodal tool-use style prompts); supports long-video reasoning and 4-16384 visual tokens per image. |
| `OpenGVLab/InternVL2_5-8B` | OpenGVLab | Mid-size dense VLM with broad tool-friendly usage | https://huggingface.co/OpenGVLab/InternVL2_5-8B | Strong practical all-rounder if you want a non-Miril open model with 1B/2B/4B/8B/26B family options for scaling tests. |
| `openbmb/MiniCPM-o-2_6` | OpenBMB | Vision + speech + streaming multimodal fusion | https://huggingface.co/openbmb/MiniCPM-o-2_6 | Highest overlap with “edge + realtime operator-in-the-loop” pipelines; supports real-time audio/video stream workflows, though not drone-native action grounding. |
| `openbmb/MiniCPM-V-2` (MiniCPM-V 2.0) | OpenBMB | Dense VLM optimized for fine-grained visual detail | https://huggingface.co/openbmb/MiniCPM-V-2 | Strong OCR and high-resolution image handling (up to 1.8M pixels) at small footprint; useful when tiny objects/text matter. |
| `deepseek-ai/deepseek-vl2` | DeepSeek | Open MoE VLM family | https://huggingface.co/deepseek-ai/deepseek-vl2 | MoE variants are compact for the same task, with 1.0B/2.8B/4.5B activated params and explicit long-context/image tiling behavior for up to multiple images. |
| `llava-hf/llava-onevision-qwen2-7b-ov-hf` | LLaVA / lmms-lab + HF | Multi-image + video-capable multimodal model | https://huggingface.co/llava-hf/llava-onevision-qwen2-7b-ov-hf | Useful if your drone workflow needs multi-frame or multi-camera context with one chat-style model. |
| `HuggingFaceM4/Idefics3-8B-Llama3` | Hugging Face | Interleaved image-text prompting | https://huggingface.co/HuggingFaceM4/Idefics3-8B-Llama3 | Good for arbitrary image/text sequences (multiple images in one prompt), but it does not generate images and has limited built-in action/output schema controls. |
| `microsoft/kosmos-2.5` | Microsoft | Text-heavy image understanding | https://huggingface.co/microsoft/kosmos-2.5 | Strong when drone payload is inspection photos with dense labels, receipts/diagrams-like text layouts, or doc-like reading tasks. |
| `google/paligemma2-28b-pt-896` | Google | Large VLM foundation with strong segmentation/object outputs | https://huggingface.co/google/paligemma2-28b-pt-896 | High-capacity but heavier model; better fit when you need segmentation/bounding boxes and can afford larger compute budget. |

## Practical takeaway

Across this comparison, none of the alternatives are a direct “autonomy policy network” replacement yet.

- If your immediate goal is `image -> structured perception`, these all remain viable alternatives to Miril.
- If your goal is `perception -> action`, you still need an explicit controller layer (planner / policy head) no matter which VLM you pick.
- For **drone fleet scale with tight edge constraints**, test these in ascending compute: `MiniCPM-V-2` (2.8B) / `OpenGVLab InternVL2.5-8B` / `Qwen2.5-VL-7B` / `MiniCPM-o-2_6` / `deepseek-vl2` variants.

Useful links for this model class:

- Qwen 2.5-VL base/model cards: https://huggingface.co/Qwen/Qwen2.5-VL-7B-Instruct
- DeepSeek-VL2 paper + code: https://arxiv.org/abs/2412.10302 , https://github.com/deepseek-ai/DeepSeek-VL2
- LLaVA-OneVision paper + repo notes: https://llava-vl.github.io , https://github.com/LLaVA-VL/LLaVA-NeXT
- IDEFICS3 model card: https://huggingface.co/HuggingFaceM4/Idefics3-8B-Llama3
- OpenGVLab InternVL family: https://github.com/OpenGVLab/InternVL
