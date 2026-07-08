# OpenVLA

`OpenVLA` is an open-source 7B vision-language-action policy from Stanford and collaborators that is designed as an accessible alternative to closed VLA baselines.

---

## Overview

The main claim: this is a real open-stack VLA that can be trained and fine-tuned with common tooling instead of requiring proprietary VLA infrastructure.

- Paper: `OpenVLA: An Open-Source Vision-Language-Action Model` (`arXiv:2406.09246`)
- Public release: open source repository and Hugging Face checkpoints
- Trained as a visual-language-action model for general manipulation in multi-task settings
- Reported as 7B parameters and open, with practical fine-tuning support (full, PEFT/LoRA)

Publicly cited performance points:
- Uses 970k robot demonstration trajectories from Open X-Embodiment
- Outperforms the older closed RT-2-X baseline by about **16.5% absolute task-success points** across benchmark tasks with **fewer parameters** (~7x smaller in reported comparison)
- Fine-tuning reports strong gains over from-scratch imitation baselines such as diffusion methods in multi-object language-grounded settings

---

## Core mechanism

OpenVLA uses a pretrained VLM backbone and adds robot control prediction:
- visual encoding from multiple visual towers (`SigLIP` + `DINOv2` style features),
- language/visual embeddings projected into a language-model style decoder (`Llama 2` 7B),
- action output in tokenized form that maps to robot control commands.

The project is structured around:
1. start from pretrained multimodal base,
2. fine-tune on robot manipulation trajectories,
3. run in closed-loop robotic evaluation,
4. further adapt with PEFT/full fine-tuning for target setups.

OpenVLA’s practical contribution is not only raw architecture but the "open and re-usable" path to build generalist policies.

---

## Why it matters for next-token style robotics

For token-first policy workflows:
- it already uses token prediction for action generation,
- it has a defined open adaptation path for dataset-specific fine-tuning,
- it is usually much easier to reproduce than closed-source VLA stacks.

For pure drop-in replacement arguments:
- it does not replace your entire training environment unless your baseline already matches VLM-style inputs,
- it is multimodal and heavier than classical Decision/Trajectory Transformer token stacks.

---

## Comparison chart

| Method | Model size / stack | Data regime | Best strength | Main limitation |
|---|---|---|---|---|
| **OpenVLA** | 7B, open VLM + action tokenizer pipeline | Open X-Embodiment (970k traj) + downstream fine-tuning | open reproducible VLA baseline, strong policy transfer after fine-tune | deployment quality varies by robot stack/adapter setup |
| **RT-2-X** | large VLA (open-source details from RT-X family; often larger) | OpenX + multimodal pretraining style | semantic instruction-following + open question coverage | higher compute and harder-to-reproduce closed/public split |
| **RT-2** | vision-language-action transformer | web-scale + robot data | strong reasoning-like behavior in language-conditioned tasks | proprietary ecosystem complexity and heavier inference burden |
| **RT-1-X** | RT-1-style transformer on open-embodiment mixture | OpenX mixture | solid cross-embodiment transfer from tokenized action format | less VLA-level language grounding than open VLA families |
| **SARA-RT** | RT-X family with attention efficiency up-training | RT-X-compatible setups | low-latency adaptation for RT-style stacks | still research-stage upgrade layer |
| **Diffusion Policy** | generative action trajectory model | demonstration datasets | smooth multi-modal control trajectories | denoising inference overhead, usually separate pipeline |
| **Decision / Trajectory Transformer** | state/action or return-conditioned token trajectories | offline logs | easy to reproduce sequence-baseline | narrower modality, less direct multimodal grounding |

---

## Pros

- Fully open architecture and model release path
- Direct relevance to real-world manipulation with instruction conditioning
- Strong benchmark position vs closed RT-X-like systems
- Explicitly built for practical fine-tuning and adaptation to new robots
- Easier reproducibility for collaboration-heavy or budget-limited projects

## Cons

- Requires careful tokenizer/model coupling to your robot action representation
- Closed-loop deployment needs engineering on inference stack and safety wrappers
- Performance gains depend on quality and alignment of robot demonstrations
- Not a minimal NTP baseline; more involved than pure transformer imitation-only policies

---

## Practical implementation notes

- Prefer OpenVLA when you want an open, full-stack VLA baseline, not only a single module swap.
- Start with a frozen encoder and LoRA-style adaptation before full fine-tune.
- Keep robot action scaling and frame conventions consistent between demos and execution.
- Evaluate with:
  - same task set,
  - same episode timeout,
  - consistent camera and language prompt templates.
- For token-first experiments, compare against:
  - vanilla Diffusion Policy,
  - RT-2(-X),
  - and a Decision Transformer baseline.

Simple workflow sketch:

```text
Load OpenVLA base
Load OpenX-style trajectory dataset (or your dataset converted to compatible format)
Fine-tune via LoRA for target robot
Validate in closed-loop rollout with prompt + image stream
Monitor safety rails: workspace bounds, collision, command clamp
```

---

## Related notes

- [[Open X-Embodiment]]
- [[RT-2]]
- [[RT-1-X]]
- [[RT-2-X]]
- [[SARA-RT]]
- [[Decision Transformer]]
- [[Trajectory Transformer]]
- [[Diffusion Policy]]

## External references

- [OpenVLA homepage](https://openvla.github.io/?trk=public_post-text)
- [OpenVLA arXiv](https://arxiv.org/abs/2406.09246)
- [OpenVLA GitHub](https://github.com/openvla/openvla)
- [Open X-Embodiment](https://robotics-transformer-x.github.io/)
- [RT-1](https://robotics-transformer1.github.io/)
- [RT-2](https://robotics-transformer2.github.io/)

---

## Summary

OpenVLA is the practical open VLA baseline to keep around when you want tokenized vision-language-action control with reproducible tooling, fine-tuning workflows, and benchmark competitiveness without a closed model dependency.
