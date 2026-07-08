# RT-1

`RT-1` here means **RT-1: Robotics Transformer 1** (Google / Everyday Robots / DeepMind), not an unrelated abbreviation like "runtime RT-1".  
This is the first large-scale real-world "robotics transformer" that combines language instructions + vision and predicts discretized actions as tokens in a closed-loop policy.

---

## Overview

RT-1 is an end-to-end imitation-learning policy for real robots. It uses language-conditioned vision input and outputs action tokens at runtime.  
The project page describes it as a way to test whether "large-capacity architectures + large, diverse, real-world data" can give robot models useful scaling behavior.

- Paper: `arXiv:2212.06817` (RT-1: Robotics Transformer for Real-World Control at Scale)
- Main sources: [RT-1 project page](https://robotics-transformer1.github.io/) and [arXiv paper](https://arxiv.org/abs/2212.06817)
- Scale reported publicly: over **130k demonstrations/episodes**, over **700 tasks**, **13 robots**, collected over **17 months**.
- Baseline comparison reported around **97% seen-task success** and **~76% unseen-task (zero-shot instruction) success** in their task set.

---

## Core idea

- Build a large dataset of robot trajectories with:
  - observations (multi-camera context),
  - natural-language task instruction,
  - associated actions.
- Encode input:
  - images via CNN backbone (EfficientNet in the original),
  - instruction embedding via text encoder,
  - fuse through FiLM-style conditioning,
  - compress with token-like spatial pooling.
- Transformer consumes the token stream and predicts the next discretized action token each step.
- Execute in closed loop at robot runtime.

Reported action dimensions are discretized for output and include:
- arm movement dimensions (`x, y, z, roll, pitch, yaw`) + gripper,
- base movement (`x, y, yaw`) + mode/termination signal.

RT-1 is closest to the "token prediction" framing when your action space is tokenized (or tokenized via a behavior/action tokenizer), but it adds significant multimodal encoder machinery.

---

## Fit for tokenized next-token work

For a token-first pipeline:
- It is a **good match** if your stack already uses tokenized actions and language-conditioned context.
- It is a **weak match** if your project is pure low-level state/action RL without language/vision multimodal fusion.

What makes it aligned with tokenized control:
- action is generated autoregressively as discrete tokens;
- sequence context includes recent observations and instructions;
- training is largely supervised teacher forcing.

What makes it less drop-in:
- strong coupling to vision-language preprocessing,
- closed-loop runtime constraints and robot-action grammar,
- nontrivial robot-specific tokenization requirements.

---

## Comparison chart

| Method | Core architecture | Input conditioning | Action representation | Data need | Zero-shot generalization | Real-robot fit |
|---|---|---|---|---|---|---|
| **RT-1** | vision-language + transformer backbone | image history + language instruction | discretized action tokens | very large real-robot trajectory corpus (130k+ episodes, 700+ tasks) | strong cross-task/generalization emphasis | strong; real-time closed-loop target explicitly |
| **Decision Transformer** | trajectory sequence model | `(return-to-go, state, action)` token window | tokenized or continuous action head | offline trajectories | task transfer limited by token grammar + dataset | medium; mostly value-replaced offline-policy behavior |
| **Trajectory Transformer** | sequence decoder planner | full trajectory chunks | action token/continuous depending on setup | offline trajectories, often large | good trajectory-level planning but still data-limited | medium; often research prototype-level deployment |
| **ACT / ALOHA** | chunked token decoder | observation history | chunked action tokens | demonstration trajectories | good for high-frequency skills with strong priors | practical for behavior cloning variants |
| **Diffusion Policy** | iterative denoising sequence model | observation/context conditioned | continuous action vectors (not pure one-step tokens) | curated demonstrations, often many demos | limited explicit zero-shot unless conditioned heavily | strong control smoothness; heavier inference |
| **OpenVLA** | open vla-style policy stack | vision-language conditioning | continuous + tokenized actions depending on implementation | RL/VLA datasets + fine-tuning | strong language-conditioned adaptation; community-focused | strong for research and adaptation |
| **RT-2** | vision-language-action foundation stack | vision + language + reasoning behavior | tokenized action space with broader semantic decoder behavior | internet-scale + robotics data | much larger multimodal transfer potential | heavier stack and cost than RT-1 |

---

## Pros

- Very concrete evidence that large, diverse real data can improve robot transfer.
- Clean closed-loop control framing for practical deployment.
- Strong "instructions + history" conditioning for multitask use.
- Useful baseline if your target is "one policy for many tasks" with token outputs.

## Cons

- Requires heavy multimodal data engineering (vision + language pipeline + action quantization).
- Real-time runtime overhead and latency constraints can dominate model design.
- Less suitable as a pure sequence-only drop-in if your project is only state-action tokens.
- Training infra and data requirements are high to match published behavior.

---

## Implementation notes for this knowledge base

- If your local work is already tokenizing robot observations/actions, keep RT-1 concepts as a full-stack baseline:
  - tokenize each of language instruction, visual context, and action command,
  - align token schedules across modalities,
  - decode a next action token sequence with safety checks.
- Start with a reduced action vocabulary to keep sampling stable.
- Keep step rate explicit (RT-1 reports low-hertz closed-loop operation around a few Hz in public figures).
- Add fail-safe policies around low confidence tokens.

Minimal pseudo-flow:

```text
for each timestep t:
  obs_tokens   = encode_visual_history(frames_t)
  goal_tokens  = encode_instruction(task_text)
  act_hist     = encode_prev_actions(token_history)
  input_seq    = [obs_tokens, goal_tokens, act_hist]
  action_t     = transformer_decoder_next_token(input_seq)
  apply_action(action_t)
```

---

## Why this belongs in your "NTP-style" list

- RT-1 is useful when you want to test:
  - tokenized action output quality,
  - instruction conditioning,
  - scaling behavior as you add robots/tasks without rewriting full RL loops.
- It is a strong baseline in the "tokenize + predict next action" family but not the only one.

---

## Related notes

- [[Decision Transformer]]
- [[Trajectory Transformer]]
- [[Behavior Transformer]]
- [[ACT Action Chunking Transformer]]
- [[FAST]]
- [[Diffusion Policy]]
- [[RT-2]]

## External references

- [RT-1 website](https://robotics-transformer1.github.io/)
- [RT-1 arXiv](https://arxiv.org/abs/2212.06817)
- [Google AI RT-1 release](https://ai.googleblog.com/2022/12/rt-1-robotics-transformer-for-real.html)
- [Google project overview via Hugging Face](https://huggingface.co/papers/2212.06817)

---

# Summary

RT-1 is a practical multimodal robotics transformer baseline where the policy is trained end-to-end from language + vision to tokenized robot actions. It is still highly useful for token-first robotics work, especially for real-time inference and instruction-conditioned behavior, but it is less of a pure "swap token head" change and more of a full stack.
