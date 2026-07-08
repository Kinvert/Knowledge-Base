# StARformer

**StARformer** (`State-Action-Reward Transformer`) is a visual sequence-modeling architecture for RL from `Jinghuan Shang`, `Kumara Kahatapitiya`, `Xiang Li`, and `Michael S. Ryoo` (arXiv, 2021; ECCV 2022; IEEE TPAMI).  
It is designed for robot and vision-based RL where observations are images and the policy is learned by predicting next actions from a state-action-reward history.

---

## Overview

- Problem framing: sequence modeling over trajectories `state -> action -> reward`.
- Contribution: explicitly model short-step transition structure with a two-stage transformer stack, then model longer-horizon dependencies on top.
- Target setup: offline RL and imitation learning (behavioral cloning without online interaction).
- Key claim: improves long-horizon modeling versus plain Decision Transformer style attention, especially on image-based tasks.

---

## Core mechanism (what is actually new)

1. Extract **StAR-representations** per transition (within a local neighborhood in time), where each transition includes:
   - image state patch tokens,
   - action token,
   - reward token.
2. Use a **Step Transformer** to model this local transition structure and produce short-term transition features.
3. Feed these local features + full-sequence image features into a **Sequence Transformer** for long-range planning.
4. Train action token prediction in a next-token style objective.

This gives StARformer a stronger local inductive bias than `Decision Transformer`-style global attention over raw `(state, action, reward)` tokens.

---

## Where it lands in your “tokenizing observations and actions” pipeline

Use StARformer when:
- your observations are high-dimensional images,
- your current baseline struggles with long-horizon credit assignment,
- you want something closer to a robotics-oriented sequence model than fully multimodal VLA stacks.

Avoid it when:
- you mainly have structured low-dimensional state vectors and do not need a two-stage transformer stack,
- you need lightweight inference and minimal model overhead,
- you need native language grounding (then `RT-2`/`RT-2-X`/`OpenVLA` territory is more relevant).

---

## Comparison chart

| Method | Objective | Representation | Planning scope | Key strength vs StARformer | Typical use |
|---|---|---|---|---|---|
| **StARformer** | next-token action prediction over image-based transition sequences | image patches + action + reward tokens (`s_t`, `a_t`, `r_t`) + long-range fusion | medium-horizon with explicit local+global stages | explicit local transition modeling + long-range scaling; tuned for visual RL | Offline RL/imitation from image replay buffers |
| **Decision Transformer** | reward-conditioned trajectory modeling | encoded state/action tokens (+ optional return) | offline rollouts / return-conditioned generation | simpler stack; strong baseline and widely reused | baseline for trajectory sequence modeling |
| **Trajectory Transformer** | autoregressive trajectory generation | state/action/reward token trajectories | explicit planning over future tokens, often with search at inference | stronger horizon planning control; simpler decomposition than StARformer | offline RL when planning behavior over fixed horizons |
| **Behavior Transformer / VQ-BeT** | behavior cloning token prediction | state tokens + quantized action tokens | short-to-medium horizon; behavior mode coverage via VQ | high-quality action quantization for continuous actions | imitation-first tasks, no explicit return/reward token stream |
| **ACT / ALOHA** | chunked action prediction + overlap-replan | latent action chunks from teleop datasets | local receding-horizon control loops | robust real-robot execution behavior with chunk-level runtime | manipulation teleop and sim-to-real imitation |
| **FAST** | action tokenizer + sequence head swap | frequency/latent-aware action tokenization | compatible with transformer decoders | practical tokenization upgrade to improve sample efficiency | if your bottleneck is raw continuous-action token coding |
| **RT-2** | vision-language-action sequence policy | image + language + action tokens | long-horizon language-conditioned behavior | larger semantic grounding stack beyond pure control sequence model | manipulation + language condition |

---

## Why it is not a drop-in next-token refactor

- StARformer changes both feature front-end (state patching) and model internals (Step + Sequence Transformer stack).
- It is more than swapping a decoder/loss; it redefines transition representation.
- Good fit for your direction, but integration cost is medium/high if your existing code is pure `s,a,r` token arrays.

---

## Practical notes from the public implementation

The authors provide a reference PyTorch implementation with both `atari` and `DMC` scripts.

- Repo: `https://github.com/elicassion/StARformer`
- The README exposes:
  - conda env setup,
  - dataset loading paths,
  - three run modes via `--model_type` (`star`, `star_rwd`, `star_fusion`, `star_stack`),
  - GPU memory vs `seq_len` benchmarks and epoch time references.

Minimal command pattern from the README:

```bash
conda env create -f atari_and_dmc/conda_env.yml
conda activate starformer

python run_star_atari.py --seed 123 \
  --data_dir_prefix /path/to/atari_dataset \
  --epochs 10 --num_steps 500000 --num_buffers 50 \
  --batch_size 64 --seq_len 30 --model_type star_rwd \
  --game Breakout
```

```bash
python run_star_dmc.py --seed 123 \
  --data_dir_prefix /path/to/dmc_dataset \
  --epochs 10 --seq_len 30 --model_type star_rwd \
  --batch_size 64 --domain ball_in_cup --task catch \
  --lr 1e-4
```

For a quick migration checklist:

- Keep your trajectory in consistent `(s_t, a_t, r_t)` triplets.
- Preserve temporal alignment of reward and action with the corresponding observation.
- Start with `model_type=star` (imitation) on a small dataset before trying `star_rwd` with offline RL.
- Compare against `Decision Transformer` with identical replay windows to isolate the Step/Sequence effect.

---

## Pros

- Clear explicit inductive bias for local transition structure.
- Better long-sequence utilization reported than plain decision-transformer style attention.
- Strong fit for vision-based RL and behavior cloning.
- Works as a practical template when you want more than naive token flattening.

## Cons

- Higher implementation complexity than a direct token-decoding replacement.
- More memory-hungry than baseline DT due to patch tokens + dual transformer stages.
- Heavier engineering burden for non-image modalities unless you create compatible state embeddings.

---

## Open questions to track

- What is the best `Step Transformer` window for your own env when `seq_len` grows?  
  (window width can materially affect tradeoff between locality bias and responsiveness)
- Does reward token removal still work cleanly for your implicit-reward or sparse-reward tasks?
- Can token compression methods (e.g., FAST-style action compression) be added while preserving StAR-local attentions?
- Is a hybrid with return-conditioning useful for sparse reward planning?

---

## External resources

- ArXiv: https://arxiv.org/abs/2110.06206  
- TPAMI DOI: https://doi.org/10.1109/TPAMI.2022.3204708  
- Repository: https://github.com/elicassion/StARformer  
- Paper landing on GitHub README: https://github.com/elicassion/StARformer#readme

---

## Related notes

- [[Decision Transformer]]
- [[Trajectory Transformer]]
- [[Behavior Transformer]]
- [[ACT Action Chunking Transformer]]
- [[FAST]]
- [[RT-2]]
- [[OpenVLA]]
- [[RT-1]]
