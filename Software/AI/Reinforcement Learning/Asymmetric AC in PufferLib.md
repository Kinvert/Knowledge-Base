---
title: Asymmetric AC in PufferLib
aliases:
  - Asymmetric Actor-Critic in PufferLib
  - PufferLib Asymmetric Actor-Critic
  - Asymmetric Actor-Critic PufferLib
tags:
  - reinforcement-learning
  - pufferlib
  - actor-critic
  - sim-to-real
  - robotics
---

# Asymmetric AC in PufferLib

This is the shortest safe way to implement asymmetric actor-critic training in this stack: keep actor inputs deployment-safe, but let the critic read a wider privileged state during training.

The note is concrete so you can apply it directly in a PufferLib project.

---

## Why PufferLib

`PufferLib` is useful here because the rollout loop is explicit and centralized.

The training flow in this repo shows:

- vectorized env creation in `pufferlib.vector.make`
- a shared `PuffeRL` train loop
- policy call style `self.policy(mb_obs, state)` returning `(logits, newvalue)`

That means you can add asymmetry inside one policy and one env contract without rebuilding a custom trainer.

---

## Required split

You need three fixed pieces.

1. `o_actor`: deployable observation for the actor policy.
2. `o_priv`: privileged training-only channels (`x`,`v`, contacts, hidden state, etc.).
3. `o_train`: concat that the critic reads.

Define this contract:

- `obs_dim_actor = len(o_actor)`
- `obs_dim_priv = len(o_priv)`
- `obs_dim_train = obs_dim_actor + obs_dim_priv`

At deployment, the `policy` must never see `o_priv`.

---

## Step 1 — Env contract in practice

Expose a single observation that still contains privileged channels for training, but keep a selector for actor-only slices.

If your trainer/policy expects flat tensors only, keep the wrapper as a local adapter that only returns slices and concatenated arrays; avoid dict outputs in that case.

```python
import gymnasium as gym
import numpy as np
from gymnasium import spaces

class AsymmetricObsWrapper(gym.Wrapper):
    def __init__(self, env, actor_slice):
        super().__init__(env)
        self.actor_slice = actor_slice
        obs_space = env.observation_space
        # PufferLib C99/Gym envs usually expose Box here.
        self._actor_space = spaces.Box(
            low=np.array(obs_space.low)[actor_slice],
            high=np.array(obs_space.high)[actor_slice],
            shape=(actor_slice.stop - actor_slice.start,),
            dtype=np.float32,
        )
        self.observation_space = spaces.Dict({
            "actor": self._actor_space,
            "train": obs_space,
        })

    def observation(self, observation):
        return {
            "actor": np.asarray(observation, dtype=np.float32)[self.actor_slice],
            "train": np.asarray(observation, dtype=np.float32),
        }
```

In this style, rollout code can choose whether it uses `["actor"]` or `["train"]`.

`pufferlib.wrappers.WrapFunction` is available if you prefer functional wrapping, but this class wrapper keeps indexing explicit.

---

## Step 2 — Policy split (real code change)

Keep the PPO/PufferLib forward contract and split inside the model.

```python
import torch
import torch.nn as nn

class AsymActorCritic(nn.Module):
    def __init__(self, actor_obs_dim, train_obs_dim, action_dim, hidden=256):
        super().__init__()
        self.actor_obs_dim = actor_obs_dim

        self.actor_body = nn.Sequential(
            nn.Linear(actor_obs_dim, hidden),
            nn.ReLU(),
            nn.Linear(hidden, hidden),
            nn.ReLU(),
        )
        self.critic_body = nn.Sequential(
            nn.Linear(train_obs_dim, hidden),
            nn.ReLU(),
            nn.Linear(hidden, hidden),
            nn.ReLU(),
        )
        self.actor_head = nn.Linear(hidden, action_dim)
        self.value_head = nn.Linear(hidden, 1)

    def forward(self, obs, state=None):
        # obs can be dict from wrapper or flat tensor from older envs
        if isinstance(obs, dict):
            actor_x = obs["actor"]
            critic_x = obs["train"]
        else:
            actor_x = obs[..., : self.actor_obs_dim]
            critic_x = obs

        logits = self.actor_head(self.actor_body(actor_x))
        values = self.value_head(self.critic_body(critic_x)).squeeze(-1)
        return logits, values

    def act(self, actor_obs):
        logits = self.actor_head(self.actor_body(actor_obs))
        return torch.distributions.Categorical(logits=logits).sample()
```

This keeps `self.policy(mb_obs, state)` compatible with the PufferLib trainer and still exposes a deploy-safe actor-only entrypoint.

---

## Step 3 — Deploy path (truncate privileged part)

Create a hard boundary at inference time.

```python
def run_deploy_step(policy, obs_from_hw, actor_obs_dim):
    # obs_from_hw is strictly deployment sensor input
    actor_obs = torch.as_tensor(obs_from_hw, device=policy.device)[:, :actor_obs_dim]
    return policy.act(actor_obs)
```

If your environment returns full vectors in production, physically slice to `actor_obs_dim` before every policy call.

If `obs_from_hw` is not fixed-width, build a small adapter from your middleware (ROS, socket frame, shared memory) that outputs exactly actor obs.

---

## Minimal config/launch checklist

Use this order when integrating with `pufferlib.pufferl`:

1. define actor/privileged indices in one config file.
2. wrap training env so it returns privileged-aware observations.
3. instantiate `AsymActorCritic(actor_obs_dim=..., train_obs_dim=..., action_dim=...)`.
4. keep PufferLib train entry as-is (it still expects `(logits, value)` from policy).
5. log actor-only rollout eval with an adapter that truncates to actor channels.

Existing CLI example in this vault:

`python -m pufferlib.pufferl train my_env --vec.num-workers 4`

If your parser accepts dotted CLI overrides, keep to the pattern and add your own namespace flags for `actor_obs_dim` and `train_obs_dim`.

---

## What to watch for

1. Never let privileged channels leak into evaluation policy inputs.
2. Add a strict deployment check: actor obs shape is fixed, privileged channels are absent, and reward/rollout success is measured from hardware-visible signals only.
3. Keep value-loss clipping and policy-loss logic unchanged until asymmetry wiring is verified.
4. Run A/B with identical scripts: symmetric actor-critic, asymmetric split, and deployment-only actor inference.

---

## Comparison table

| Setup | Actor input | Critic input | Deployment behavior | Main risk |
|---|---|---|---|---|
| Symmetric AC | Full obs | Full obs | Same as train | Leaks simulator assumptions |
| Dictionary split policy | `obs["actor"]` | `obs["train"]` | Dict-aware evaluator needed | Wrapper/integration bugs |
| Flat split policy | actor prefix + full vector | same flat vector | Truncate at runtime | Slice mismatch in obs format |
| Two-network asymmetry | Two models | Full obs | Train/infer model swap | Higher code churn |
| Teacher-Student distillation | deployable obs only | optional teacher has more state | Fully actor-safe | Distillation tuning overhead |
| Distillation in PufferLib | deployable obs + fixed student | optional external student training | Clean deploy graph | Extra pipeline step |

---

## Relation map

- [[Asymmetric Actor-Critic]]
- [[PufferLib]]
- [[PufferLib C99 Environment Authoring]]
- [[Privileged Information]]
- [[Teacher Student Distillation]]
- [[State Representation Learning]]

---

## Sources and references

- PufferLib docs: https://puffer.ai/docs.html
- PufferLib repository: https://github.com/jxhe/pufferlib
- Isaac/robotics observation-splitting conventions in task authoring workflows in this vault
