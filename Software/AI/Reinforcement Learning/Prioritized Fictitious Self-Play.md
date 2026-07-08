# PFSP (Prioritized Fictitious Self-Play)

**PFSP** usually means **Prioritized Fictitious Self-Play**. It is a tournament-style opponent-sampling strategy for multi-agent or zero-sum RL training where past versions of agents are kept in a pool and sampled with a non-uniform probability.

The idea is to avoid wasting self-play time against agents you already crush, while not ignoring strong counter-strategies that can expose exploitable behavior.

---

## 🧠 What PFSP is

PFSP is a refinement of regular fictitious self-play.

- **Self-play (SP)**: one policy learns mostly against itself.
- **Fictitious self-play (FSP)**: agent cycles through a uniform sample of historical policies.
- **Prioritized Fictitious Self-Play (PFSP)**: sample historical opponents by a function of recent win rates, usually biased toward harder opponents.
- **Practical note**: PFSP is used as a *training curriculum mechanism*, not a network architecture.

PFSP is most associated with AlphaStar’s league training work (Nature 2019), where it was used alongside exploiters and mixed training roles.

---

## 🎯 Why PFSP exists

Vanilla SP tends to cycle in non-transitive games because an agent can overfit to one narrow strategy and then get exploited by a previously weak response.

FSP fixes this partially by including historical checkpoints, but it can still spend samples on opponents that are now too weak.

PFSP solves this by increasing sampling pressure on opponents with high strategic value.

- It reduces compute wasted on obsolete easy opponents.
- It increases pressure on the current weak points of the learning policy.
- It is usually easier to combine with a larger league design.

---

## ⚙️ PFSP mechanics

In the common AlphaStar-style setup, for current agent **A**:

1. Maintain candidate pool **C** of frozen historical opponents.
2. Estimate win-rate for each opponent `x` in `[0,1]` versus **A**.
3. Transform each `x` with a priority function `f(x)`.
4. Sample opponent `B` proportionally:

```text
P(B) = f(win_rate(A,B)) / Σ_{o in C} f(win_rate(A,o))
```

Typical `f` forms:

- `fhard(x) = (1 - x)^p` : emphasizes opponents that beat A more often.
- `fvar(x) = x(1-x)` : emphasizes opponents close to the current skill band.

`p` is usually tuned to control entropy:

- small `p` -> softer prioritization
- large `p` -> stronger focus on hardest opponents

AlphaStar-style references use a league with several agent roles (main agent, main exploiters, league exploiters), not only PFSP alone.

---

## 🧪 Is PFSP in PufferLib?

Not as a named first-class PFSP API in the public PufferLib docs.

- PufferLib docs describe a high-performance backend and replay/training optimizations.
- They do not define a built-in `PFSPOpponentSampler` or `league` API in the public cheat-sheet.
- People commonly run PFSP externally or in custom training harness code.

So when someone says PFSP in PufferLib Discord, it is likely:

- either shorthand for an external curriculum idea adapted into a custom environment/runner, or
- mixing terminology with prioritized replay and league-like scheduling logic.

---

## 📚 Terminology and implementation references

- **DFSP / pFSP terminology**: some projects use lowercase `pFSP`.
- **Historical opponent pool**: checkpointed actor policies saved over time.
- **Payoff matrix**: empirical table of win-rates between policies.
- **League training**: multi-role setup used in advanced training for diversity and exploitation pressure.
- **Exploiters**: agents specialized for exposing weak spots in the main policy.

In open-source practice, DI-engine implementations expose `_pfsp_branch` methods that select historical players using `pfsp(win_rates, weighting=...)`, which is a concrete reference point for how PFSP is often encoded.

---

## 🔍 PFSP comparison

| Method | Opponent Source | Sampling Strategy | Core Advantage | Common Failure Mode |
|---|---|---|---|---|
| Self-play | Current policy | current only | simple | cyclic behaviors, forgetting |
| FSP | history of own policy | uniform over population | stable against collapse | wastes samples on weak history |
| PFSP | history + optional exploiters | weighted by win-rate priority | faster curriculum pressure on weak points | over-prioritization can create brittle oscillation |
| NFSP | average policy vs best response (RL + SL mix) | fictitious-average + RL buffer split | converges better in imperfect-information setups | heavier algorithm complexity |
| PSRO / league style | explicit policy generator + selector | game-theoretic population updates | strong strategic coverage in games | expensive, high plumbing overhead |
| Prioritized Replay (buffer) | transition data | priority by TD error or similar | sample efficiency in value-learning | not opponent-sampling; often confused with PFSP |

---

## ✅ PFSP strengths

- Finds exploiters earlier than uniform FSP in many non-transitive games.
- Preserves historical coverage while emphasizing current strategic gaps.
- Compatible with PPO variants used in many RL stacks.

## ⚠️ PFSP weak points

- Needs robust opponent evaluation infrastructure.
- Win-rate noise can destabilize sampling if windows are too short.
- Can overfit to sparse high-priority opponents without periodic diversity checks.

---

## 🛠️ Minimal PFSP flow (framework-agnostic)

```text
policy_pool = []  # frozen checkpoints with metadata
payoff = {}       # pairwise historical win-rates

for each training_epoch:
    evaluate current policy vs sampled pool slice
    update payoff[current, opponent]
    weights = priority_fn(payoff[current, :])
    opponent = sample(pool, weights)
    run rollout against opponent
    train on collected data
    every N updates:
        save snapshot into pool
        prune old entries if pool exceeds limit
```

Use this with your existing logger and environment adapters. In a PufferLib setup, you typically add this scheduler outside the core `puffer` API and feed selected opponent policies back into your league/eval harness.

---

## 🧭 For your PufferLib context

If you want PFSP behavior while staying mostly in PufferLib:

- Keep PFSP scheduling outside the environment binding layer.
- Snapshot policies at fixed step intervals.
- Maintain a lightweight evaluator that computes a rolling win-rate matrix.
- Apply a bounded weighting function to avoid mode collapse to one opponent.
- Periodically force low-probability opponent samples to recover diversity.

This makes PFSP more like an external curriculum than a built-in library feature, which is usually easier to reason about and debug.

---

## 🔗 Related concepts and notes

- [[Reinforcement Learning]]
- [[PufferLib]]
- [[PPO]]
- [[Actor Critic]]
- [[MARL]]
- [[Prioritized Replay]]

---

## 📚 External resources

- [AlphaStar: Grandmaster level in StarCraft II using multi-agent reinforcement learning (Nature)](https://www.nature.com/articles/s41586-019-1724-z)
- [AlphaStar blog summary (Google DeepMind)](https://deepmind.google/blog/alphastar-grandmaster-level-in-starcraft-ii-using-multi-agent-reinforcement-learning/)
- [DI-engine PFSP branch example (`_pfsp_branch`)](https://www.aidoczh.com/di-engine/_modules/ding/league/player.html)
- [PufferLib docs](https://puffer.ai/docs.html)
- [Syllabus PFSP references and curriculum context](https://ryannavillus.github.io/Syllabus/implemented-automatic-curriculum-learning-methods/implemented-curricula.html)
