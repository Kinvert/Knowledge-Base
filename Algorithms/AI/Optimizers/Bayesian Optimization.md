# Bayesian Optimization 🧪

Bayesian Optimization is a sample-efficient strategy for optimizing expensive black-box functions. It builds and updates a surrogate model of the objective, then uses an acquisition function to pick the next experiment. It is most useful when evaluations are expensive, noisy, and derivative-free, such as simulation design sweeps or hardware calibration.

---

## 🧠 Overview
- **Goal:** find near-optimal input parameters with the fewest objective evaluations.
- **Core mechanism:** combine:
  - a probabilistic surrogate model (often a Gaussian process),
  - an acquisition rule balancing exploitation and exploration.
- **Workflow:** evaluate points → fit/update surrogate → score candidates with acquisition → run new evaluation → repeat.
- In this note, the term `BO` usually refers to Bayesian Optimization in this standard setup.

---

## ⚙️ Core Concepts
- **Surrogate model**: cheap predictive model for objective value, often returns mean and uncertainty.
- **Acquisition function**: scoring rule for which point to evaluate next; common examples are Expected Improvement (EI), Upper Confidence Bound (UCB), and Probability of Improvement (PI).
- **Exploration-exploitation tradeoff**: choose points that are promising, uncertain, or both.
- **Noise handling**: BO can include observation noise by modeling output variance and using repeated evaluations or robust acquisition settings.
- **Budget-aware optimization**: BO is explicitly constrained by evaluation count and compute cost.
- **Input bounds / priors**: practical BO starts with sane domains for every parameter.

---

## 📊 Comparison Chart

| Method | Uses Function Model | Uses Uncertainty | Sample Efficiency | Strength | Weakness |
|--------|--------------------|------------------|------------------|----------|----------|
| Bayesian Optimization | Yes (surrogate) | Yes | High | Strong on expensive black-box objectives | Per-step overhead is heavier |
| Random Search | No | No | Low | Fast to implement | Wastes budget on uninformative regions |
| Grid Search | No | No | Very Low | Deterministic, easy to reproduce | Exponential blowup with dimension |
| CMA-ES | No | No explicit posterior | Medium | Good for rugged landscapes | Needs more evaluations than BO in low-data regimes |
| Evolutionary Strategies | No | No explicit | Medium | Handles large, messy search spaces | Weaker convergence control |
| Hyperband/BOHB | Partial | Indirect | High in early stopping settings | Great for expensive ML training | Needs staged budgets / early stopping setup |

---

## 🚀 Practical Settings
- Start with a low-dimensional search first (often `< 20` active dimensions).
- Use space-filling initial points (Latin hypercube or random) before exploitation.
- Prefer `EI` or `UCB` as first acquisition objective; move to risk-sensitive options for noisy objectives.
- Use a strict total evaluation budget, e.g., 50–200 evaluations for moderate simulator tuning.
- Log every trial result (`seed`, params, raw score, runtime, constraint violations) for reproducibility.

Recommended baseline defaults:
- `n_initial_points` around 8–20.
- `n_calls` driven by budget and simulator cost.
- Explicit bounds + types (`float`, `int`, `categorical`) to avoid invalid samples.

---

## 🛠️ Code Snippets

### Python with Optuna
```python
import optuna

def objective(trial):
    lr = trial.suggest_float("lr", 1e-5, 1e-2, log=True)
    batch_size = trial.suggest_int("batch_size", 16, 128, step=16)
    gamma = trial.suggest_float("gamma", 0.9, 0.999)

    # Run your expensive eval here:
    # return validation metric to minimize or maximize (negate if needed)
    score = run_experiment(lr, batch_size, gamma)
    return score

sampler = optuna.samplers.TPESampler(seed=42)  # BO-style tree-structured Parzen estimator
study = optuna.create_study(direction="minimize", sampler=sampler)
study.optimize(objective, n_trials=80)
print(study.best_params)
print(study.best_value)
```

### Python with gp_minimize (scikit-optimize)
```python
from skopt import gp_minimize
from skopt.space import Real, Integer

space = [
    Real(1e-5, 1e-2, prior="log-uniform", name="lr"),
    Integer(16, 128, name="batch_size"),
]

res = gp_minimize(
    func=run_evaluator,
    dimensions=space,
    n_calls=60,
    n_initial_points=12,
    acq_func="EI",
    random_state=42,
)

print(res.x, res.fun)
```

### BO in distributed sweep workflows
- In multi-GPU settings, coordinate trials with a central trial DB and worker nodes.
- Keep objective deterministic per trial (or store seeds) so reruns are comparable.
- If training is expensive, checkpoint and resume candidate evaluations.

---

## ✅ Strengths
- Very effective when each objective call is expensive.
- Naturally balances exploring unknown regions and exploiting good regions.
- Works with noisy measurements when modeled correctly.
- Can optimize expensive simulations and physical experiments better than blind search.

---

## ❌ Weaknesses
- Can underperform in high dimensions without careful feature engineering.
- Per-iteration compute cost can become significant.
- Strongly dependent on prior bounds and kernel/likelihood choices.
- Harder to debug than simple grid/random search.

---

## 🧩 Relation to Reinforcement Learning
Some workflows in `[[Reinforcement Learning]]` and robotics use BO to tune policy/simulation hyperparameters before policy training. That is different from policy optimization itself:
- **BO:** optimize a black-box score with limited compute budget.
- **RL:** optimize policy behavior over trajectories and environment interaction.

So BO can be a strong outer loop (e.g., simulator parameter tuning), while RL is typically an inner loop.

---

## 📚 Related Concepts / Notes
- [[Gradient Based Optimization]]
- [[Bayes Filter]] (probabilistic estimation background for uncertainty reasoning)
- [[wandb]] (tracking sweeps and experiment metadata)
- [[CARBS]]
- [[tpot]]
- [[System Identification for Sim2Real|System Identification in Sim2Real]]
- [[Hyperparameter Tuning|Hyperparameter Optimization]]

---

## 🔗 External Resources
- GP-Bandit and BO primers: `https://bayesian-optimization.github.io`
- Optuna docs: `https://optuna.readthedocs.io`
- scikit-optimize docs: `https://scikit-optimize.github.io/stable/`
- BoTorch docs: `https://botorch.org`
- BoTorch and Ax tutorials: `https://ax.dev`

---

## 🎯 Key Takeaway
Bayesian Optimization is most valuable when each experiment is costly and you only have a strict compute budget. The model then decides where to try next, which is usually much better than blind search under those constraints.

