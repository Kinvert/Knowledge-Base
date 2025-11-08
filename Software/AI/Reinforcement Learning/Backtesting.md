# Backtesting

Backtesting is the act of taking a strategy, model, or policy and running it through *historical* data to estimate how it would have performed. In crypto (ex. [[Hyperliquid]]), this is usually candle or tick level Market Replay. In general engineering contexts, backtesting is used in any domain where you want to simulate decisions over a historical time series.

---

## 🧠 Overview

Backtesting ≈ “hypothetical run of your algorithm in the past”.  
The core assumption: history gives insight into future performance.  
The core warning: history rarely maps 1:1 to the future.

---

## 🧩 Core Concepts

- “Market Replay Engine”: load historical OHLCV, L2, or L4 book state and walk forward
- “Reference Frame”: at each timestep you show the algo only what would have been known at that moment
- “No Leakage”: absolutely avoid giving future information to the algorithm
- “Slippage” and “Fees” must be modeled

---

## 🔁 Comparison Chart

| Thing | Purpose | Data Used | Determinism | Good For RL? |
|---|---|---|---|---|
| Backtesting | historical playback | historical | High | Yes |
| Paper trading | live but fake money | live feed | Medium | Limited |
| Forward testing | small real capital | live feed | Low | Good final eval |
| Simulation (synthetic) | invented/modelled worlds | synthetic | Arbitrary | Excellent for theory |
| Monte Carlo Resampling | probabilistic replays | mixed | Medium | Yes, if scenario-based |

---

## 🧪 Use Cases

- Trading system dev on [[Hyperliquid]] data
- Testing grid bots, trend followers, market makers
- Weather models (forecasting storms and evaluating “if this model existed last year, how would it have predicted these events?”)
- Power grid decision policies: run past load profiles + past storms
- Robot fleet scheduling against old traffic or sensor logs
- Supply chain simulations on old SKU flows
- Epidemiology models replaying pandemic curves

---

## ✅ Strengths

- inexpensive
- deterministic
- repeatable
- good for comparing variations of hyperparameters

---

## ❌ Weaknesses

- past ≠ future
- regime shifts destroy predictive utility
- often teaches strategies that only work in that dataset

---

## 🧵 Related Concepts

- [[Market Replay]]
- [[Market Replay Engine]]
- [[Hyperliquid]]
- [[OHLCV Data]]
- [[Time Series Forecasting]]
- [[vectorbt]]
- [[Reinforcement Learning]] (Reinforcement Learning)

---

## 🔗 External Resources

- Hyperliquid API docs
- vectorbt
- zipline
