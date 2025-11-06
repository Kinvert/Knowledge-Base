# vectorbt

vectorbt is a Python-first vectorized backtesting and simulation framework designed to run extremely fast by leveraging NumPy, Numba, and pandas. It is highly suited for testing thousands to millions of parameter combinations, and is commonly used on crypto orderbook feeds like Hyperliquid (especially aggregated bar OHLCV) due to its performance characteristics.

---

## 🧠 Overview

vectorbt treats strategy signals, indicators, entries/exits as arrays and backtests them in parallel using vectorization. This eliminates Python-level loops and greatly increases speed compared to object/per-trade simulation engines like Zipline or Backtrader.

---

## 🔧 Core Concepts

- Everything is an array: entries, exits, signal, indicator, portfolio state
- High throughput parameter sweeps (think fast grid searches)
- Designed to stay in pandas and NumPy; no custom domain-specific language to learn
- Can integrate with external data stores and feeds (Hyperliquid, CCXT, CSVs, Parquet, etc)

---

## 📊 Comparison Chart

| Framework | Paradigm | Very Large Param Sweeps | Live Capable | Good for High Frequency | Difficulty |
|---|---|---|---|---|---|
| vectorbt | vectorized | ✅ best-in-class | ⚠️ possible but not main target | ✅ | medium |
| Backtrader | event-driven | ❌ | ✅ good | ⚠️ | low |
| Zipline | event-driven | ❌ | ⚠️ limited now | ⚠️ | medium |
| freqtrade | bot-first framework | ⚠️ | ✅ | ✅ | medium |
| pandas-ta + manual | DIY | ⚠️ | depends | depends | high |
| Qlib | factor + ML workflow | ✅ | ❌ mainly research | ⚠️ | high |

---

## 🎯 Use Cases

- hyperparameter sweeps over thousands or millions of combinations
- brute force indicator sweeps
- sanity checking strategies on multiple assets/timeframes
- post-hoc strategy analytics and decomposition
- statistical risk profiling

---

## ✅ Strengths

- one of the fastest ways to test an idea on historical data
- does not force an “object model” for strategies
- designed for pandas + NumPy engineers already doing research
- pairs extremely well with Jupyter and notebooks

---

## ❌ Weaknesses

- not a full bot framework
- vectorization can restrict some stateful strategies (you may need rolling windows or JIT custom logic)
- less “beginner friendly” than Backtrader

---

## 🔄 Related Concepts / Notes

- [[Hyperliquid]] (orderflow and data source context)
- [[Backtesting]] (general concept)
- [[Zipline]] (event-based)
- [[Backtrader]] (event-based)
- [[NumPy]] (foundation)
- [[pandas]] (foundation)
- [[Algorithmic Trading]]

---

## 🌐 External Resources

- https://github.com/polakowo/vectorbt
- https://vectorbt.dev

