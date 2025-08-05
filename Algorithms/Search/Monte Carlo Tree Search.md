# Monte Carlo Tree Search

Monte Carlo Tree Search (MCTS) is a heuristic search algorithm used for decision-making in domains with large or infinite state spaces. It is especially powerful in game-playing AI and robotic planning tasks due to its ability to balance exploration and exploitation without requiring a complete model of the environment.

MCTS builds a search tree incrementally and uses random simulations to estimate the value of actions, making it suitable for scenarios where classical search techniques are computationally infeasible.

---

## 🧠 Overview

MCTS incrementally builds a search tree using the results of simulated playouts (Monte Carlo simulations). It balances:
- **Exploration** (trying less-visited nodes to discover new strategies)
- **Exploitation** (favoring nodes with high average rewards)

The algorithm cycles through four main phases: **Selection**, **Expansion**, **Simulation**, and **Backpropagation**.

---

## 📘 Core Concepts

- **Selection:** Traverse the tree using a selection policy like [[UCT]] (Upper Confidence Bound applied to Trees).
- **Expansion:** Add new child nodes to the tree.
- **Simulation:** Run a rollout (random or heuristic) to estimate the outcome from the new node.
- **Backpropagation:** Update the node statistics based on the result of the simulation.

---

## 🛠️ Key Features

- Works well without a full model of the environment
- Can handle vast or continuous action spaces
- Adaptive to the time available for computation
- Asymptotically converges to the optimal decision with more simulations

---

## 📊 Comparison Chart

| Algorithm                    | Model-Based | Handles Large Spaces | Domain Knowledge Needed | Usage                      |
|-----------------------------|-------------|------------------------|--------------------------|----------------------------|
| MCTS                        | Optional    | ✅                     | ❌                       | Robotics, Games, Planning  |
| [[Minimax]]                 | ✅           | ❌                     | ✅                       | Chess, Go (small state)    |
| [[A*]]                      | ✅           | ❌                     | ✅                       | Pathfinding                |
| [[DQN]] (Deep Q Network)    | ❌           | ✅                     | ✅ (network structure)   | Continuous control         |
| [[RRT]] (Rapidly-exploring Random Tree) | ❌ | ✅               | ❌                       | Robotic path planning      |

---

## ✅ Strengths

- Does not require a full transition model
- Flexible and general-purpose
- Scales well with more computation time
- Can incorporate domain-specific heuristics to improve performance

---

## ⚠️ Weaknesses

- Slower convergence than supervised or deep RL methods in some tasks
- May require many simulations to find good actions
- Performance heavily depends on the rollout policy
- Memory usage can grow significantly

---

## 🧪 Use Cases

- Game AI (e.g., Go, Chess, Shogi)
- Robot decision-making under uncertainty
- Planning in complex or dynamic environments
- [[Autonomous Vehicles]] for maneuver planning
- [[SLAM]] (for decision-making layers)

---

## 🔗 Related Concepts

- [[UCT]] (Upper Confidence Trees)
- [[Markov Decision Process]] (MDP)
- [[Reinforcement Learning]]
- [[RRT]] (Rapidly-exploring Random Tree)
- [[Minimax]]
- [[Planning under Uncertainty]]

---

## 🔧 Compatible Items

- [[OpenAI Gym]] / [[Gymnasium]]
- [[ROS]] (Robot Operating System)
- Custom environments with tree-like decision processes
- Board game engines or simulators

---

## 🌐 External Resources

- [Wikipedia: Monte Carlo Tree Search](https://en.wikipedia.org/wiki/Monte_Carlo_tree_search)
- [A Survey of Monte Carlo Tree Search Methods](https://arxiv.org/abs/2012.05757)
- [AlphaGo: Mastering the game of Go with deep neural networks and tree search](https://www.nature.com/articles/nature16961)

---

## 📚 Further Reading

- Browne et al. – *A Survey of MCTS Methods*
- Sutton & Barto – *Reinforcement Learning: An Introduction*
- Silver et al. – *Mastering the game of Go with deep neural networks and tree search*

---
