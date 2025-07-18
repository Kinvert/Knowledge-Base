# PufferLib Cheatsheet 🧠

`PufferLib` is a lightweight Python library built to simplify the use of reinforcement learning (RL) environments, especially with Gymnasium and other standardized APIs. It supports vectorization, wrappers, and a consistent interface across many environments, including PettingZoo, IsaacGym, and DeepMind Control Suite.

---

## 🚀 Install PufferLib

Install the latest version via pip:

`pip install pufferlib`
or
`uv pip install pufferlib`

---

## Editable

`uv pip install -e .`

`python -m pufferlib.pufferl train puffer_breakout --vec.num-workers 4`

---

## 🔧 Create and Wrap Environment

Wrap a standard Gymnasium env:

`import pufferlib; env = pufferlib.make("CartPole-v1")`

Use vectorized envs:

`env = pufferlib.vector.make("CartPole-v1", num_envs=8)`

Create a custom wrapper:

`env = pufferlib.wrappers.WrapFunction(env, your_wrapper_function)`

---

## 🔄 Vectorization

Single-line vectorized env creation:

`env = pufferlib.vector.make("LunarLander-v2", num_envs=4, backend="gym")`

Supported backends:

- `gym`
- `pettingzoo`
- `dmc`
- `isaac`

---

## 🧠 Multi-Agent Support

Wrap PettingZoo envs easily:

`env = pufferlib.make("mpe/simple_spread_v2")`

PufferLib auto-detects and handles multi-agent logic where supported.

---

## 🧪 Check Env Info

Get obs/action space:

`env.observation_space`  
`env.action_space`

Check sample:

`env.reset()`  
`env.step(env.action_space.sample())`

---

## 🛠️ Debugging Tools

Inspect wrappers:

`pufferlib.debug.inspect_wrappers(env)`

Test compatibility:

`pufferlib.test.run_test_suite("CartPole-v1")`

---

## 🔌 Integrations

- Works well with Stable-Baselines3
- Supports Gymnasium and classic OpenAI Gym APIs
- Compatible with PettingZoo for multi-agent setups
- Plays nice with JAX/torch-based backends like Isaac Gym and DeepMind Control Suite

---

## ⚙️ Developer Tips

Use string env names or EnvCreators (for custom logic):

`pufferlib.make("CartPole-v1")`  
`pufferlib.make(CustomEnvCreator())`

Use `pufferlib.namespace` to group config/data per agent or system

---

## 🧩 Related Notes

- [[PufferLib]]

---
