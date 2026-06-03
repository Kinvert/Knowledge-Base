# Box2D

**Box2D** is a 2D rigid-body physics engine. In reinforcement learning, the term usually appears in two easily-confused ways:

- **Box2D environments**: Gym/Gymnasium benchmark environments powered by the Box2D physics engine, such as `LunarLander`, `BipedalWalker`, and `CarRacing`.
- **`Box` spaces**: Gym/Gymnasium action or observation layouts for numeric arrays. This is not the physics engine.

If someone was talking about "different action or observation layouts," they were probably talking about **Gym spaces** like `Box`, `Discrete`, `MultiDiscrete`, `Dict`, or `Tuple`, not Box2D itself.

---

## Mental Model

Box2D physics:

```text
2D bodies + joints + collisions + forces -> simulated world state
```

Gymnasium environment:

```text
agent action -> env.step(action) -> Box2D physics step -> observation, reward, done, info
```

Gym spaces:

```text
action_space      = what shape/type action the policy must output
observation_space = what shape/type observation the policy receives
```

---

## Gymnasium Box2D Environments

Gymnasium groups three classic environments under **Box2D**:

| Environment | Action Space | Observation Space | Notes |
|---|---|---|---|
| `LunarLander-v3` | `Discrete(4)` by default | `Box(..., shape=(8,), dtype=float32)` | Rocket landing with position, velocity, angle, angular velocity, leg contacts. |
| `LunarLander-v3, continuous=True` | `Box(-1, +1, shape=(2,), dtype=float32)` | `Box(..., shape=(8,), dtype=float32)` | Continuous main/lateral engine throttle. |
| `BipedalWalker-v3` | `Box(-1, +1, shape=(4,), dtype=float32)` | `Box(..., shape=(24,), dtype=float32)` | Four motor speed controls; proprioception plus lidar-like readings. |
| `CarRacing-v3` | `Box([-1, 0, 0], 1, shape=(3,), dtype=float32)` | `Box(0, 255, shape=(96, 96, 3), dtype=uint8)` | Pixel observation; actions are steering, gas, brake. |
| `CarRacing-v3, continuous=False` | `Discrete(5)` | `Box(0, 255, shape=(96, 96, 3), dtype=uint8)` | Discrete no-op/right/left/gas/brake controls. |

Install:

```bash
pip install swig
pip install "gymnasium[box2d]"
```

Basic use:

```python
import gymnasium as gym

env = gym.make("LunarLander-v3")
obs, info = env.reset()

action = env.action_space.sample()
obs, reward, terminated, truncated, info = env.step(action)
done = terminated or truncated
```

---

## Box2D vs `Box`

The naming collision is the main trap.

**Box2D**:

- A physics engine.
- Simulates rigid bodies, collisions, joints, forces, friction, contacts, and sleeping bodies.
- Used internally by some Gymnasium environments.

**`gymnasium.spaces.Box`**:

- A mathematical description of a bounded numeric tensor.
- Used for continuous controls, vector observations, image observations, and bounded integer arrays.
- Has `low`, `high`, `shape`, and `dtype`.

Example `Box` spaces:

```python
from gymnasium import spaces
import numpy as np

# 2 continuous actions in [-1, 1]
action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)

# RGB image observation
observation_space = spaces.Box(low=0, high=255, shape=(96, 96, 3), dtype=np.uint8)
```

---

## Action / Observation Layouts

Gymnasium spaces describe the valid shape and type of data an RL algorithm must handle.

Common spaces:

| Space | Meaning | Example |
|---|---|---|
| `Discrete(n)` | One integer action from `0` to `n - 1` | `LunarLander-v3` default action space |
| `Box(low, high, shape)` | Numeric vector/image/tensor | Continuous control or image observations |
| `MultiDiscrete([n1, n2])` | Several independent discrete choices | Game controller buttons or multi-part actions |
| `MultiBinary(n)` | `n` binary flags | Multiple buttons pressed/not pressed |
| `Dict({...})` | Named structured observation/action | `{"image": Box(...), "state": Box(...)}` |
| `Tuple((...))` | Ordered structured observation/action | `(Discrete(3), Box(...))` |

This matters because not every RL library or policy head supports every layout directly. A lot of wrapper code exists just to flatten, stack, normalize, cast, or otherwise reshape spaces into something a model can consume.

---

## PufferLib Connection

PufferLib tries to make Gym/Gymnasium-style environments easier to batch, vectorize, and use in multi-agent setups.

Its `pufferlib.spaces` module provides compatibility aliases that work with both Gym and Gymnasium:

```python
import pufferlib.spaces

pufferlib.spaces.Box
pufferlib.spaces.Discrete
pufferlib.spaces.MultiDiscrete
pufferlib.spaces.MultiBinary
pufferlib.spaces.Tuple
pufferlib.spaces.Dict
```

The important PufferLib layout concept is `joint_space(space, n)`, which converts a single-agent space into a multi-agent joint space:

| Single-Agent Space | PufferLib Joint Space |
|---|---|
| `Discrete(k)` | `MultiDiscrete([k] * n)` |
| `MultiDiscrete([k1, k2, ...])` | `Box(shape=(n, len(space)))` |
| `Box(shape=(...))` | `Box(shape=(n, ...))` |

Example:

```python
import gymnasium
import pufferlib.spaces

single_obs = gymnasium.spaces.Box(low=0, high=255, shape=(84, 84, 3))
joint_obs = pufferlib.spaces.joint_space(single_obs, n=8)

print(joint_obs.shape)  # (8, 84, 84, 3)
```

So if the sentence mentioned PufferLib and "action/observation layouts," the topic was probably how spaces get represented when moving from single-agent Gym-style environments to batched or multi-agent training.

---

## Algorithm Implications

| Layout | Typical Algorithms / Policy Heads |
|---|---|
| `Discrete` actions | DQN, PPO with categorical policy, A2C |
| `Box` continuous actions | PPO Gaussian policy, SAC, TD3, DDPG |
| Image `Box` observations | CNN encoder, frame stacking, normalization |
| Vector `Box` observations | MLP encoder, normalization |
| `MultiDiscrete` actions | Multi-categorical policy or wrapper conversion |
| `Dict` observations | Multi-input policy or flatten/extract wrappers |

Examples:

- `LunarLander-v3` default has `Discrete(4)` actions, so a categorical policy works.
- `LunarLander-v3(continuous=True)` has `Box(2,)` actions, so it needs a continuous-action algorithm or policy head.
- `BipedalWalker-v3` is continuous control: `Box(4,)` actions.
- `CarRacing-v3` has pixel observations, so the observation layout is the hard part, not the Box2D physics.

---

## Common Pitfalls

| Problem | Cause | Fix |
|---|---|---|
| `ModuleNotFoundError: Box2D` | Missing Box2D extras | `pip install swig "gymnasium[box2d]"` |
| Confusing `Box2D` and `Box` | Similar names | Remember: Box2D = physics engine, `Box` = numeric space |
| Algorithm rejects environment | Unsupported action/obs space | Use wrappers or choose compatible policy |
| Continuous env used with DQN | DQN expects discrete actions | Use PPO/SAC/TD3/DDPG or discretize actions |
| Image observations train slowly | CNN/pixel input is expensive | Resize, grayscale, frame stack, or use state-vector env |
| Headless rendering fails | Pygame/display issue | Use `render_mode="rgb_array"` or [[Xvfb and Virtual X11 Displays]] |
| PufferLib multi-agent shape mismatch | Single-agent vs joint-space confusion | Check `single_observation_space`, `single_action_space`, and joint shapes |

---

## Quick Checks

Inspect spaces:

```python
import gymnasium as gym

env = gym.make("BipedalWalker-v3")
print(env.action_space)
print(env.observation_space)
```

Sample valid data:

```python
action = env.action_space.sample()
obs, reward, terminated, truncated, info = env.step(action)
```

Check whether a value belongs to a space:

```python
env.action_space.contains(action)
env.observation_space.contains(obs)
```

Run Box2D env with pixel rendering:

```python
env = gym.make("CarRacing-v3", render_mode="rgb_array")
obs, info = env.reset()
frame = env.render()
```

---

## Related Notes

- [[OpenAI Gym]]
- [[PufferLib]]
- [[PufferLib Cheatsheet]]
- [[Reinforcement Learning]]
- [[PPO]]
- [[SAC]]
- [[DQN]]
- [[MuJoCo]]
- [[Xvfb and Virtual X11 Displays]]

---

## Further Reading

- [Box2D documentation](https://box2d.org/documentation/)
- [Gymnasium Box2D environments](https://gymnasium.farama.org/environments/box2d/)
- [Gymnasium spaces API](https://gymnasium.farama.org/v1.1.0/api/spaces/)
- [Gymnasium LunarLander](https://gymnasium.farama.org/environments/box2d/lunar_lander/)
- [Gymnasium BipedalWalker](https://gymnasium.farama.org/environments/box2d/bipedal_walker/)
- [Gymnasium CarRacing](https://gymnasium.farama.org/environments/box2d/car_racing/)
- [PufferLib spaces](https://pufferai-pufferlib.mintlify.app/api/spaces)
