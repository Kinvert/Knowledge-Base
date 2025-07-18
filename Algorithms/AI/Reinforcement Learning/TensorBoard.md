# TensorBoard 📊

**TensorBoard** is a powerful visualization toolkit for inspecting and debugging machine learning models, especially those trained using TensorFlow or PyTorch. It’s commonly used in reinforcement learning and robotics to monitor training metrics, visualize neural network graphs, and debug performance bottlenecks.

---

## 🚀 Launch TensorBoard

From your training directory (where logs are stored):

`tensorboard --logdir=runs`

Or specify a different directory:

`tensorboard --logdir=./logs --port=6007`

Then open in browser:

`http://localhost:6006` (default)

---

## 🧠 Common Features

- **Scalars:** Plot metrics like reward, loss, accuracy over time
- **Graphs:** Visualize the computational graph of the model
- **Images:** View input/output images during training
- **Histograms:** Monitor distributions of weights/activations
- **Projector:** Explore high-dimensional embeddings like t-SNE
- **Text:** Log arbitrary text (e.g., summaries, debug info)

---

## 🔧 Logging from PyTorch

Basic usage with `SummaryWriter`:

`from torch.utils.tensorboard import SummaryWriter`

`writer = SummaryWriter()`

`writer.add_scalar("reward", reward, step)`

`writer.close()`

---

## 🔧 Logging from TensorFlow

Basic setup:

`import tensorflow as tf`

`summary_writer = tf.summary.create_file_writer("logs")`

`with summary_writer.as_default(): tf.summary.scalar("loss", loss, step=step)`

---

## 🧪 Use in Reinforcement Learning

- Log episodic rewards and lengths
- Visualize action distributions
- Track entropy, value loss, policy loss
- View agent’s camera input with `add_image`

---

## ⚙️ Integration with RL Libraries

- [[Stable-Baselines3]]: `tensorboard_log` argument in model constructor
- [[RLlib]]: Built-in TensorBoard support via Ray Tune
- [[PufferLib]]: Add your own `SummaryWriter` where needed
- [[Lightning]]: Automatically integrates logging if enabled

---

## 🧩 Related Notes

- [[Reinforcement Learning]] (Core RL concepts)
- [[Stable-Baselines3]] (Popular RL library)
- [[RLlib]] (Distributed RL and hyperparameter tuning)
- [[PufferLib]] (Env wrapping with logging hooks)
- [[PyTorch]] (Popular deep learning framework)
- [[TensorFlow]] (Deep learning library with native TensorBoard support)

---
