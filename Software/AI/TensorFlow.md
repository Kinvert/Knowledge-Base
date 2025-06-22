# ⚙️ TensorFlow

**TensorFlow** is an open-source machine learning and deep learning framework originally developed by the Google Brain team. It provides tools to design, build, train, and deploy machine learning models across a wide range of platforms — from servers to mobile devices and edge devices.

---

## 🧠 Summary

- **Type**: Machine learning / deep learning framework
- **Initial release**: 2015
- **Languages**: Primarily Python, with APIs for C++, Java, JavaScript, Go, Swift, and more
- **License**: Apache 2.0 (open source)
- **Core concept**: Defines computations as dataflow graphs where nodes represent operations and edges represent tensors (multi-dimensional arrays)

---

## 🎯 Main Features

- **Flexible architecture**: Deployable on CPUs, GPUs, TPUs
- **Tensor operations**: Efficient handling of n-dimensional arrays (tensors)
- **Auto-differentiation**: Supports automatic computation of gradients for optimization
- **Visualization**: Integrated with [[TensorBoard]] for monitoring and debugging
- **Extensive ecosystem**:
  - TensorFlow Lite (for mobile/edge)
  - TensorFlow.js (for in-browser ML)
  - TensorFlow Serving (for production deployment)
  - TensorFlow Probability, TensorFlow Extended (TFX)

---

## 🔬 Common Use Cases

- Neural networks (CNNs, RNNs, transformers)
- Natural language processing (NLP)
- Computer vision
- Time series analysis
- Reinforcement learning
- Generative models (e.g. GANs, VAEs)
- Edge deployment (via TensorFlow Lite)

---

## 📊 Comparison Table

| Feature                   | TensorFlow        | [[PyTorch]]          | [[Keras]] (standalone or wrapper) |
|---------------------------|------------------|---------------------|-----------------------------------|
| Dynamic vs static graph     | Originally static (now also dynamic via `tf.function`) | Dynamic (eager by default) | Depends on backend |
| Deployment tools           | ✅ Strong (TFX, Serving, Lite) | ⚠️ Less mature | Uses backend capabilities |
| Visualization              | ✅ TensorBoard    | ⚠️ TensorBoard + third-party | ✅ (via backend) |
| Community / Ecosystem       | ✅ Very large     | ✅ Large, growing    | ✅ Large, tied to backend |
| Ease of use                 | ⚠️ Medium        | ✅ Easier (Pythonic) | ✅ High                        |

---

## ✅ Strengths

- Production-ready with strong deployment support
- Cross-platform (mobile, embedded, cloud)
- Optimized for performance (XLA, TPU support)
- Large community and extensive documentation
- Rich tooling (debugging, visualization, model conversion)

---

## ❌ Weaknesses

- Historically more complex syntax compared to [[PyTorch]]
- Steeper learning curve (especially pre-TensorFlow 2.x)
- Can feel heavy for small projects or prototyping

---

## 🌐 External References

- [Official TensorFlow website](https://www.tensorflow.org/)
- [GitHub repo](https://github.com/tensorflow/tensorflow)
- [TensorBoard](https://www.tensorflow.org/tensorboard)

---

## 🔗 Related Notes

- [[PyTorch]]
- [[Keras]]
- [[TensorBoard]]
- [[Machine Learning]]
- [[Compute APIs]]
- [[CUDA]]
- [[Edge AI]]

---
