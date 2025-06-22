# 🔥 PyTorch

**PyTorch** is an open-source machine learning framework originally developed by Facebook's AI Research lab (FAIR). Known for its dynamic computation graph and Pythonic interface, PyTorch is widely used in research and increasingly in production for deep learning applications.

---

## 🧠 Summary

- **Type**: Machine learning / deep learning framework
- **Initial release**: 2016
- **Languages**: Python (primary), with C++ front-end support
- **License**: BSD (open source)
- **Core concept**: Uses dynamic computation graphs (define-by-run), allowing flexible model construction and debugging

---

## 🎯 Main Features

- **Dynamic computation graph**: Build and modify models on the fly — great for research
- **Tensor operations**: Similar to NumPy but with GPU acceleration
- **Autograd**: Automatic differentiation for gradient computation
- **TorchScript**: Serialize and optimize models for production
- **Strong integration**: Works well with other Python libraries (NumPy, SciPy, etc.)

---

## 🔬 Common Use Cases

- Research and rapid prototyping in deep learning
- Computer vision (CNNs, segmentation, object detection)
- Natural language processing (transformers, RNNs)
- Reinforcement learning
- Time series forecasting
- Generative models (GANs, VAEs)

---

## 📊 Comparison Table

| Feature                   | [[PyTorch]]       | [[TensorFlow]]       | [[Keras]] (standalone or wrapper) |
|---------------------------|------------------|---------------------|-----------------------------------|
| Dynamic vs static graph     | ✅ Dynamic (eager) | ⚠️ Originally static, dynamic via `tf.function` | Depends on backend |
| Deployment tools           | ⚠️ Growing (TorchServe, TorchScript) | ✅ Strong (TFX, Serving, Lite) | Uses backend capabilities |
| Visualization              | ⚠️ TensorBoard + third-party | ✅ TensorBoard | ✅ (via backend) |
| Community / Ecosystem       | ✅ Large, fast-growing | ✅ Very large | ✅ Large |
| Ease of use                 | ✅ Very Pythonic | ⚠️ Medium | ✅ High |

---

## ✅ Strengths

- Pythonic, intuitive syntax for researchers and developers
- Excellent debugging and flexibility (use of standard Python control flow)
- Strong support for GPU acceleration (CUDA)
- Growing production tools (TorchServe, TorchScript)
- Large community and ecosystem

---

## ❌ Weaknesses

- Historically less tooling for production deployment (though improving)
- Slightly fewer official production frameworks compared to TensorFlow
- Some features (e.g., mobile support) newer and less mature

---

## 🌐 External References

- [Official PyTorch website](https://pytorch.org/)
- [GitHub repo](https://github.com/pytorch/pytorch)
- [TorchServe](https://pytorch.org/serve/)

---

## 🔗 Related Notes

- [[TensorFlow]]
- [[Keras]]
- [[CUDA]]
- [[Compute APIs]]
- [[Machine Learning]]
- [[TorchScript]]
- [[ONNX]]

---
