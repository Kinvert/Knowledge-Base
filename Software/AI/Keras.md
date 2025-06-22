# 🧩 Keras

**Keras** is a high-level neural network API designed for easy and fast prototyping. It provides a user-friendly interface for building and training deep learning models. Originally developed as an independent project, it is now tightly integrated with [[TensorFlow]] as its official high-level API.

---

## 🧠 Summary

- **Type**: High-level deep learning API
- **Initial release**: 2015
- **Languages**: Python (primary)
- **License**: MIT (open source)
- **Backends**: TensorFlow (official), previously supported Theano and CNTK

---

## 🎯 Main Features

- **Ease of use**: Clear and concise syntax for defining neural networks
- **Modularity**: Models are built as sequences or graphs of fully configurable modules
- **Extensibility**: Custom layers, models, and loss functions easily implemented
- **Compatibility**: Runs seamlessly on CPU and GPU

---

## 🔬 Common Use Cases

- Educational purposes and tutorials
- Rapid prototyping of neural networks
- Production models (via TensorFlow integration)
- Transfer learning and fine-tuning of pre-trained models

---

## 📊 Comparison Table

| Feature                  | [[Keras]]              | [[PyTorch]]           | [[TensorFlow]]         |
|--------------------------|-----------------------|----------------------|-----------------------|
| API level                 | High-level             | Low / mid-level       | Low / mid / high-level |
| Flexibility               | ⚠️ Moderate (via subclassing) | ✅ High                | ✅ High                |
| Ease of use               | ✅ Very easy            | ✅ Pythonic            | ⚠️ Medium              |
| Production support        | ✅ Via TensorFlow       | ✅ (growing)           | ✅ Strong               |
| Dynamic computation graph | ⚠️ No (uses TF backend) | ✅ Yes                 | ✅ Yes (TF 2.x eager)   |

---

## ✅ Strengths

- Simple, clean, beginner-friendly API
- Rapid model development and prototyping
- Tight integration with TensorFlow ecosystem
- Large number of pre-built layers and utilities

---

## ❌ Weaknesses

- Less flexible than lower-level APIs for custom models
- Originally designed for research and prototyping — production features are through TensorFlow
- Advanced optimization or graph manipulation requires dropping to TensorFlow-level code

---

## 🌐 External References

- [Official Keras website](https://keras.io/)
- [Keras GitHub repo](https://github.com/keras-team/keras)

---

## 🔗 Related Notes

- [[TensorFlow]]
- [[PyTorch]]
- [[ONNX]]
- [[Machine Learning]]
- [[Compute APIs]]

---
