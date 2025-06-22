# 🔄 ONNX (Open Neural Network Exchange)

**ONNX (Open Neural Network Exchange)** is an open-source format for representing machine learning models. It provides a shared framework for interoperability between different deep learning frameworks, allowing models to be trained in one framework (e.g., [[PyTorch]], [[TensorFlow]]) and deployed in another.

---

## 🧠 Summary

- **Type**: Model format / interoperability standard
- **Initial release**: 2017
- **Creators**: Originally developed by Facebook and Microsoft
- **License**: MIT
- **Core concept**: Export trained models to a common format for cross-framework deployment and inference

---

## 🎯 Main Features

- Framework-agnostic model format
- Supports export and import from frameworks like [[PyTorch]], [[TensorFlow]], scikit-learn, and more
- Wide support for inference runtimes (ONNX Runtime, TensorRT, OpenVINO)
- Designed for both deep learning and classical machine learning models

---

## 🔬 Common Use Cases

- Cross-framework deployment (e.g., train in PyTorch, deploy in TensorRT)
- Hardware-accelerated inference on GPUs, CPUs, NPUs
- Model optimization (e.g., quantization, pruning)
- Serving models in production environments (e.g., cloud, edge devices)

---

## 📊 Comparison Table

| Feature                 | ONNX                    | [[TensorFlow SavedModel]] | TorchScript |
|-------------------------|-------------------------|--------------------------|-------------|
| Cross-framework support  | ✅ Yes                   | ❌ TensorFlow-specific     | ❌ PyTorch-specific |
| Hardware optimization    | ✅ Yes (via runtimes)     | ✅ Yes (TensorRT, XLA)     | ✅ Yes (TorchScript, TensorRT) |
| Flexibility for export   | ✅ Many frameworks       | ❌ TensorFlow only         | ❌ PyTorch only |
| Maturity of tooling      | ✅ Growing rapidly        | ✅ Mature                  | ✅ Mature    |

---

## ✅ Strengths

- Framework independence for model deployment
- Broad hardware compatibility (works with GPU, CPU, FPGA, NPU accelerators)
- Open ecosystem supported by major vendors (Microsoft, NVIDIA, Intel)
- Tooling for model optimization (ONNX Runtime, quantization tools)

---

## ❌ Weaknesses

- Not all model features / layers are fully supported across frameworks
- Exporting complex models may require additional conversion steps or manual fixes
- Tooling is newer compared to framework-native formats

---

## 🛠️ Example Supported Runtimes

- **ONNX Runtime** (Microsoft)
- **TensorRT** (NVIDIA)
- **OpenVINO** (Intel)
- **CoreML Tools** (Apple, via converters)

---

## 🌐 External References

- [Official ONNX website](https://onnx.ai/)
- [ONNX GitHub repo](https://github.com/onnx/onnx)
- [ONNX Runtime](https://onnxruntime.ai/)

---

## 🔗 Related Notes

- [[PyTorch]]
- [[TensorFlow]]
- [[TorchScript]]
- [[TensorRT]]
- [[Compute APIs]]
- [[Machine Learning]]

---
