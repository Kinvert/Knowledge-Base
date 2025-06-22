# 🤖 Transformer (Neural Network Architecture)

The **Transformer** is a deep learning architecture introduced in 2017 by Vaswani et al. in the paper *"Attention is All You Need"*. It revolutionized natural language processing (NLP) and beyond by relying entirely on self-attention mechanisms rather than recurrence or convolution.

---

## 🧠 Summary

- **Type**: Neural network architecture
- **Initial purpose**: Sequence modeling, machine translation
- **Key feature**: Uses self-attention to process entire sequences in parallel
- **Initial release**: 2017 (via the seminal paper)
- **Frameworks**: Implemented in [[TensorFlow]], [[PyTorch]], [[Keras]], [[ONNX]] and others

---

## 🎯 Main Features

- **Self-attention**: Learns contextual relationships between words or tokens in a sequence
- **Parallelization**: Unlike RNNs, processes all tokens at once, allowing faster training
- **Scalability**: Forms the basis of large-scale models like BERT, GPT, T5
- **Positional encoding**: Adds information about token position, since attention lacks inherent sequence order

---

## 🔬 Common Use Cases

- Machine translation
- Text summarization
- Question answering
- Speech recognition
- Image classification (Vision Transformer - ViT)
- Multi-modal learning (e.g. combining text + vision)

---

## 📊 Comparison Table

| Feature                    | Transformer                  | RNN / LSTM                    | CNN (for sequence tasks)         |
|----------------------------|-----------------------------|------------------------------|----------------------------------|
| Sequential processing       | ❌ No (parallelizable)        | ✅ Yes                         | ⚠️ Limited                      |
| Long-range dependency model | ✅ Excellent (via attention)  | ⚠️ Challenging                | ⚠️ Limited                      |
| Computation cost (training) | ⚠️ Higher                    | ✅ Lower (per step)            | ✅ Lower                        |
| Flexibility (input types)    | ✅ High (can generalize)      | ⚠️ Mostly sequential data      | ⚠️ Mostly grid-like data        |

---

## ✅ Strengths

- Captures long-range dependencies effectively
- Highly parallelizable → faster training on modern hardware (e.g. GPUs, TPUs)
- Backbone for state-of-the-art models in NLP and beyond
- Flexible architecture — adapted to vision, speech, multi-modal tasks

---

## ❌ Weaknesses

- Large memory and compute requirements, especially for long sequences
- Can be overkill for small datasets or simple tasks
- Needs positional encoding because it lacks sequence-awareness natively

---

## 🌐 External References

- [Original paper](https://arxiv.org/abs/1706.03762) — *Attention is All You Need*
- [TensorFlow Transformer guide](https://www.tensorflow.org/text/tutorials/transformer)
- [PyTorch implementation example](https://pytorch.org/tutorials/beginner/transformer_tutorial.html)

---

## 🔗 Related Notes

- [[Attention Mechanism]]
- [[Self-Attention]]
- [[BERT]]
- [[GPT]]
- [[TensorFlow]]
- [[PyTorch]]
- [[Machine Learning]]

---
