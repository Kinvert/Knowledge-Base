# 🧠 BERT (Bidirectional Encoder Representations from Transformers)

**BERT** is a deep learning model for natural language understanding (NLU) based on the [[Transformer]] architecture. Introduced by Google AI in 2018, BERT set new benchmarks on a wide range of NLP tasks by using a bidirectional approach to model context from both directions (left and right) simultaneously.

---

## 🧠 Summary

- **Type**: Pre-trained transformer-based language model
- **Published by**: Google AI
- **Initial release**: 2018
- **License**: Apache 2.0 (open source)
- **Main innovation**: Deep bidirectional self-attention for better context understanding

---

## 🎯 Main Features

- **Bidirectional context**: Unlike earlier models (e.g. GPT) that read text left-to-right or right-to-left, BERT reads both ways simultaneously.
- **Pre-training + fine-tuning**: Pre-trained on large corpora (Wikipedia, BooksCorpus), then fine-tuned on specific tasks.
- **Masked language model (MLM)**: Randomly masks words in a sentence and trains the model to predict them.
- **Next sentence prediction (NSP)**: Trains the model to understand sentence relationships.

---

## 🔬 Common Use Cases

- Question answering
- Sentiment analysis
- Named entity recognition (NER)
- Text classification
- Sentence similarity tasks
- Language inference

---

## 📊 Comparison Table

| Model               | Directionality            | Pre-training tasks          | Size (base version) | Example use case                  |
|---------------------|--------------------------|-----------------------------|--------------------|------------------------------------|
| [[BERT]]            | ✅ Bidirectional           | MLM + NSP                    | ~110M params        | NLU, QA, sentiment                |
| [[GPT]]             | ❌ Unidirectional (left→right) | Language modeling             | ~117M params        | Text generation                   |
| [[RoBERTa]]         | ✅ Bidirectional (BERT variant) | MLM (no NSP)                  | ~125M params        | NLU                               |
| [[DistilBERT]]      | ✅ Bidirectional (smaller) | MLM + NSP                    | ~66M params         | NLU on edge devices               |

---

## ✅ Strengths

- Strong performance on many NLP benchmarks (GLUE, SQuAD)
- Open source and widely available pre-trained models
- Adaptable via fine-tuning for specific tasks
- Bidirectional context leads to better comprehension

---

## ❌ Weaknesses

- Large size → high memory and compute requirements
- Not designed for text generation
- Slow inference for large-scale production without optimization

---

## 🌐 External References

- [Original BERT paper](https://arxiv.org/abs/1810.04805)
- [Google AI BERT announcement](https://ai.googleblog.com/2018/11/open-sourcing-bert-state-of-art-pre.html)
- [Hugging Face Transformers BERT docs](https://huggingface.co/transformers/model_doc/bert.html)

---

## 🔗 Related Notes

- [[Transformer]]
- [[Attention Mechanism]]
- [[GPT]]
- [[RoBERTa]]
- [[DistilBERT]]
- [[Machine Learning]]
- [[TensorFlow]]
- [[PyTorch]]

---
