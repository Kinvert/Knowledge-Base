# Open X-Embodiment

**Open X-Embodiment** (OXE) is a collaborative robotics dataset combining 60+ existing datasets from 34 research labs worldwide. It contains demonstrations across diverse robots, tasks, and environments, enabling training of "generalist" robot policies that transfer across embodiments. Google DeepMind's RT-X models trained on OXE show improved generalization compared to single-dataset training.

---

## Overview

Traditionally, robot learning trains separate models per robot, task, and environment. OXE tests whether pooling data across robots enables generalist policies—similar to how LLMs benefit from diverse text data. The dataset spans manipulation, navigation, and mobile manipulation across robot arms, quadrupeds, and humanoids.

Website: https://robotics-transformer-x.github.io/

---

## Dataset Statistics

| Metric | Value |
|--------|-------|
| **Datasets** | 60+ |
| **Research Labs** | 34 |
| **Robot Types** | Arms, mobile bases, quadrupeds, humanoids |
| **Total Trajectories** | 1M+ demonstrations |
| **Countries** | Global (US, Europe, Asia) |

---

## Included Datasets

| Category | Examples |
|----------|----------|
| **Kitchen/Household** | Bridge, RT-1, Kitchen Manipulation |
| **Tabletop Manipulation** | Berkeley Autolab, Furniture Bench |
| **Mobile Manipulation** | TOTO, Language Table |
| **Navigation** | NYU Door Opening |
| **Dexterous** | DROID, Cable Routing |

---

## RT-X Models

Google trained two models on OXE:

| Model | Architecture | Parameters | Key Result |
|-------|--------------|------------|------------|
| **RT-1-X** | Transformer for control | 35M | +50% success vs single-dataset |
| **RT-2-X** | VLM fine-tuned for actions | 55B | Emergent reasoning + control |

Both output robot actions as tokens, treating control as a sequence modeling problem.

---

## Key Findings

- **Positive Transfer**: Training on diverse robots improves performance on each
- **Emergent Skills**: RT-2-X shows reasoning about novel objects
- **Scale Benefits**: More data consistently improves generalization
- **Embodiment Gap**: Some robots benefit more than others

---

## Comparison: Robot Datasets

| Dataset | Trajectories | Robots | Scenes | Focus |
|---------|--------------|--------|--------|-------|
| **Open X-Embodiment** | 1M+ | Many | 300+ | Generalist policies |
| [[DROID]] | 76K | 1 type | 564 | Scene diversity |
| Bridge | 60K | 1 | Limited | Kitchen tasks |
| RT-1 | 130K | 1 | Google offices | Real-world scale |

---

## Use Cases

- Training foundation models for robotics
- Benchmarking cross-embodiment transfer
- Pre-training before fine-tuning on target robot
- Studying what transfers across robots

---

## Accessing the Data

```python
# Via TensorFlow Datasets
import tensorflow_datasets as tfds
dataset = tfds.load('open_x_embodiment')

# Via HuggingFace
from datasets import load_dataset
dataset = load_dataset("jxu124/OpenX-Embodiment")
```

---

## Strengths

- Largest multi-robot dataset
- Proven transfer learning benefits
- Standardized format (RLDS)
- Active community contributions
- Open access

---

## Weaknesses

- Quality varies across source datasets
- Some robots underrepresented
- Scene diversity still limited (~300 total)
- Mostly manipulation (less locomotion)

---

## Related Notes

- [[DROID]] (Complementary high-diversity dataset)
- [[Sim2Real]] (OXE is real-world data)
- [[Imitation Learning]] (Primary use case)
- [[Reinforcement Learning]]
- [[Transformer]] (RT-X architecture)

---

## External Resources

- [Project Website](https://robotics-transformer-x.github.io/)
- [Paper (arXiv)](https://arxiv.org/abs/2310.08864)
- [TensorFlow Dataset](https://www.tensorflow.org/datasets/catalog/open_x_embodiment)
- [HuggingFace](https://huggingface.co/datasets/jxu124/OpenX-Embodiment)

---
