# Objaverse

**Objaverse** is a massive dataset of 3D objects created by Allen Institute for AI (AI2). Objaverse 1.0 contains 800K+ annotated 3D models; Objaverse-XL expands this to 10M+ objects from diverse sources including artist-created models, photogrammetry scans, and historic artifact scans. It's become a foundational resource for training 3D generative models, embodied AI, and robotics simulation.

---

## Overview

Before Objaverse, 3D datasets were small and limited in diversity. Objaverse changed this by aggregating millions of objects with metadata (captions, tags, categories). Combined with procedural scene generators like [[ProcTHOR]] and [[Infinigen]], Objaverse enables creation of diverse training environments at scale.

Website: https://objaverse.allenai.org/

---

## Dataset Versions

| Version | Objects | Sources | Released |
|---------|---------|---------|----------|
| **Objaverse 1.0** | 800K+ | Sketchfab, curated | 2022 |
| **Objaverse-XL** | 10M+ | GitHub, Thingiverse, Polycam, Smithsonian | 2023 |

---

## Core Features

- **Diverse Objects**: Furniture, vehicles, animals, tools, food, architecture
- **Rich Metadata**: Captions, tags, categories, animations
- **Multiple Formats**: GLB, OBJ, FBX available
- **Annotations**: Semantic tags, descriptions
- **Permissive Licensing**: Most objects usable for research

---

## Use Cases in Robotics

| Application | How Objaverse Helps |
|-------------|---------------------|
| **Scene Generation** | Populate [[ProcTHOR]]/[[Infinigen]] with diverse objects |
| **Manipulation Training** | Varied object shapes/sizes for grasping |
| **Object Detection** | Train detectors on rendered synthetic data |
| **[[Domain Randomization]]** | Swap objects to increase policy robustness |
| **3D Vision** | Train depth estimation, 3D reconstruction |

---

## Comparison: 3D Object Datasets

| Dataset | Objects | Diversity | Annotations | Robotics Use |
|---------|---------|-----------|-------------|--------------|
| **Objaverse-XL** | 10M+ | Very high | Tags, captions | Scene population |
| ShapeNet | 51K | Medium | Categories | Manipulation |
| ModelNet | 127K | Low | Categories | Classification |
| Google Scans | 1K | Low | High quality | Manipulation |
| YCB | 77 | Very low | Precise physics | Benchmarking |

---

## Integration Examples

**With ProcTHOR:**
```python
# ProcTHOR can use Objaverse assets for scene diversity
scene = procthor.generate(
    rooms=["kitchen"],
    asset_source="objaverse"
)
```

**With Isaac Sim:**
- Convert GLB → USD
- Import into [[Isaac Replicator]] for synthetic data
- Use with [[Domain Randomization]] pipelines

---

## Key Projects Using Objaverse

- **AI2 Embodied AI**: Navigation and manipulation training
- **Zero-1-to-3**: 3D reconstruction from single image
- **Point-E / Shap-E**: OpenAI 3D generation models
- **SUGAR**: 3D visual pre-training for robotics

---

## Strengths

- Largest public 3D dataset by far
- High diversity across categories
- Active maintenance by AI2
- Multiple download formats
- HuggingFace integration

---

## Weaknesses

- Quality varies (user-generated content)
- Some objects lack physics properties
- May need cleanup for simulation
- Large download size

---

## Related Notes

- [[ProcTHOR]] (Uses Objaverse assets)
- [[Infinigen]] (Complementary procedural generation)
- [[Isaac Replicator]] (Synthetic data with 3D assets)
- [[Domain Randomization]] (Object variation technique)
- [[Sim2Real]] (End goal)

---

## External Resources

- [Objaverse Website](https://objaverse.allenai.org/)
- [Objaverse-XL Paper](https://arxiv.org/abs/2307.05663)
- [HuggingFace Dataset](https://huggingface.co/datasets/allenai/objaverse)
- [GitHub Repository](https://github.com/allenai/objaverse)

---
