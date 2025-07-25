# Image Classification

Image Classification is a fundamental task in computer vision that involves assigning a label to an entire image. It plays a critical role in a wide range of applications from medical diagnostics and facial recognition to autonomous driving and agricultural monitoring.

## Overview

At its core, image classification involves three main stages:

1. **Input Image** – A digital image, usually in RGB or grayscale format.
2. **Feature Extraction** – Transforming the image into a numerical feature representation.
3. **Classification** – Assigning the image to one or more predefined categories.

Depending on the setup, classification can be:
- **Binary**: e.g. cat vs. non-cat.
- **Multiclass**: e.g. distinguishing between dog, cat, and horse.
- **Multilabel**: e.g. an image might simultaneously contain tags like "beach", "sunset", and "people".

---

## Comparison to Related Tasks

| Task                  | Label Granularity            | Output Example            | Localization Required | Typical Models Used             |
|-----------------------|------------------------------|----------------------------|------------------------|----------------------------------|
| **Image Classification** | Entire image                 | `dog`                      | No                     | ResNet, EfficientNet, Vision Transformers |
| **Object Detection**     | Per object                  | `dog` at (x, y, w, h)      | Yes                    | YOLO, Faster R-CNN, DETR         |
| **Semantic Segmentation**| Per pixel                  | `dog` mask                 | Yes                    | DeepLab, U-Net, SegFormer        |
| **Instance Segmentation**| Per instance, per pixel     | `dog` mask per instance    | Yes                    | Mask R-CNN, YOLACT                |
| **Image Captioning**     | Whole image + sentence      | "A dog running on grass"   | Implied                | CNN + RNN/Transformer            |

---

## Key Concepts

### Feature Extraction
Early methods used handcrafted features like SIFT and HOG. Modern approaches rely on Convolutional Neural Networks (CNNs), which automatically learn hierarchical feature representations.

### Transfer Learning
Instead of training from scratch, models pre-trained on large datasets (like ImageNet) can be fine-tuned on a new, smaller dataset, saving time and improving performance.

### Loss Functions
- **Cross-Entropy Loss**: Most common for classification tasks.
- **Focal Loss**: Useful for imbalanced datasets.
- **Label Smoothing**: Reduces overconfidence in predictions.

---

## Major Architectures

| Model         | Year | Key Innovations                                     | Parameters | ImageNet Top-1 Acc. | Notes                                     |
|---------------|------|-----------------------------------------------------|------------|----------------------|-------------------------------------------|
| **AlexNet**   | 2012 | First deep CNN to win ImageNet                     | ~60M       | 57%                 | Sparked modern deep learning revolution   |
| **VGG-16**    | 2014 | Deep, simple, consistent 3x3 convs                 | ~138M      | 71.5%               | Heavy and memory-intensive                |
| **GoogLeNet** | 2014 | Inception modules (multi-scale processing)        | ~6.8M      | 69.8%               | More efficient, less parameters           |
| **ResNet-50** | 2015 | Residual connections to enable very deep networks | ~25.6M     | 76.2%               | Widely adopted baseline                   |
| **DenseNet-121** | 2016 | Dense connections, improves feature reuse        | ~8M        | 74.9%               | Compact and high-performing               |
| **EfficientNet-B0–B7**| 2019 | Compound scaling: depth, width, resolution   | Varies     | 77–84%+             | Excellent accuracy-efficiency tradeoff    |
| **Vision Transformer (ViT)** | 2020 | Attention-based, image as patches       | ~86M (ViT-B) | ~77.9%              | No CNNs, purely transformer-based         |

---

## Software Libraries & Tools

| Tool / Library         | Use Case                              | Notes                                                                 |
|------------------------|----------------------------------------|-----------------------------------------------------------------------|
| **PyTorch**            | Model building, training, and inference| Dynamic computation graph, widely used in research                   |
| **TensorFlow / Keras** | High-level and low-level API options   | Extensive ecosystem (e.g. TFX, TF Lite, TF Hub)                      |
| **FastAI**             | Rapid prototyping with PyTorch         | Simplifies training and tuning                                       |
| **Hugging Face**       | Pretrained ViTs and image-text models  | Originally NLP, now supports vision models as well                   |
| **OpenCV**             | Preprocessing and traditional CV tasks | Lightweight, fast C++ backend                                        |
| **ONNX Runtime**       | Model deployment                       | Cross-platform and framework-agnostic inference engine               |
| **WandB / TensorBoard**| Logging and experiment tracking        | Visualization of loss, accuracy, gradients, etc.                     |

---

## Datasets

| Dataset      | Classes | Images       | Domain                | Notes                                 |
|--------------|---------|--------------|------------------------|---------------------------------------|
| **ImageNet** | 1,000   | 1.4M         | General objects        | Benchmark for large-scale training    |
| **CIFAR-10/100**| 10/100 | 60K         | Low-resolution         | Good for prototyping                  |
| **MNIST**    | 10      | 70K          | Handwritten digits     | Simple dataset for beginners          |
| **Fashion-MNIST** | 10  | 70K          | Clothing items         | Drop-in MNIST replacement             |
| **Flowers-102** | 102   | ~8K          | Botanical              | Used in fine-grained classification   |
| **iNaturalist**| ~8K    | 675K         | Species classification | High intra-class variance             |

---

## Evaluation Metrics

| Metric             | Description                                                | Typical Use Case                      |
|--------------------|------------------------------------------------------------|----------------------------------------|
| **Accuracy**       | Percentage of correct predictions                          | Balanced datasets                      |
| **Top-K Accuracy** | Whether correct label is in the top K predictions          | Multi-class with many classes          |
| **Precision**      | TP / (TP + FP) – Positive prediction correctness           | Imbalanced datasets                    |
| **Recall**         | TP / (TP + FN) – Positive class coverage                   | Rare class importance                  |
| **F1 Score**       | Harmonic mean of precision and recall                      | Balancing precision and recall         |
| **Confusion Matrix**| Visual breakdown of predicted vs. true classes            | Insight into misclassification patterns|

---

## Typical Pipeline

1. **Data Preprocessing**
   - Resize, normalize, augment (flip, crop, rotate, etc.)
2. **Model Initialization**
   - Use pretrained weights or random init
3. **Training Loop**
   - Forward pass → compute loss → backward pass → update weights
4. **Validation & Evaluation**
   - Run inference on validation set and compute metrics
5. **Inference**
   - Export model and apply to new images
6. **Deployment**
   - Use ONNX, TensorFlow Lite, or edge deployment frameworks

---

## Real-World Applications

- **Medical Imaging**: Classifying X-rays or MRIs (e.g. detecting pneumonia)
- **Autonomous Vehicles**: Traffic sign recognition
- **Agriculture**: Plant disease detection
- **Retail**: Product tagging and visual search
- **Security**: Facial recognition systems

---

## Tips and Considerations

- Always **balance your dataset** or use class weighting for imbalanced problems.
- Monitor **overfitting**: strong training accuracy but poor validation accuracy is a red flag.
- Use **data augmentation** to improve generalization.
- For production, optimize using **quantization** and **pruning**.
- Evaluate multiple models on **latency vs. accuracy** tradeoffs for deployment.

---

## See Also

- [[Object Detection]]
- [[Semantic Segmentation]]
- [[Vision Transformers]]
- [[Transfer Learning]]
- [[Data Augmentation for Images]]
- [[Confusion Matrix and Evaluation Metrics]]
