# NeRF (Neural Radiance Fields)

NeRF (Neural Radiance Fields) represents a scene as a continuous volumetric radiance function parameterized by a neural network — the network maps a 5D coordinate `(x,y,z,θ,φ)` (position + view direction) to color and density, and images are synthesized by differentiable volume rendering along camera rays. It was introduced in "NeRF: Representing Scenes as Neural Radiance Fields for View Synthesis" (Mildenhall et al., 2020) and quickly became a foundational technique for photorealistic novel-view synthesis.

Links / quick references  
- Original paper (arXiv PDF): `https://arxiv.org/pdf/2003.08934`  
- Reference implementation (community): `https://github.com/Mr4k/NeRFImpl`  
- Accessible explainer / "Learn You a NeRF": `https://www.peterstefek.me/nerf.html`

---

## 🔭 Overview

NeRF fits a continuous 3D scene representation by optimizing a neural network so that rendered views (via volume rendering) match input photos. It is best suited to static scenes captured from many views with known camera poses (COLMAP poses are commonly used). NeRF excels at producing photorealistic renderings and view-dependent effects (specularities, soft shadows) without explicit meshing.

---

## 🧠 Core Concepts

- **Radiance field (continuous):** a function `F(x, d) -> (c, σ)` where `x` is 3D location, `d` is view direction; `c` = color, `σ` = volume density. Render images by integrating along rays (standard volume rendering).  
- **Positional encoding:** high-frequency Fourier features applied to inputs so the MLP can represent high-frequency variation (important for fine detail).  
- **Ray sampling & quadrature:** sample points along each ray, query network, accumulate color with transmittance weights (compositing).  
- **Training:** minimize photometric loss between rendered rays and ground-truth pixels; typically uses known intrinsics + extrinsics (COLMAP) or joint optimization variants.

---

## 🔬 How it works (high-level pipeline)

1. Capture a set of images with overlapping coverage and estimate camera poses (e.g., `COLMAP`).  
2. For each training iteration sample camera rays (random pixels), sample depth points along rays, apply positional encoding, and run the MLP to get `(c, σ)` for each sample.  
3. Volume-render each sampled ray to compute RGB and compare with ground truth; backpropagate to update weights.  
4. At inference, render arbitrary novel views by casting rays and compositing MLP outputs.

---

## ➗ Math outline (compact)

- MLP `F(γ(x), γ(d)) -> (r,g,b,σ)` where `γ` is positional encoding.  
- Volume rendering: `C(r) = ∫ T(t) σ(t) c(t) dt`, where `T(t)=exp(-∫ σ(s) ds)` is transmittance. Implementation uses discrete quadrature: `C ≈ Σ w_i c_i`, `w_i = T_i (1 - exp(-σ_i Δ_i))`.

---

## ⚖️ Comparison Chart — NeRF vs common alternatives

| Method | Output | Real-time? | Strengths | Typical weakness / failure modes | Notes / source |
|---|---:|:---:|---|---|---|
| **NeRF (MLP + volume rendering)** | Photoreal novel views (rendered pixels) | No (originally slow) | High fidelity, view-dependent effects | Slow training & rendering; needs many views & poses. | |
| **Photogrammetry (SfM + MVS + textured mesh)** | Mesh + textures | Yes (render via GPU) | Standard pipeline, well-understood, widely used (COLMAP) | Poor view-dependent lighting; may struggle with thin/transparent surfaces. | |
| **Voxel / TSDF fusion (dense 3D recon)** | Volumetric TSDF (mesh via marching cubes) | Often yes | Robust for depth sensors & SLAM; easy to integrate | Memory heavy; low texture fidelity vs NeRF | |
| **Gaussian Splatting** | Millions of 3D Gaussians (splatting) | Yes — real-time rendering demonstrated | Very fast rendering/training; high quality for scenes (SIGGRAPH 2023). | Newer; storage & tooling still evolving | |
| **instant-ngp / hash-grid encodings** | Fast NeRF-like models | Near real-time rendering after training | Orders-of-magnitude speedups for training and inference (tiny-cuda-nn). | Sometimes lower quality for some scenes; requires CUDA / GPU | |
| **Light Field / Multi-plane Images (MPI)** | Layered RGBA planes | Often real-time | Simple and fast for forward-facing scenes | Limited parallax and view range | — |

> Notes: The landscape moves fast — recent methods (instant-ngp, Gaussian Splatting, Nerfstudio + Nerfacto) trade quality / memory / speed differently; choose per-application.

---

## 🧩 Variants & notable follow-ups

- **mip-NeRF** — multiscale / integrated positional encoding to reduce aliasing (improves multiscale rendering).  
- **NeRF++** — improved parameterization for unbounded/360° scenes.  
- **NeRF– (NeRF minus)** — jointly optimize cameras and NeRF (camera-free training).  
- **instant-ngp (multires hash)** — extreme speedups by using multiresolution hash encoding + tiny networks (NVIDIA / Müller et al.).  
- **Nerfstudio / Nerfacto** — a modular PyTorch framework & practical recipe combining speed/quality for pipelines & tooling.  
- **Gaussian Splatting** — splat-based radiance representation enabling realtime rendering and fast training (SIGGRAPH 2023).

---

## 🛠️ Developer Tools & Implementations

- `https://github.com/Mr4k/NeRFImpl` — community implementation referenced in tutorials/Colab.  
- `https://github.com/NVlabs/instant-ngp` — instant neural graphics primitives (NeRF + hash encoding).  
- `https://nerf.studio` / Nerfstudio — modular framework for NeRF experiments and pipelines.  
- `https://github.com/google/mipnerf` — mip-NeRF reference implementation (JAX).  
- Gaussian Splatting official code: `https://github.com/graphdeco-inria/gaussian-splatting`.

Examples (one-line commands, inline):  
- `git clone https://github.com/Mr4k/NeRFImpl.git`  
- `git clone https://github.com/NVlabs/instant-ngp.git`

---

## ✅ Strengths

- Photorealistic novel view synthesis with view-dependent effects.  
- Compact, continuous scene representation (no explicit mesh required).  
- Amenable to differentiable rendering tasks (inverse graphics, joint optimization).  

---

## ❌ Weaknesses / Practical limitations

- **Compute & time:** original NeRF trains slowly (hours to days on GPUs) and inference is not real-time. Modern follow-ups mitigate this.  
- **Capture requirements:** needs many well-calibrated views; performance drops with sparse or poorly posed datasets (though newer methods and joint-optimization help).  
- **Editing & geometry export:** getting clean meshes or editable geometry is nontrivial (export to point clouds/meshes is possible via tools/tricks).  

---

## 📦 Hardware & dataset tips

- A CUDA-capable GPU with at least several GB VRAM recommended for medium scenes; instant-ngp / tiny-cuda-nn performs best with modern NVIDIA GPUs.  
- For camera poses use `COLMAP` or similar SfM pipelines; for forward-facing captures consider turntable / hemispherical paths with even coverage.  
- Multi-scale / anti-aliased captures benefit from mip-NeRF style preprocessing.  

---

## 🧾 Use Cases (Robotics & beyond)

- Synthetic view generation for perception datasets (augmenting training data).  
- Visual localization & relighting research.  
- High-fidelity environment capture for simulation and mixed-reality.  
- 3D reconstruction pipelines where photoreal rendering matters more than explicit geometry.  

---

## 🔗 Related Concepts / Notes

- [[Volume Rendering]] (core to NeRF compositing)  
- [[Positional Encoding]] (Fourier features)  
- [[COLMAP]] (SfM / camera poses)  
- [[mip-NeRF]] (anti-aliasing & multiscale)  
- [[instant-ngp]] (fast training & multires hash)  
- [[Gaussian Splatting]] (real-time radiance representation)  
- [[Nerfstudio]] (pipelines & tooling)

---

## 📚 Further Reading & canonical links

- Paper: "NeRF: Representing Scenes as Neural Radiance Fields for View Synthesis" (Mildenhall et al., 2020). `https://arxiv.org/pdf/2003.08934`  
- Learn-you-a-NeRF (practical explainer): `https://www.peterstefek.me/nerf.html`  
- Mr4k/NeRFImpl (community repo referenced in tutorials): `https://github.com/Mr4k/NeRFImpl`  
- mip-NeRF: `https://arxiv.org/abs/2103.13415`  
- instant-ngp: `https://github.com/NVlabs/instant-ngp` (and paper on multiresolution hash encodings).  
- Gaussian Splatting (SIGGRAPH 2023): `https://arxiv.org/abs/2308.04079` and official code.  
- Nerfstudio (framework + Nerfacto recipe): `https://nerf.studio` / paper.  

---

## 🏁 Summary

NeRF is a milestone in neural rendering: a simple, flexible formulation (MLP + volume rendering) that produces photoreal novel views from images. Since 2020 a rapid ecosystem of improvements (anti-aliasing, speedups, unbounded scenes, tooling, and new representations like Gaussian Splatting) has formed — pick a variant or tooling stack (instant-ngp, Nerfstudio, Gaussian Splatting) based on your constraints (quality ↔ speed ↔ storage).
