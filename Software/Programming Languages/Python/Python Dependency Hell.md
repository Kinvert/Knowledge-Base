# Python Dependency Hell

Python Dependency Hell is the collection of problems teams hit when Python packages, native libraries, toolchains, system paths, and multiple interpreters interact in surprising ways. This note digs deep into common failure modes, compares the major tooling (Conda/mamba/micromamba, venv/virtualenv, pyenv, pip, pipenv, poetry, pip-tools, pipx, **uv**), and covers C/C++/Rust/Zig bindings, wheels vs source builds, reproducible CI pipelines, and practical recipes for fast-moving startup teams.

---

## ⚙️ Overview
Dependency hell arises from several interacting factors:
- many packages require compiled native libs (BLAS, CUDA, libssl, libsodium)
- multiple Python versions and interpreters on one machine
- binary-ABI incompatibilities (glibc, manylinux tags)
- package managers with different dependency resolution semantics
- PATH and environment activation side effects
- mixing system package managers (apt/yum/brew) with Python installers

Fixing it is partly about tooling choice and mostly about predictable build & CI practices: lockfiles, wheels, explicit native deps, and lightweight reproducible environments (containers, micromamba, or **uv-managed environments**).

---

## 🧭 Quick taxonomy (tools & roles)
- **Interpreter/version managers:** `pyenv`, system `python`, `uv python`
- **Virtual environment managers:** `venv`, `virtualenv`, `conda env`, `uv venv`
- **Package installers / dependency managers:** `pip`, `conda`, `mamba`, `micromamba`, **uv**
- **Higher-level dependency tools / workflow:** `poetry`, `pipenv`, `pip-tools` (`pip-compile`), `poetry-core`
- **Global tool installers for CLIs:** `pipx`, `uvx`
- **Build tools for native extensions:** `setuptools`, `maturin` (Rust), `cibuildwheel`, `scikit-build`, `pybind11`, `cffi`
- **System package managers:** `apt`, `yum`, `brew` (used for native prerequisites)

---

## 🧠 Core failure modes (why "hell" happens)
1. **Path & Activation Side Effects** — Activating Conda or other envs rewrites `PATH` and can change `LD_LIBRARY_PATH`, `PYTHONPATH`, and shell hooks; unexpected activation order can shadow system `python` or binaries like `gcc`.  
2. **Mixed Managers** — Installing some packages with `conda` and some with `pip` in the same env leads to inconsistent dependency graphs and broken native libs. Similar issues arise when mixing `uv`-managed envs with legacy global pip installs.  
3. **Binary ABI mismatch** — Wheels built for a different `manylinux` tag or linked against different `glibc`/libstdc++ versions fail at import.  
4. **Solver Differences** — `conda` historically had a slower solver (fixed by `mamba`), `pip` uses SAT solver semantics vs dependency constraints expressed differently across tools; **uv introduces a fast, parallel resolver optimized for deterministic installs**.  
5. **Transitive C deps** — Python package X depends on system lib Y; if Y is missing or wrong version, `pip install X` will fail or compile from source unpredictably.  
6. **Reproducibility gaps** — Not pinning direct and transitive versions, or not locking platform-specific wheels, causes builds to diverge across developers and CI.

---

## 📊 Tool comparison — Environment & installer overview

| Tool | Role | Strengths | Weaknesses | Best for |
|------|------|----------|-----------|----------|
| `conda` | Env manager + package manager (binaries) | Manages native deps, prebuilt binary packages (conda-forge), cross-platform | Can change `PATH` and shell state; mixing with `pip` risky; larger environment size | Data science stacks, native libs, GPUs |
| `mamba` | Drop-in fast `conda` solver | Very fast resolution, same ecosystem | Still conda ecosystem semantics | Faster conda workflows |
| `micromamba` | Lightweight `mamba` (no Python) | Tiny bootstrap, good for containers/CI | Less human-friendly local UX than conda | CI, Docker bootstrap |
| `venv` / `virtualenv` | Virtualenv for system Python | Minimal, predictable (uses system Python) | No native dependency management; you must install compilers & libs | App teams with simple pure-Python deps |
| `pyenv` | Python version manager | Per-project Python versions using shims | Adds shim layer to `PATH`, can be confusing | Teams needing many Python versions |
| `pip` | Package installer (PyPI) | Ubiquitous, fastest for pure-Python | Installs from sources when wheels missing; no native libs management | Pure-Python packages, CI with wheels |
| `poetry` | Dependency resolution + packaging | Single-file workflow (pyproject.toml), lockfile, publishing | Sometimes strict or unexpected resolver behavior; multiplatform wheel handling needs setup | App teams wanting reproducible builds & publishing |
| `pipenv` | `pip` + `virtualenv` integration | Opinionated env + lockfile | Historically slow/buggy; less popular now | Legacy projects |
| `pip-tools` (`pip-compile`) | Produce pinned `requirements.txt` from loose specs | Deterministic locked files for `pip` | Extra build step, needs workflow discipline | Teams using `pip` + `venv` but want reproducibility |
| `pipx` | Install Python CLIs globally | Installs tools isolated from project | Not for apps, just CLIs | Developer tooling (black, ruff) |
| **uv** | All-in-one package/env manager | Fast, universal lockfile, built-in venv & Python version management | Newer tool, smaller ecosystem than pip/conda | Startups & teams needing fast reproducible installs, mixed projects |

---

## 🔬 Multiple comparison tables (focus areas)

### 1) Resolution & reproducibility

| Tool | Lockfile | Reproducible installs across OS? | Handles native libs? |
|------|----------|----------------------------------|-----------------------|
| pip + constraints/pip-tools | `requirements.txt` (pinned) | Yes if pinned to wheels per-platform | No (needs system libs) |
| poetry | `poetry.lock` | Yes-ish (platform-specific differences) | No, relies on wheels or conda for native libs |
| **uv** | Universal lockfile in `pyproject.toml` | Yes | No native lib packaging (but installs wheels and uses pip backend efficiently) |
| conda | `environment.yml` + explicit pins | Yes (conda packages are binaries) | Yes — main advantage |
| micromamba | same as conda | Yes, great in CI/docker | Yes |

### 2) Native C/C++ deps & bindings

| Tool | Native prebuilt libs | Wheel building support | Easiest for C-extension-heavy projects |
|------|----------------------|------------------------|-----------------------------------------|
| conda/conda-forge | Yes (many libs packaged) | N/A (uses prebuilt pkgs) | Best (no toolchain required on host) |
| pip (manylinux wheels) | Depends on wheel availability | `wheel`, `setuptools`, `scikit-build` | OK if manylinux wheels exist |
| cibuildwheel + pip | Builds manylinux wheels in CI | Excellent (produces wheels you can host on PyPI) | Good for OSS packages |
| maturin (Rust) | Produces wheels (manylinux) | Excellent for Rust bindings | Best for Rust-based extensions |
| pybind11 / setuptools | Source extension building | Works but requires dev toolchain | Good if CI builds wheels for you |
| **uv** | Uses pip wheels; can cache & reuse across projects | Built-in wheel installation; can invoke cibuildwheel indirectly | Good for Rust/C extensions when using wheels |

### 3) Performance & developer UX

| Tool | Solve speed | Local UX friction | CI-friendliness |
|------|-------------|-------------------|------------------|
| [[pip]] + venv | Very fast | Low friction | Very CI-friendly with wheels |
| mamba | Very fast | Requires conda ecosystem | Excellent in conda CI |
| poetry | Moderate | Reasonable UX, single file | Good but needs wheel building for native extensions |
| [[Conda]] | Historically slow solver; `mamba` fixes that | Moderate; activation changes PATH | Very CI-friendly when using micromamba in containers |
| [[UV]] | Very fast (parallel resolution) | Minimal friction; built-in venv & Python | Excellent; single tool handles lockfile, venv, Python version, and dev CLI execution |

---

## 🧩 Conda: path issues, myths, facts
**Short answer:** Conda *does* modify shell environment variables and `PATH` as part of activation. That behavior can surprise teams if they don't understand activation semantics. That isn't "broken" but it *is* impactful.

**Details & practical implications:**
- `conda activate <env>` places the env's `bin`/`Scripts` at the front of `PATH`, so `python`, `gcc`, and other binaries from that env override system binaries. This is intentional to ensure the env's interpreter and native libs are used.
- Older conda recommended adding shell hooks to auto-activate; some shells got slower or confusing prompts. Modern conda and `mamba` are better but still modify `PATH`.
- `conda` sets `LD_LIBRARY_PATH` in some cases (particularly with certain packages), which can cause runtime linking differences and subtle failures when calling system binaries. Use `conda run`/`micromamba run` in CI to avoid persistent shell state.
- Mixing `conda` installs and external system installs (apt/brew) can lead to version mismatches (e.g., `openssl`).
- If you use `pyenv` shims + `conda` activation, order matters: pyenv shims are in `PATH` too; the active env may override the shim.

**Mitigations / Best practices when using conda:**
- Prefer `mamba` or `micromamba` for speed and CI. Use `micromamba` in Docker to bootstrap reproducible images with minimal side effects.
- Avoid global `conda` base env activation; keep `base` deactivated. Use per-project envs only.
- Never mix `conda` and `pip` blindly. If you must `pip install`, prefer installing pure-Python packages after `conda install` of native deps, and pin versions.
- Use `conda-lock` (community tool) or explicit `environment.lock.yml` to pin exact package builds across platforms.
- Use `conda-forge` as the single channel where possible; mixing `defaults` and `conda-forge` increases solver surprise.

---

## ⚙️ UV

**uv** is a fast Python package & project manager written in Rust.

Key features:
- Replaces `pip`, `pip-tools`, `virtualenv`/`venv`, `pyenv`, `pipx`, and more.
- 10×–100× faster than pip due to parallel resolution, efficient caching, and Rust performance.
- Universal lockfile via `pyproject.toml` and `tool.uv` config.
- Built-in virtual environments (`uv venv`), Python version management (`uv python`), and tool/script execution (`uv run`, `uvx`).
- Global cache for deduplicated dependencies.
- Dependency override support, alternative sources, platform-specific dependencies.
- Designed for both app projects and library development with predictable, reproducible installs.

---

## UV vs Conda (direct comparison)

| Feature | uv | conda |
|---------|----|-------|
| Lockfile | `pyproject.toml` universal | `environment.yml` + optional `conda-lock` |
| Env isolation | Built-in venv (`uv venv`) | Conda envs |
| Python version management | Built-in (`uv python`) | Requires pyenv or system Python |
| Native libs | Relies on pip wheels; no native lib packaging | Conda packages include native libs |
| Solve speed | Very fast; parallel resolver | Historically slower; `mamba` fixes speed |
| CI / reproducibility | Excellent; single tool for env + deps | Excellent with micromamba; larger images if using full Anaconda |
| Ecosystem | Smaller than conda; fewer prebuilt binaries | Very large; conda-forge provides many prebuilt binaries |
| Best use case | Startups, pure-Python apps, hybrid projects, fast iteration | Data science, ML, GPU-heavy projects needing prebuilt native binaries |

---

## 🛠 Python Bindings (C/C++/Rust/Zig) — practical guidance

### Build & distribution patterns
- **Prefer wheels**: Prebuilt wheels for your target platforms remove a major source of hell. Use `cibuildwheel` (Linux manylinux, macOS, Windows) in CI to produce wheels and upload them to PyPI or an internal wheelhouse.
- **If your project depends on native libs** (OpenBLAS, libsndfile, libxml2): conda-forge often supplies ready-made packages. For apps that run in containers, install the native libs using system package manager in `Dockerfile` (Debian/Ubuntu: `apt-get install -y libsndfile1`) or use micromamba to install conda packages inside the container.
- **Use `pybind11` or `cffi`** for C++ bindings. For Rust, use `maturin`. These tools can produce wheels directly.
- **For Zig-based native code**: There is growing support for producing C ABI artifacts; you still need to produce wheels using `setuptools` or `scikit-build` glue, or call Zig from `cibuildwheel` CI.
- **Manylinux policy**: Build Linux wheels in manylinux containers (cibuildwheel or manylinux images). This ensures broader compatibility.

### Common traps
- Building locally with `gcc` different from CI can cause ABI mismatches. Always build in controlled CI that matches the target environment.
- Python headers and matching `python-dev` packages must match the interpreter used for wheel building.
- Linking to system libraries that aren't present on the target runtime (e.g., different glibc) will break wheel portability.

---

## 🚀 Best choices for different team profiles

### A. Data-science / ML team with heavy native deps (numpy, scipy, cuda, tensorflow, pytorch)
- **Use Conda (conda-forge) + mamba** for development.  
- For CI and deployment, prefer **Docker + micromamba** to create minimal reproducible images.  
- Build any custom native wheels with `cibuildwheel` and host in a private index or conda channel if needed.  
- Lock envs with `conda-lock` or pinned `environment.yml`.

**Why:** conda abstracts native binaries and GPU toolchains, avoiding constant `pip` source builds.

### B. Web / API / App team, mostly pure-Python
- **Use Poetry** (for dependency spec + lockfile) or `pip` + `pip-tools` + `venv`.  
- Use `pip-compile` to generate a pinned `requirements.txt` for CI.  
- Containerize with Docker and install from the pinned `requirements.txt` or `poetry export --format requirements.txt`.  
- Use `pipx` for developer CLI tools.

**Why:** smaller envs, fast installs, minimal native complexity.

### C. Small, fast-moving startup with mixed needs (some ML, some APIs)
- **Hybrid approach:** Use `venv`/`pip`/`pip-tools` for the app stack, but use **conda only in dedicated data/ML worker images**.  
- Central rule: avoid using conda for the app unless you need native deps it uniquely solves. Prefer containerized separation (one container built with conda for heavy ML tasks; web images use slim Python images).

**Why:** avoids conda path side-effects for API containers; keeps developer iteration fast.

### D. Library authors (publishing wheels)
- Use `cibuildwheel` to produce manylinux/macos/windows wheels. Test installing from PyPI clean VMs or containers.  
- Use `maturin` for Rust-based libs.

---

## ✅ Practical startup-friendly workflows (recipes)

### Recipe A: Fast app dev (web service)
1. Developer: `python -m venv .venv`  
2. Activate: `source .venv/bin/activate`  
3. Use `poetry` or `pip-tools` for dependency resolution:
   - With poetry: `poetry add flask` and commit `poetry.lock`.
   - Or: maintain `pyproject.toml` and `requirements.in` + `pip-compile`.
4. CI: build Docker image with `COPY requirements.txt` and `pip install --no-cache-dir -r requirements.txt`.
5. For native deps in production, install via the Docker `apt-get` lines or multi-stage builds.

### Recipe B: ML notebook + GPU (data science)
1. Use `mamba` to create env: `mamba create -n py39 python=3.9 cuda-toolkit=11.8 -c conda-forge`.
2. Install major libs with conda: `mamba install -c conda-forge pytorch torchvision`.
3. For project-specific pure-Python libs, use `pip` after env activation and pin them in `requirements.txt`.
4. Use Docker with `micromamba` to reproduce image for training jobs.

### Recipe C: Building native extension wheels for distribution
1. Use `cibuildwheel` in CI to output wheels for manylinux/mac/win.
2. Upload wheels to PyPI or an internal index.
3. Consumers `pip install yourpackage` and get prebuilt wheels — avoids source builds.

---

## 🧭 CI / CD best practices
- Build and publish wheels in CI for each platform rather than compiling at consumer install time.
- Use Docker base images matched to production (e.g., `python:3.11-slim` or micromamba-based images).
- Use `micromamba` to install conda packages in CI images quickly without pulling an entire Anaconda distribution.
- Always run `pip install --upgrade pip build` during CI to avoid old pip wheel builder issues.
- Verify smoke tests by creating a clean container and installing only the published artifacts.

---

## ⚠️ Pitfalls & how to avoid them (practical cheatsheet)
- **Problem:** `ImportError` on a worker but local dev works  
  **Fix:** ensure wheels were built against the same platform; add native libs to production container.
- **Problem:** `conda` env activation changes PATH unexpectedly  
  **Fix:** use `conda run` or `micromamba run` in CI; teach team to avoid activating base env.
- **Problem:** `pip install` compiles from source in CI and takes 30+ minutes  
  **Fix:** publish wheels or use conda packages for the heavy deps.
- **Problem:** version resolution takes forever with large envs  
  **Fix:** use `mamba` or constrain versions tighter; pin transitive deps with `pip-compile`.
- **Problem:** Different developers on macOS/Linux see different behavior  
  **Fix:** containerize dev or provide a dev container (VS Code devcontainer) matching CI.

---

## 🔑 Recommendations (short — what to pick)

- **If you need native libs / GPUs / scientific stack:** `conda` (use `mamba`/`micromamba` for speed & CI). Use conda-forge and avoid mixing channels.
- **If you want minimal friction, reproducibility for pure-Python webapps:** `venv` + `pip` + `pip-tools` or `poetry`.
- **If you publish native extension packages:** Invest in `cibuildwheel` and host wheels (or conda packages).
- **For developer tooling:** `pipx` for CLIs, `pyenv` for managing interpreter versions (but be mindful of PATH interactions).
- **For startup teams wanting speed and fewer surprises:** containerize development (Dev Containers), and standardize on one reproducible image per service (either pip-based slim image or micromamba-based image for ML).

---

## 🔗 Related Concepts / Notes
- [[cibuildwheel]] (manylinux wheel build)
- [[Conda]] (conda-forge, mamba, micromamba)
- [[Anaconda]]
- [[venv]] (virtualenv)
- [[pyenv]]
- [[poetry]]
- [[pip-tools]]
- [[pipx]]
- [[Native Extensions]]
- [[Manylinux Wheel]]
- [[Docker for Python]]
- [[UV]]

---

## 🔎 Further Reading / External Resources
- PEP 517 / PEP 518 (build system interface)
- Manylinux wheel policy
- conda-forge documentation
- poetry docs and migration guides
- cibuildwheel examples
- maturin (for Rust bindings)

---

## 🧾 TL;DR (Executive Summary)
- There is no single silver bullet. Choose **conda** for heavy native/GPU stacks, **pip/poetry/pip-tools + venv** for webapps and fast iteration.
- Use **mamba/micromamba** to fix slow conda solver problems and make CI reproducible.
- **Build and publish wheels** for native extensions—don’t rely on consumers to compile them.
- Avoid mixing package managers blindly; prefer containment (containers or per-project envs) and explicit lockfiles.
- For startups: aim for *fast iteration + reproducible CI* — often achieved by local `venv` development + containerized production images; use conda images only where necessary.
