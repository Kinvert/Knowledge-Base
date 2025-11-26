# setup.py

**setup.py** is the traditional build and distribution script used in Python projects to define package metadata, dependencies, and installation behavior. It plays a foundational role in packaging Python-based Reinforcement Learning (RL) libraries, environments, and tooling, enabling reproducible installs, versioning, and integration with the broader Python ecosystem.

---

## 🧠 Overview

`setup.py` is driven by the `setuptools` library and acts as the executable specification describing how a Python project should be built, installed, and distributed. Historically, nearly all Python packages relied on this file to define their structure and installation rules, but modern tooling increasingly shifts responsibility toward declarative formats like `pyproject.toml`.

In RL workflows, `setup.py` is often found in research codebases, custom simulation environments, and internal tooling, where rapid iteration and flexible dependency control are common.

---

## ⚙️ How It Works

At its core, `setup.py` executes a Python script that calls `setuptools.setup()` with a collection of parameters describing the package:
- Package name and version
- Dependencies and optional extras
- Entry points and CLI scripts
- Build configuration
- Metadata (author, license, description)
- Extension module compilation rules

It is invoked by tools such as:
- `pip install .`
- `python setup.py install`
- `python setup.py sdist`
- `python setup.py bdist_wheel`

These commands trigger build and install processes based on the logic declared inside the file.

---

## 🔑 Key Features

- Imperative control over build logic
- Flexible dependency specification
- Custom install hooks
- Packaging configuration for PyPI
- Extension module compilation support
- Script and CLI exposure

---

## 🧩 Core Concepts

- setuptools API
- Python packaging ecosystem
- Distribution metadata
- Source and binary distributions
- Build lifecycle hooks
- Dependency resolution

---

## 📊 Comparison Chart

| Packaging Method | Style | Flexibility | Modern Recommendation | Typical Usage |
|------------------|-------|-------------|------------------------|---------------|
| setup.py | Imperative | High | Legacy / Transitional | Older Python packages |
| [[pyproject.toml]] | Declarative | Medium–High | Preferred | Modern Python projects |
| [[setup.cfg]] | Declarative | Medium | Transitional | Config-based packaging |
| [[Poetry]] | Tool-based | High | Strongly Recommended | Modern dependency management |
| [[Flit]] | Minimalist | Medium | Recommended | Simple packages |
| [[Hatch]] | Tool-based | High | Emerging Standard | Advanced workflows |
| [[Conda Recipes]] | Environment-focused | Medium | Niche | Scientific computing |

---

## 🎯 Use Cases

- Packaging RL libraries for internal deployment
- Installing custom gym environments
- Defining build logic for simulation engines
- ML research code distribution
- Versioning and dependency locking

---

## ✅ Strengths

- Extremely flexible and powerful
- Scriptable logic for complex builds
- Wide historical adoption
- Deep integration with setuptools

---

## ❌ Weaknesses

- Imperative and error-prone
- Harder to maintain than declarative formats
- Superseded by modern standards
- Can introduce reproducibility issues

---

## 🧾 Cheatsheet: One-Liners & Common kwargs

### Common Commands
- Install local package: `pip install .`
- Editable install: `pip install -e .`
- Build source distribution: `python setup.py sdist`
- Build wheel: `python setup.py bdist_wheel`
- Install with verbose output: `pip install . -v`
- Clean build artifacts: `python setup.py clean --all`

### Core `setuptools.setup()` kwargs

**Metadata**
- `name`
- `version`
- `description`
- `long_description`
- `long_description_content_type`
- `author`
- `author_email`
- `url`
- `project_urls`
- `license`
- `classifiers`
- `keywords`

**Package Discovery**
- `packages`
- `py_modules`
- `package_dir`
- `include_package_data`
- `exclude_package_data`

**Dependencies**
- `install_requires`
- `extras_require`
- `setup_requires`
- `python_requires`
- `tests_require`

**Build / Distribution**
- `cmdclass`
- `ext_modules`
- `entry_points`
- `scripts`
- `zip_safe`
- `data_files`

**Advanced / Hooks**
- `dependency_links`
- `namespace_packages`
- `test_suite`
- `test_loader`
- `obsoletes`
- `provides`
- `requires`

---

**⚡ Using uv with setup.py**

`uv` is a modern, ultra-fast Python package manager that works seamlessly with legacy `setup.py` projects.

**Common Usage**
- Install package: `uv pip install .`
- Editable install: `uv pip install -e .`
- Install with extras: `uv pip install .[dev,rl]`
- Lock dependencies: `uv pip compile requirements.in`
- Sync environment: `uv pip sync requirements.txt`

**Notes**
- `uv` respects `install_requires` and `extras_require`
- Wraps PEP-517 behavior but supports legacy setup.py workflows
- Accelerates RL experiment iterations and reproducible builds

---

## ✏️ Editable Mode

Editable mode allows installed packages to directly reflect changes made in the source code without reinstallation.

**Invocation**
- Using pip: `pip install -e .`
- Using setuptools: `python setup.py develop`

**Behavior**
- Creates a symlink to the source directory
- Changes take effect immediately
- Ideal for:
  - Development of RL environments
  - Rapid experimentation
  - Debugging reusable modules

**Notes**
- Relies on setuptools `develop` mode
- Best used with virtual environments
- Compatible with modern build backends, but conflicts can occur if using strict PEP-517-only setups

---

## 🔧 Compatible Items

- Python
- setuptools
- pip
- PyPI
- wheel
- virtualenv
- [[CI-CD]]
- [[Python Packaging]]

---

## 🧪 Variants

- setup.py with setup.cfg hybrid
- setup.py with pyproject.toml bootstrap
- Legacy-only setup.py projects
- Auto-generated setup.py via tools
- Custom build script extensions

---

## 🛠 Developer Tools

- setuptools
- pip
- wheel
- twine
- Poetry
- Hatch
- Conda Build Tools

---

## 📚 Documentation and Support

- Python Packaging Authority (PyPA) guides
- setuptools official documentation
- pip user guides
- PyPI publishing documentation
- Migration guides to pyproject.toml

---

## 🧬 Capabilities

- Build automation
- Dependency management
- Version control coordination
- Distribution orchestration
- Installation scripting
- Binary package generation

---

## 🔍 Key Highlights

- Backbone of legacy Python packaging
- Highly flexible but increasingly deprecated
- Still common in RL research repositories
- Core to Python distribution history

---

## 🔗 Related Concepts / Notes

- [[Python]]
- [[setuptools]]
- [[pip]]
- [[PyPI]]
- [[Poetry]]
- [[pyproject.toml]]
- [[CI-CD]]
- [[Virtual Environments]]
- [[Dependency Management]]
- [[Python Dependency Hell]]

---

## 🏁 Summary

`setup.py` has long served as the primary mechanism for defining, building, and distributing Python packages. While its flexibility made it ubiquitous, modern packaging standards now favor declarative approaches that reduce complexity and improve reproducibility. Nevertheless, understanding `setup.py` remains essential for navigating legacy Python and RL codebases, especially in research and experimental environments.
