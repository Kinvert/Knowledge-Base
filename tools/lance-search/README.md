# Knowledge Base Lance Search

This is a separate local toolchain for semantic search over the Obsidian vault.
It is installed into `.lance`, not a general project venv, so LanceDB,
SentenceTransformers, and the CUDA-enabled PyTorch stack stay isolated from the
Markdown repository itself.

## Setup

The environment mirrors the known-good `/home/claude/sim2real` Lance setup. To
avoid re-downloading most of the large CUDA stack on this machine, build from
that local uv cache when it is present:

```bash
UV_CACHE_DIR=/home/claude/sim2real/.uv-cache uv venv .lance --python 3.12
UV_CACHE_DIR=/home/claude/sim2real/.uv-cache uv pip install --python .lance/bin/python -r tools/lance-search/requirements-gpu-cu128.txt
UV_CACHE_DIR=/home/claude/sim2real/.uv-cache uv pip install --python .lance/bin/python --no-deps -e tools/lance-search
```

For a fully independent rebuild, use this vault's `.uv-cache` instead and drop
`--offline`; the first install will download the same packages:

```bash
UV_CACHE_DIR=/home/claude/Documents/Knowledge-Base/.uv-cache uv venv .lance --python 3.12
UV_CACHE_DIR=/home/claude/Documents/Knowledge-Base/.uv-cache uv pip install --python .lance/bin/python -r tools/lance-search/requirements-gpu-cu128.txt
UV_CACHE_DIR=/home/claude/Documents/Knowledge-Base/.uv-cache uv pip install --python .lance/bin/python --no-deps -e tools/lance-search
```

Pinned stack:

- `torch==2.10.0+cu128`
- `sentence-transformers==5.2.3`
- `transformers==5.3.0`
- `lancedb==0.29.2`
- `all-MiniLM-L6-v2` embeddings, 384 dimensions

Check CUDA visibility from the tool venv:

```bash
source .lance/bin/activate
lance-doctor
```

In Codex's default command sandbox, WSL GPU access may be hidden and
`lance-doctor` can report `resolved_device: cpu`. From a normal shell, or from an
approved unsandboxed Codex command, the same `.lance` venv should report CUDA
when the GPU is visible.

## Index

```bash
source .lance/bin/activate
lance-reindex --root /home/claude/Documents/Knowledge-Base
```

The default index lives in `.lancedb/`. SentenceTransformer model files live in
`.lance-cache/sentence-transformers/`. Both are local vault artifacts. LanceDB
search is exact/flat by default, which is appropriate for this small Markdown
corpus.

## Search

```bash
source .lance/bin/activate
search "where are embeddings and vector databases discussed"
```

The indexer walks the working tree directly and stores note chunks with path and
line metadata. It skips venvs, caches, generated LanceDB data, git metadata, and
common binary artifacts.

## PDF Text Extraction

```bash
source .lance/bin/activate
lance-pdf-to-md path/to/document.pdf
```

This writes Markdown next to the PDF by default. Use `--out-dir <dir>` to place
extracted Markdown elsewhere.
