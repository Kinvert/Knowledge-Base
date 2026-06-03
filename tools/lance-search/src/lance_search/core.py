from __future__ import annotations

import hashlib
import json
import os
import re
import textwrap
from dataclasses import dataclass
from datetime import datetime, timezone
from importlib import metadata
from pathlib import Path
from typing import Iterator, Sequence


DEFAULT_MODEL = "all-MiniLM-L6-v2"
DEFAULT_TABLE = "repo_chunks"
DEFAULT_DB_DIR = ".lancedb"
DEFAULT_CACHE_DIR = ".lance-cache/sentence-transformers"

SKIP_DIR_NAMES = {
    ".eggs",
    ".agents",
    ".codex",
    ".git",
    ".hg",
    ".hypothesis",
    ".lance",
    ".lance-cache",
    ".lance-index",
    ".lancedb",
    ".mypy_cache",
    ".nox",
    ".pytest_cache",
    ".ruff_cache",
    ".svn",
    ".tox",
    ".uv-cache",
    ".venv",
    "__pycache__",
    "build",
    "checkpoints",
    "dist",
    "downloads",
    "eggs",
    "env",
    "experiments",
    "htmlcov",
    "node_modules",
    "raylib_wasm",
    "target",
    "venv",
    "wandb",
}

SKIP_DIR_PREFIXES = ("raylib-", "box2d-")

BINARY_SUFFIXES = {
    ".7z",
    ".a",
    ".bin",
    ".bmp",
    ".db",
    ".dylib",
    ".egg",
    ".elf",
    ".gif",
    ".gz",
    ".ico",
    ".jpeg",
    ".jpg",
    ".npy",
    ".npz",
    ".o",
    ".pdf",  # handled separately
    ".png",
    ".pyc",
    ".pyo",
    ".so",
    ".sqlite",
    ".tar",
    ".tgz",
    ".tiff",
    ".webp",
    ".whl",
    ".zip",
}

TEXT_SUFFIXES = {
    ".bash",
    ".bs2",
    ".c",
    ".cfg",
    ".cmake",
    ".cpp",
    ".css",
    ".csv",
    ".cu",
    ".cuh",
    ".dockerignore",
    ".gitignore",
    ".h",
    ".hpp",
    ".html",
    ".ini",
    ".ino",
    ".ipynb",
    ".java",
    ".js",
    ".json",
    ".jsx",
    ".lock",
    ".lua",
    ".m",
    ".md",
    ".mdx",
    ".py",
    ".rs",
    ".sh",
    ".spin",
    ".sql",
    ".toml",
    ".ts",
    ".tsx",
    ".txt",
    ".xml",
    ".yaml",
    ".yml",
}


@dataclass(frozen=True)
class ChunkConfig:
    lines: int = 80
    overlap: int = 12
    min_chars: int = 1


@dataclass(frozen=True)
class SourceChunk:
    path: str
    text: str
    start_line: int | None = None
    end_line: int | None = None
    page: int | None = None
    kind: str = "text"


@dataclass(frozen=True)
class IndexStats:
    files_seen: int
    chunks_indexed: int
    db_path: Path
    table: str
    model: str
    device: str
    vector_dim: int | None = None


def find_repo_root(start: Path | None = None) -> Path:
    current = (start or Path.cwd()).resolve()
    for candidate in (current, *current.parents):
        if (candidate / ".git").exists():
            return candidate
    return current


def default_db_path(root: Path) -> Path:
    return root / DEFAULT_DB_DIR


def default_cache_dir(root: Path) -> Path:
    return root / DEFAULT_CACHE_DIR


def resolve_device(device: str = "auto") -> str:
    if device != "auto":
        return device
    try:
        import torch
    except Exception:
        return "cpu"
    return "cuda" if torch.cuda.is_available() else "cpu"


def should_skip_dir(path: Path) -> bool:
    name = path.name
    return name in SKIP_DIR_NAMES or name.startswith(SKIP_DIR_PREFIXES)


def should_skip_file(path: Path) -> bool:
    suffix = path.suffix.lower()
    if suffix == ".pdf":
        return False
    if suffix in BINARY_SUFFIXES:
        return True
    return False


def iter_indexable_files(root: Path, max_file_bytes: int = 16 * 1024 * 1024) -> Iterator[Path]:
    root = root.resolve()
    for dirpath, dirnames, filenames in os.walk(root):
        current = Path(dirpath)
        dirnames[:] = sorted(
            dirname for dirname in dirnames if not should_skip_dir(current / dirname)
        )
        for filename in sorted(filenames):
            path = current / filename
            if should_skip_file(path):
                continue
            try:
                if path.stat().st_size > max_file_bytes:
                    continue
            except OSError:
                continue
            suffix = path.suffix.lower()
            if suffix == ".pdf" or suffix in TEXT_SUFFIXES:
                yield path
                continue
            if read_text_file(path, max_bytes=max_file_bytes) is not None:
                yield path


def read_text_file(path: Path, max_bytes: int = 16 * 1024 * 1024) -> str | None:
    try:
        data = path.read_bytes()
    except OSError:
        return None
    if len(data) > max_bytes or b"\x00" in data:
        return None

    for encoding in ("utf-8", "utf-16", "latin-1"):
        try:
            text = data.decode(encoding)
        except UnicodeDecodeError:
            continue
        if _looks_like_text(text):
            return text
    return None


def _looks_like_text(text: str) -> bool:
    if not text:
        return True
    sample = text[:4096]
    control = sum(
        1 for char in sample if ord(char) < 32 and char not in "\n\r\t\f\b"
    )
    return control / max(len(sample), 1) < 0.02


def chunk_text(path: str, text: str, config: ChunkConfig = ChunkConfig()) -> Iterator[SourceChunk]:
    if config.lines < 1:
        raise ValueError("chunk line count must be at least 1")
    if config.overlap < 0 or config.overlap >= config.lines:
        raise ValueError("chunk overlap must be non-negative and smaller than line count")

    lines = text.splitlines()
    if not lines:
        stripped = text.strip()
        if stripped:
            yield SourceChunk(path=path, text=stripped, start_line=1, end_line=1)
        return

    step = config.lines - config.overlap
    for start in range(0, len(lines), step):
        window = lines[start : start + config.lines]
        body = "\n".join(window).strip()
        if len(body) >= config.min_chars:
            yield SourceChunk(
                path=path,
                text=body,
                start_line=start + 1,
                end_line=start + len(window),
            )
        if start + config.lines >= len(lines):
            break


def extract_pdf_chunks(
    root: Path,
    path: Path,
    config: ChunkConfig = ChunkConfig(),
) -> Iterator[SourceChunk]:
    from pypdf import PdfReader

    rel_path = path.relative_to(root).as_posix()
    try:
        reader = PdfReader(str(path))
    except Exception as exc:  # pypdf raises several parser-specific errors.
        yield SourceChunk(
            path=rel_path,
            text=f"PDF extraction failed for {rel_path}: {exc}",
            page=None,
            kind="pdf_error",
        )
        return

    try:
        pages = list(reader.pages)
    except Exception as exc:
        yield SourceChunk(
            path=rel_path,
            text=f"PDF extraction failed for {rel_path}: {exc}",
            page=None,
            kind="pdf_error",
        )
        return

    for index, page in enumerate(pages, start=1):
        try:
            text = page.extract_text() or ""
        except Exception as exc:
            text = f"PDF page extraction failed for page {index}: {exc}"
        for chunk in chunk_text(
            rel_path,
            text,
            ChunkConfig(lines=config.lines, overlap=config.overlap, min_chars=config.min_chars),
        ):
            yield SourceChunk(
                path=chunk.path,
                text=chunk.text,
                start_line=chunk.start_line,
                end_line=chunk.end_line,
                page=index,
                kind="pdf",
            )


def iter_source_chunks(
    root: Path,
    config: ChunkConfig = ChunkConfig(),
    max_file_bytes: int = 16 * 1024 * 1024,
) -> Iterator[SourceChunk]:
    root = root.resolve()
    for path in iter_indexable_files(root, max_file_bytes=max_file_bytes):
        rel_path = path.relative_to(root).as_posix()
        if path.suffix.lower() == ".pdf":
            yield from extract_pdf_chunks(root, path, config=config)
            continue
        text = read_text_file(path, max_bytes=max_file_bytes)
        if text is None:
            continue
        yield from chunk_text(rel_path, text, config=config)


def chunk_id(chunk: SourceChunk) -> str:
    payload = json.dumps(
        {
            "path": chunk.path,
            "start_line": chunk.start_line,
            "end_line": chunk.end_line,
            "page": chunk.page,
            "kind": chunk.kind,
            "text_sha256": hashlib.sha256(chunk.text.encode("utf-8")).hexdigest(),
        },
        sort_keys=True,
    )
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def chunk_record(chunk: SourceChunk) -> dict[str, object]:
    return {
        "id": chunk_id(chunk),
        "path": chunk.path,
        "start_line": chunk.start_line,
        "end_line": chunk.end_line,
        "page": chunk.page,
        "kind": chunk.kind,
        "text": chunk.text,
    }


def batched(items: Sequence[dict[str, object]], batch_size: int) -> Iterator[list[dict[str, object]]]:
    for index in range(0, len(items), batch_size):
        yield list(items[index : index + batch_size])


def load_embedding_model(model_name: str, cache_dir: Path, device: str = "auto"):
    from sentence_transformers import SentenceTransformer

    resolved_device = resolve_device(device)
    model = SentenceTransformer(
        model_name,
        cache_folder=str(cache_dir),
        device=resolved_device,
    )
    return model, resolved_device


def rebuild_index(
    root: Path,
    db_path: Path | None = None,
    table_name: str = DEFAULT_TABLE,
    model_name: str = DEFAULT_MODEL,
    cache_dir: Path | None = None,
    device: str = "auto",
    chunk_config: ChunkConfig = ChunkConfig(),
    batch_size: int = 64,
    max_file_bytes: int = 16 * 1024 * 1024,
    normalize_embeddings: bool = True,
) -> IndexStats:
    import lancedb

    root = root.resolve()
    db_path = (db_path or default_db_path(root)).resolve()
    cache_dir = (cache_dir or default_cache_dir(root)).resolve()
    cache_dir.mkdir(parents=True, exist_ok=True)

    records = [chunk_record(chunk) for chunk in iter_source_chunks(root, chunk_config, max_file_bytes)]
    if not records:
        raise RuntimeError(f"no indexable chunks found under {root}")

    model, resolved_device = load_embedding_model(model_name, cache_dir, device=device)
    embedded: list[dict[str, object]] = []
    for batch in batched(records, batch_size):
        vectors = model.encode(
            [str(record["text"]) for record in batch],
            batch_size=batch_size,
            convert_to_numpy=True,
            normalize_embeddings=normalize_embeddings,
            show_progress_bar=False,
        )
        for record, vector in zip(batch, vectors):
            record = dict(record)
            record["vector"] = vector.tolist()
            embedded.append(record)

    db_path.mkdir(parents=True, exist_ok=True)
    db = lancedb.connect(str(db_path))
    if table_name in db.table_names():
        db.drop_table(table_name)
    db.create_table(table_name, data=embedded)
    write_manifest(
        db_path=db_path,
        table_name=table_name,
        model_name=model_name,
        device=resolved_device,
        root=root,
        chunks=len(embedded),
        vector_dim=len(embedded[0]["vector"]) if embedded else None,
        normalize_embeddings=normalize_embeddings,
    )
    return IndexStats(
        files_seen=sum(1 for _ in iter_indexable_files(root, max_file_bytes=max_file_bytes)),
        chunks_indexed=len(embedded),
        db_path=db_path,
        table=table_name,
        model=model_name,
        device=resolved_device,
        vector_dim=len(embedded[0]["vector"]) if embedded else None,
    )


def search_index(
    query: str,
    root: Path,
    db_path: Path | None = None,
    table_name: str = DEFAULT_TABLE,
    model_name: str = DEFAULT_MODEL,
    cache_dir: Path | None = None,
    device: str = "auto",
    limit: int = 8,
    normalize_embeddings: bool = True,
) -> list[dict[str, object]]:
    import lancedb

    root = root.resolve()
    db_path = (db_path or default_db_path(root)).resolve()
    cache_dir = (cache_dir or default_cache_dir(root)).resolve()
    db = lancedb.connect(str(db_path))
    table = db.open_table(table_name)
    model, _resolved_device = load_embedding_model(model_name, cache_dir, device=device)
    vector = model.encode(
        [query],
        convert_to_numpy=True,
        normalize_embeddings=normalize_embeddings,
        show_progress_bar=False,
    )[0].tolist()
    return table.search(vector).limit(limit).to_list()


def write_manifest(
    db_path: Path,
    table_name: str,
    model_name: str,
    device: str,
    root: Path,
    chunks: int,
    vector_dim: int | None,
    normalize_embeddings: bool,
) -> None:
    manifest = {
        "created_at": datetime.now(timezone.utc).isoformat(),
        "root": str(root),
        "table": table_name,
        "model": model_name,
        "device": device,
        "chunks": chunks,
        "vector_dim": vector_dim,
        "normalize_embeddings": normalize_embeddings,
    }
    (db_path / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")


def doctor_info(root: Path, model_name: str = DEFAULT_MODEL, device: str = "auto") -> dict[str, object]:
    root = root.resolve()
    info: dict[str, object] = {
        "root": str(root),
        "db_path": str(default_db_path(root)),
        "cache_dir": str(default_cache_dir(root)),
        "model": model_name,
        "requested_device": device,
        "resolved_device": resolve_device(device),
    }
    for package in (
        "lancedb",
        "sentence-transformers",
        "transformers",
        "torch",
        "numpy",
        "pyarrow",
    ):
        try:
            info[package] = metadata.version(package)
        except metadata.PackageNotFoundError:
            info[package] = None

    try:
        import torch

        info["torch_cuda_version"] = torch.version.cuda
        info["cuda_available"] = torch.cuda.is_available()
        if torch.cuda.is_available():
            info["cuda_device_name"] = torch.cuda.get_device_name(0)
            info["cuda_device_capability"] = torch.cuda.get_device_capability(0)
            info["torch_cuda_arch_list"] = torch.cuda.get_arch_list()
    except Exception as exc:
        info["torch_error"] = str(exc)
    return info


def default_markdown_path(pdf_path: Path, out_dir: Path | None) -> Path:
    if out_dir is None:
        return pdf_path.with_suffix(".md")
    return out_dir / f"{pdf_path.stem}.md"


def pdf_to_markdown(pdf_path: Path, output_path: Path | None = None) -> Path:
    from pypdf import PdfReader

    output_path = output_path or default_markdown_path(pdf_path, None)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    reader = PdfReader(str(pdf_path))
    parts = [f"# {pdf_path.name}", ""]
    for index, page in enumerate(reader.pages, start=1):
        text = page.extract_text() or ""
        parts.extend([f"## Page {index}", "", text.strip(), ""])
    output_path.write_text("\n".join(parts).rstrip() + "\n", encoding="utf-8")
    return output_path


def format_result(result: dict[str, object], rank: int, width: int = 1000) -> str:
    path = str(result.get("path", ""))
    page = result.get("page")
    start_line = result.get("start_line")
    location = path
    if page:
        location = f"{location} page {page}"
    elif start_line:
        location = f"{location}:{start_line}"
    distance = result.get("_distance")
    score = f" distance={float(distance):.4f}" if distance is not None else ""
    text = re.sub(r"\s+", " ", str(result.get("text", ""))).strip()
    snippet = textwrap.shorten(text, width=width, placeholder=" ...")
    return f"[{rank}] {location}{score}\n{snippet}"
