from __future__ import annotations

import argparse
from pathlib import Path

from .core import (
    DEFAULT_MODEL,
    DEFAULT_TABLE,
    ChunkConfig,
    default_cache_dir,
    default_db_path,
    default_markdown_path,
    find_repo_root,
    format_result,
    doctor_info,
    pdf_to_markdown,
    rebuild_index,
    search_index,
)


def positive_int(value: str) -> int:
    parsed = int(value)
    if parsed < 1:
        raise argparse.ArgumentTypeError("must be at least 1")
    return parsed


def root_path(value: str | None) -> Path:
    if value:
        return Path(value).expanduser().resolve()
    return find_repo_root(Path.cwd())


def add_common_index_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--root", help="Repository root. Defaults to the nearest parent with .git.")
    parser.add_argument("--db", help="LanceDB directory. Defaults to <root>/.lancedb.")
    parser.add_argument("--table", default=DEFAULT_TABLE, help=f"LanceDB table name. Default: {DEFAULT_TABLE}.")
    parser.add_argument(
        "--model",
        default=DEFAULT_MODEL,
        help=f"SentenceTransformer model. Default: {DEFAULT_MODEL}.",
    )
    parser.add_argument(
        "--device",
        default="auto",
        help="Embedding device: auto, cuda, or cpu. Default: auto.",
    )
    parser.add_argument(
        "--cache-dir",
        help="SentenceTransformer cache directory. Defaults to <root>/.lance-cache/sentence-transformers.",
    )


def resolved_paths(args: argparse.Namespace) -> tuple[Path, Path, Path]:
    root = root_path(args.root)
    db_path = Path(args.db).expanduser().resolve() if args.db else default_db_path(root)
    cache_dir = (
        Path(args.cache_dir).expanduser().resolve()
        if args.cache_dir
        else default_cache_dir(root)
    )
    return root, db_path, cache_dir


def reindex_main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Build a local LanceDB semantic index for this checkout.")
    add_common_index_args(parser)
    parser.add_argument("--chunk-lines", type=positive_int, default=80, help="Lines per text chunk.")
    parser.add_argument("--overlap-lines", type=int, default=12, help="Overlapping lines between chunks.")
    parser.add_argument("--batch-size", type=positive_int, default=64, help="Embedding batch size.")
    parser.add_argument(
        "--max-file-bytes",
        type=positive_int,
        default=16 * 1024 * 1024,
        help="Skip individual files larger than this many bytes.",
    )
    args = parser.parse_args(argv)

    root, db_path, cache_dir = resolved_paths(args)
    stats = rebuild_index(
        root=root,
        db_path=db_path,
        table_name=args.table,
        model_name=args.model,
        cache_dir=cache_dir,
        device=args.device,
        chunk_config=ChunkConfig(lines=args.chunk_lines, overlap=args.overlap_lines),
        batch_size=args.batch_size,
        max_file_bytes=args.max_file_bytes,
    )
    print(
        f"Indexed {stats.chunks_indexed} chunks from {stats.files_seen} files "
        f"into {stats.db_path} table={stats.table} model={stats.model} "
        f"device={stats.device} dim={stats.vector_dim}"
    )


def search_main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Search the local LanceDB semantic index.")
    add_common_index_args(parser)
    parser.add_argument("query", nargs="+", help="Natural language query.")
    parser.add_argument("--limit", type=positive_int, default=8, help="Number of results.")
    args = parser.parse_args(argv)

    root, db_path, cache_dir = resolved_paths(args)
    query = " ".join(args.query)
    results = search_index(
        query=query,
        root=root,
        db_path=db_path,
        table_name=args.table,
        model_name=args.model,
        cache_dir=cache_dir,
        device=args.device,
        limit=args.limit,
    )
    for rank, result in enumerate(results, start=1):
        print(format_result(result, rank))
        if rank != len(results):
            print()


def pdf_to_md_main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Extract PDF text into Markdown files.")
    parser.add_argument("pdfs", nargs="+", help="PDF path(s) to extract.")
    parser.add_argument("--out-dir", help="Directory for Markdown outputs. Defaults next to each PDF.")
    args = parser.parse_args(argv)

    out_dir = Path(args.out_dir).expanduser().resolve() if args.out_dir else None
    for raw_pdf in args.pdfs:
        pdf = Path(raw_pdf).expanduser().resolve()
        output = default_markdown_path(pdf, out_dir)
        written = pdf_to_markdown(pdf, output)
        print(written)


def doctor_main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Report .lance search runtime and CUDA status.")
    parser.add_argument("--root", help="Repository root. Defaults to the nearest parent with .git.")
    parser.add_argument("--model", default=DEFAULT_MODEL, help=f"SentenceTransformer model. Default: {DEFAULT_MODEL}.")
    parser.add_argument("--device", default="auto", help="Embedding device: auto, cuda, or cpu. Default: auto.")
    args = parser.parse_args(argv)

    info = doctor_info(root_path(args.root), model_name=args.model, device=args.device)
    for key, value in info.items():
        print(f"{key}: {value}")


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Knowledge Base Lance search tools")
    subcommands = parser.add_subparsers(dest="command", required=True)

    reindex_parser = subcommands.add_parser("reindex", help="Build or rebuild the semantic index.")
    add_common_index_args(reindex_parser)
    reindex_parser.add_argument("--chunk-lines", type=positive_int, default=80)
    reindex_parser.add_argument("--overlap-lines", type=int, default=12)
    reindex_parser.add_argument("--batch-size", type=positive_int, default=64)
    reindex_parser.add_argument("--max-file-bytes", type=positive_int, default=16 * 1024 * 1024)

    search_parser = subcommands.add_parser("search", help="Search the semantic index.")
    add_common_index_args(search_parser)
    search_parser.add_argument("query", nargs="+")
    search_parser.add_argument("--limit", type=positive_int, default=8)

    pdf_parser = subcommands.add_parser("pdf-to-md", help="Extract PDF text into Markdown.")
    pdf_parser.add_argument("pdfs", nargs="+")
    pdf_parser.add_argument("--out-dir")

    doctor_parser = subcommands.add_parser("doctor", help="Report runtime and CUDA status.")
    doctor_parser.add_argument("--root")
    doctor_parser.add_argument("--model", default=DEFAULT_MODEL)
    doctor_parser.add_argument("--device", default="auto")

    args = parser.parse_args(argv)
    if args.command == "reindex":
        reindex_main(_namespace_to_args(args, reindex_parser))
    elif args.command == "search":
        search_main(_namespace_to_args(args, search_parser))
    elif args.command == "pdf-to-md":
        pdf_to_md_main(_namespace_to_args(args, pdf_parser))
    elif args.command == "doctor":
        doctor_main(_namespace_to_args(args, doctor_parser))


def _namespace_to_args(args: argparse.Namespace, parser: argparse.ArgumentParser) -> list[str]:
    output: list[str] = []
    for action in parser._actions:
        if not action.option_strings or action.dest in {"help"}:
            continue
        value = getattr(args, action.dest)
        if value is None:
            continue
        option = action.option_strings[-1]
        if isinstance(value, bool):
            if value:
                output.append(option)
        elif isinstance(value, list):
            for item in value:
                output.extend([option, str(item)])
        else:
            output.extend([option, str(value)])
    for action in parser._actions:
        if action.option_strings:
            continue
        value = getattr(args, action.dest)
        if isinstance(value, list):
            output.extend(str(item) for item in value)
        elif value is not None:
            output.append(str(value))
    return output


if __name__ == "__main__":
    main()
