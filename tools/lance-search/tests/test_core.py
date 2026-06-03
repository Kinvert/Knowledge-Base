import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import patch

from lance_search.core import (
    ChunkConfig,
    chunk_text,
    default_markdown_path,
    extract_pdf_chunks,
    iter_indexable_files,
    read_text_file,
    resolve_device,
)


class ChunkTextTests(unittest.TestCase):
    def test_chunks_text_with_line_numbers_and_overlap(self):
        text = "\n".join(f"line {idx}" for idx in range(1, 8))
        chunks = list(chunk_text("notes/example.txt", text, ChunkConfig(lines=3, overlap=1)))

        self.assertEqual(
            [(chunk.start_line, chunk.end_line, chunk.text.splitlines()) for chunk in chunks],
            [
                (1, 3, ["line 1", "line 2", "line 3"]),
                (3, 5, ["line 3", "line 4", "line 5"]),
                (5, 7, ["line 5", "line 6", "line 7"]),
            ],
        )


class FileDiscoveryTests(unittest.TestCase):
    def test_iter_indexable_files_skips_tool_outputs_and_keeps_docs(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / ".lance").mkdir()
            (root / ".lance" / "bin").mkdir()
            (root / ".lance" / "bin" / "python").write_text("binary", encoding="utf-8")
            (root / ".lancedb").mkdir()
            (root / ".lancedb" / "data.lance").write_text("index", encoding="utf-8")
            (root / "Software" / "AI").mkdir(parents=True)
            wanted = root / "Software" / "AI" / "Embeddings.md"
            wanted.write_text("Embedding search docs", encoding="utf-8")
            (root / "build").mkdir()
            (root / "build" / "hello.elf").write_bytes(b"\x7fELF")

            files = {path.relative_to(root).as_posix() for path in iter_indexable_files(root)}

        self.assertEqual(files, {"Software/AI/Embeddings.md"})

    def test_read_text_file_rejects_binary_content(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "data.bin"
            path.write_bytes(b"hello\x00world")

        self.assertIsNone(read_text_file(path))


class DeviceTests(unittest.TestCase):
    def test_resolve_device_preserves_explicit_device(self):
        self.assertEqual(resolve_device("cuda"), "cuda")
        self.assertEqual(resolve_device("cpu"), "cpu")


class PdfPathTests(unittest.TestCase):
    def test_default_markdown_path_places_pdf_text_next_to_source_by_default(self):
        pdf = Path("/docs/vector-search-guide.pdf")

        self.assertEqual(
            default_markdown_path(pdf, None),
            Path("/docs/vector-search-guide.md"),
        )

    def test_extract_pdf_chunks_reports_page_iteration_errors(self):
        class BrokenPages:
            def __iter__(self):
                raise RuntimeError("crypto dependency missing")

        class BrokenReader:
            pages = BrokenPages()

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            pdf = root / "docs" / "vector-search-guide.pdf"
            pdf.parent.mkdir()
            pdf.write_bytes(b"%PDF")
            fake_pypdf = types.SimpleNamespace(PdfReader=lambda _path: BrokenReader())
            with patch.dict(sys.modules, {"pypdf": fake_pypdf}):
                chunks = list(extract_pdf_chunks(root, pdf))

        self.assertEqual(len(chunks), 1)
        self.assertEqual(chunks[0].kind, "pdf_error")
        self.assertIn("crypto dependency missing", chunks[0].text)


if __name__ == "__main__":
    unittest.main()
