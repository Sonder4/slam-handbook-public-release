#!/usr/bin/env python3
"""Expand temporary equation markers in translated chapter Markdown.

The source equations are copied verbatim so that translation edits cannot
silently alter symbols, tags, or matrix layouts.
"""
from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "content" / "original" / "full.md"
MARKER = re.compile(r"\{\{EQS:(\d+)\.(\d+)(?:-(?:\d+\.)?(\d+))?\}\}")
TABLE_MARKER = re.compile(r"\{\{TABLE:(\d+)\.(\d+)\}\}")
BLOCK = re.compile(r"\$\$\n(.*?)\n\$\$", re.S)
TAG = re.compile(r"\\tag\s*\{(\d+)\.(\d+)\}")
TABLE = re.compile(r"<table>.*?</table>", re.S)


def main() -> int:
    source = SOURCE.read_text(encoding="utf-8")
    equations: dict[tuple[int, int], str] = {}
    for match in BLOCK.finditer(source):
        tag = TAG.search(match.group(1))
        if tag:
            key = (int(tag.group(1)), int(tag.group(2)))
            equations[key] = match.group(0)
    print(f"source equation blocks: {len(equations)}")
    chapter10_start = source.find("Event-based SLAM")
    chapter10_end = source.find("Inertial Odometry for SLAM", chapter10_start + 1)
    tables = TABLE.findall(source[chapter10_start:chapter10_end])
    print(f"chapter10 tables: {len(tables)}")

    for chapter in (10, 11, 12):
        path = ROOT / "content" / f"chapter-{chapter}.md"
        if not path.exists():
            continue
        text = path.read_text(encoding="utf-8")

        def expand(match: re.Match[str]) -> str:
            c = int(match.group(1))
            first = int(match.group(2))
            last = int(match.group(3) or first)
            blocks = [equations[(c, n)] for n in range(first, last + 1)]
            return "\n\n".join(blocks)

        expanded = MARKER.sub(expand, text)
        if chapter == 10:
            table_index = 0

            def expand_table(match: re.Match[str]) -> str:
                nonlocal table_index
                table_index += 1
                return tables[table_index - 1]

            expanded = TABLE_MARKER.sub(expand_table, expanded)
        path.write_text(expanded, encoding="utf-8")
        print(f"Expanded equations in {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
