#!/usr/bin/env python3
"""Extract Chapters 7--18 from the canonical handbook PDF for translation."""

from __future__ import annotations

import argparse
from pathlib import Path

import pymupdf


CHAPTER_PAGES = {
    7: (209, 239),
    8: (240, 265),
    9: (266, 297),
    10: (298, 319),
    11: (320, 348),
    12: (349, 374),
    13: (382, 412),
    14: (413, 432),
    15: (433, 469),
    16: (470, 505),
    17: (506, 536),
    18: (537, 563),
}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    document = pymupdf.open(args.input)
    args.output.mkdir(parents=True, exist_ok=True)
    for chapter, (start, end) in CHAPTER_PAGES.items():
        text = "\n\n".join(document[page - 1].get_text() for page in range(start, end + 1))
        output = args.output / f"chapter-{chapter}-raw.txt"
        output.write_text(text, encoding="utf-8")
        print(f"Extracted Chapter {chapter}: PDF pages {start}-{end} -> {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
