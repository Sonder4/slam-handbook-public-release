#!/usr/bin/env python3
"""Copy the authorized extracted English handbook source into the project."""

from __future__ import annotations

import argparse
import hashlib
import re
import shutil
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", nargs="+", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--images-input", nargs="+", type=Path)
    parser.add_argument("--images-output", type=Path)
    args = parser.parse_args()

    content = "\n".join(path.read_text(encoding="utf-8-sig") for path in args.input).encode("utf-8")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(content)
    if args.images_input and args.images_output:
        args.images_output.mkdir(parents=True, exist_ok=True)
        lines = content.decode("utf-8-sig").splitlines()
        image_names = {
            Path(match.group(1)).name
            for line in lines
            for match in re.finditer(r"!\[\]\(([^)]+)\)", line)
        }
        copied = 0
        for image_dir in args.images_input:
            for image in image_dir.glob("*"):
                if image.name not in image_names or not image.is_file():
                    continue
                shutil.copy2(image, args.images_output / image.name)
                copied += 1
    else:
        copied = 0
    print(
        f"Synced {args.output} ({len(content)} bytes, "
        f"sha256={hashlib.sha256(content).hexdigest()}, images={copied})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
