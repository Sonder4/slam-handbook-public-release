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
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--images-input", type=Path)
    parser.add_argument("--images-output", type=Path)
    args = parser.parse_args()

    content = args.input.read_bytes()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(content)
    if args.images_input and args.images_output:
        args.images_output.mkdir(parents=True, exist_ok=True)
        lines = content.decode("utf-8-sig").splitlines()
        starts = [
            next(
                index
                for index, line in enumerate(lines)
                if line.strip() == title and index > 500
            )
            for title in (
                "Factor Graphs for SLAM Frank Dellaert, Michael Kaess, and Timothy Barfoot",
                "Advanced State Variable Representations",
            )
        ]
        end = next(
            index
            for index in range(starts[1] + 1, len(lines))
            if lines[index].strip() == "# Robustness to Incorrect Data Association and Outliers"
        )
        image_names = {
            Path(match.group(1)).name
            for line in lines[starts[0] : end]
            for match in re.finditer(r"!\[\]\(([^)]+)\)", line)
        }
        for image in args.images_input.glob("*"):
            if image.name not in image_names:
                continue
            if image.is_file():
                shutil.copy2(image, args.images_output / image.name)
    print(f"Synced {args.output} ({len(content)} bytes, sha256={hashlib.sha256(content).hexdigest()})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
