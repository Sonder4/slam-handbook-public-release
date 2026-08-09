#!/usr/bin/env python3
"""Prepare canonical translated Markdown for MathJax-enabled MkDocs pages."""

from __future__ import annotations

import argparse
import html
import re
import shutil
from pathlib import Path


IMAGE_RE = re.compile(r"^!\[\]\(([^)]+)\)\s*$")
CAPTION_RE = re.compile(r"^图\s+(\d+\.\d+)\b")
TAG_RE = re.compile(r"\\tag\s*\{([^{}]+)\}")
FIGURE_REF_RE = re.compile(r"图\s*(?P<number>[12]\.\d+)\b")
FORMULA_REF_RE = re.compile(
    r"(?P<label>(?:式|公式)\s*[（(]\s*(?P<number>[12]\.\d+[a-z]?)\s*[）)])"
)
INLINE_MATH_RE = re.compile(r"(?<!\$)\$(?!\$).*?(?<!\$)\$")


def anchor(kind: str, number: str) -> str:
    return f"{kind}-{number.replace('.', '-')}"


def chapter_file(number: str) -> str:
    return f"chapter-{number.split('.', maxsplit=1)[0]}.md"


def display_formula_number(number: str) -> str:
    return f"式（{number}）"


def discover_figures(lines: list[str]) -> list[str]:
    figures: list[str] = []
    for index, line in enumerate(lines):
        if IMAGE_RE.match(line.strip()):
            caption_index = index + 1
            while caption_index < len(lines) and not lines[caption_index].strip():
                caption_index += 1
            caption = (
                CAPTION_RE.match(lines[caption_index].strip())
                if caption_index < len(lines)
                else None
            )
            if caption:
                figures.append(caption.group(1))
    return figures


def discover_formulas(text: str) -> list[str]:
    return [match.group(1).strip() for match in TAG_RE.finditer(text)]


def make_href(current_file: str, target_file: str, target_anchor: str) -> str:
    return f"#{target_anchor}" if current_file == target_file else f"{target_file}#{target_anchor}"


def replace_references(
    line: str,
    current_file: str,
    figures: set[str],
    formulas: set[str],
) -> str:
    """Link prose references while leaving inline mathematics untouched."""

    def replace_prose(prose: str) -> str:
        def figure_link(match: re.Match[str]) -> str:
            number = match.group("number")
            if number not in figures:
                return match.group(0)
            href = make_href(current_file, chapter_file(number), anchor("fig", number))
            return f"[{match.group(0)}]({href})"

        def formula_link(match: re.Match[str]) -> str:
            number = match.group("number")
            if number not in formulas:
                return match.group(0)
            href = make_href(current_file, chapter_file(number), anchor("eq", number))
            return f"[{match.group('label')}]({href})"

        prose = FORMULA_REF_RE.sub(formula_link, prose)
        return FIGURE_REF_RE.sub(figure_link, prose)

    parts: list[str] = []
    start = 0
    for match in INLINE_MATH_RE.finditer(line):
        parts.append(replace_prose(line[start:match.start()]))
        parts.append(match.group(0))
        start = match.end()
    parts.append(replace_prose(line[start:]))
    return "".join(parts)


def make_reference_index(figures: list[str], formulas: list[str]) -> list[str]:
    figure_links = " · ".join(
        f'<a href="#{anchor("fig", number)}">图 {html.escape(number)}</a>' for number in figures
    )
    formula_links = " · ".join(
        f'<a href="#{anchor("eq", number)}">{html.escape(display_formula_number(number))}</a>'
        for number in formulas
    )
    return [
        '<details class="reference-index">',
        "<summary>图表与公式索引</summary>",
        f"<p><strong>图</strong>：{figure_links}</p>",
        f"<p><strong>公式</strong>：{formula_links}</p>",
        "</details>",
        "",
    ]


def transform_chapter(
    source: Path,
    all_figures: set[str],
    all_formulas: set[str],
) -> str:
    lines = source.read_text(encoding="utf-8-sig").splitlines()
    local_figures = discover_figures(lines)
    local_formulas = discover_formulas("\n".join(lines))
    output: list[str] = []
    current_file = source.name
    index = 0
    inserted_index = False

    while index < len(lines):
        raw = lines[index]
        stripped = raw.strip()

        if not inserted_index and raw.startswith("# "):
            output.append(raw)
            output.append("")
            output.extend(make_reference_index(local_figures, local_formulas))
            inserted_index = True
            index += 1
            continue

        if stripped == "$$":
            formula_lines: list[str] = []
            index += 1
            while index < len(lines) and lines[index].strip() != "$$":
                formula_lines.append(lines[index])
                index += 1
            if index >= len(lines):
                raise ValueError(f"Unclosed display formula in {source}")
            formula = "\n".join(formula_lines).strip()
            formula = re.sub(r"\\tag\s*\{", r"\\tag{", formula)
            tagged = TAG_RE.search(formula)
            if tagged:
                output.extend(
                    [
                        f'<a id="{anchor("eq", tagged.group(1).strip())}" class="equation-anchor"></a>',
                        "",
                    ]
                )
            output.extend(["$$", formula, "$$"])
            index += 1
            continue

        image = IMAGE_RE.match(stripped)
        if image:
            caption_index = index + 1
            while caption_index < len(lines) and not lines[caption_index].strip():
                caption_index += 1
            caption = (
                CAPTION_RE.match(lines[caption_index].strip())
                if caption_index < len(lines)
                else None
            )
            if caption:
                number = caption.group(1)
                output.extend([f'<a id="{anchor("fig", number)}" class="figure-anchor"></a>', ""])
            output.append(raw)
            if caption:
                output.extend(["", f"*{lines[caption_index].strip()}*"])
                index = caption_index + 1
            else:
                index += 1
            continue

        output.append(replace_references(raw, current_file, all_figures, all_formulas))
        index += 1

    return "\n".join(output) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    source_dir = args.source.resolve()
    output_dir = args.output.resolve()
    if source_dir == output_dir:
        raise ValueError("Website output directory must differ from the source directory.")
    chapters = sorted(source_dir.glob("chapter-*.md"))
    if not chapters:
        raise ValueError(f"No chapter Markdown files found in {source_dir}")

    chapter_lines = {path: path.read_text(encoding="utf-8-sig").splitlines() for path in chapters}
    all_figures = {number for lines in chapter_lines.values() for number in discover_figures(lines)}
    all_formulas = {
        number for lines in chapter_lines.values() for number in discover_formulas("\n".join(lines))
    }
    if len(all_formulas) != 148:
        raise ValueError(f"Expected 148 tagged equations, found {len(all_formulas)}")

    if output_dir.exists():
        shutil.rmtree(output_dir)
    shutil.copytree(source_dir, output_dir)
    for chapter in chapters:
        destination = output_dir / chapter.name
        destination.write_text(
            transform_chapter(chapter, all_figures, all_formulas), encoding="utf-8"
        )

    print(f"Prepared {len(chapters)} chapters, {len(all_formulas)} equations, and {len(all_figures)} figures.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
