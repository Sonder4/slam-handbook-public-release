#!/usr/bin/env python3
"""Prepare Chinese and English chapter Markdown for the MkDocs website."""

from __future__ import annotations

import argparse
import html
import re
import shutil
from pathlib import Path


IMAGE_RE = re.compile(r"^!\[\]\(([^)]+)\)\s*$")
CAPTION_RE = re.compile(r"^图\s+(\d+\.\d+)\b")
ENGLISH_CAPTION_RE = re.compile(r"^Figure\s+\d+\.\d+\b")
ENGLISH_FIGURE_REF_RE = re.compile(r"Figure\s+(?P<number>[12]\.\d+)\b")
ENGLISH_FORMULA_REF_RE = re.compile(
    r"(?P<label>Equation\s*\(?\s*(?P<number>[12]\.\d+[a-z]?)\s*\)?)"
)
TAG_RE = re.compile(r"\\tag\s*\{([^{}]+)\}")
CITATION_RE = re.compile(r"\[(\d+(?:\s*[,;]\s*\d+)*)\]")
AUTHOR_RE = re.compile(r"^\*\*作者：\*\*")
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
        prose = FIGURE_REF_RE.sub(figure_link, prose)
        return CITATION_RE.sub(
            lambda match: f'<sup class="citation">[{match.group(1)}]</sup>',
            prose,
        )

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


def counterpart_link(chapter_number: str, language: str) -> str:
    # MkDocs resolves Markdown links relative to the current document.
    # Keep the counterpart target extensionless so Material's URL rewriting
    # remains correct both locally and under the repository Pages base path.
    if language == "zh":
        counterpart = (
            f'<a class="counterpart-link" href="../original-chapter-{chapter_number}/">'
            "English original</a>"
        )
    else:
        counterpart = (
            f'<a class="counterpart-link" href="../chapter-{chapter_number}/">'
            "中文译文</a>"
        )
    return (
        '<nav class="chapter-reader-links" aria-label="章节阅读模式">'
        f"{counterpart}"
        f'<a class="counterpart-link" href="../parallel-chapter-{chapter_number}/">'
        "中英对照阅读</a>"
        "</nav>"
    )


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

        if raw.startswith("#"):
            output.extend([raw, ""])
            if not inserted_index and raw.startswith("# "):
                chapter_number = source.stem.rsplit("-", maxsplit=1)[-1]
                output.extend([counterpart_link(chapter_number, "zh"), ""])
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

        if stripped == "":
            output.append("")
            index += 1
            continue

        line = replace_references(raw, current_file, all_figures, all_formulas)
        if AUTHOR_RE.match(stripped):
            output.extend([line, "{ .chapter-authors }"])
        else:
            output.extend([line, "{ .book-paragraph }"])
        index += 1

    return "\n".join(output) + "\n"


def source_chapter_range(lines: list[str], chapter_number: str) -> tuple[int, int]:
    chapter_titles = {
        "1": "Factor Graphs for SLAM Frank Dellaert, Michael Kaess, and Timothy Barfoot",
        "2": "Advanced State Variable Representations",
    }
    start = next(
        index
        for index, line in enumerate(lines)
        if line.strip() == chapter_titles[chapter_number]
        and index > 500
    )
    next_title = (
        "Advanced State Variable Representations"
        if chapter_number == "1"
        else "# Robustness to Incorrect Data Association and Outliers"
    )
    end = next(
        index
        for index in range(start + 1, len(lines))
        if lines[index].strip() == next_title
        and index > start + 100
    )
    return start, end


ENGLISH_CHAPTER_METADATA = {
    "1": {
        "source_title": "Factor Graphs for SLAM Frank Dellaert, Michael Kaess, and Timothy Barfoot",
        "title": "Factor Graphs for SLAM",
        "authors": "Frank Dellaert, Michael Kaess, and Timothy Barfoot",
    },
    "2": {
        "source_title": "Advanced State Variable Representations",
        "title": "Advanced State Variable Representations",
        "authors": "Timothy Barfoot, Frank Dellaert, Michael Kaess, and Jose Luis Blanco-Claraco",
    },
}

PARALLEL_CHAPTER_TITLES = {
    "1": "第 1 章中英对照阅读",
    "2": "第 2 章中英对照阅读",
}


def transform_english_chapter(lines: list[str], chapter_number: str) -> str:
    start, end = source_chapter_range(lines, chapter_number)
    chapter = lines[start:end]
    metadata = ENGLISH_CHAPTER_METADATA[chapter_number]
    title = metadata["title"]
    authors = metadata["authors"]
    body_start = next(
        index for index, line in enumerate(chapter) if line.strip() == metadata["source_title"]
    )
    body = chapter[body_start + 1 :]
    while body and not body[0].strip():
        body = body[1:]
    if body and body[0].strip() == authors:
        body = body[1:]
    local_figures: set[str] = set()
    for image_index, line in enumerate(body):
        if not IMAGE_RE.match(line.strip()):
            continue
        caption_index = image_index + 1
        while caption_index < len(body) and not body[caption_index].strip():
            caption_index += 1
        if caption_index < len(body):
            caption = re.search(r"Figure\s+(\d+\.\d+)\b", body[caption_index])
            if caption and ENGLISH_CAPTION_RE.match(body[caption_index].strip()):
                local_figures.add(caption.group(1))
    local_formulas = set(discover_formulas("\n".join(body)))

    def replace_english_references(line: str) -> str:
        line = ENGLISH_FIGURE_REF_RE.sub(
            lambda match: f'[{match.group(0)}](#{anchor("fig", match.group("number"))})'
            if match.group("number") in local_figures
            else match.group(0),
            line,
        )
        line = ENGLISH_FORMULA_REF_RE.sub(
            lambda match: f'[{match.group("label")}]({"#" + anchor("eq", match.group("number"))})'
            if match.group("number") in local_formulas
            else match.group(0),
            line,
        )
        return replace_references(line, "", set(), set())
    output = [
        f"# {title}",
        "",
        counterpart_link(chapter_number, "en"),
        "",
        authors,
        "{ .chapter-authors }",
        "",
    ]
    index = 0
    while index < len(body):
        raw = body[index]
        stripped = raw.strip()
        if not stripped:
            output.append("")
        elif stripped.startswith("#"):
            output.extend([stripped, ""])
        elif stripped == "$$":
            formula_lines = []
            index += 1
            while index < len(body) and body[index].strip() != "$$":
                formula_lines.append(body[index])
                index += 1
            if index >= len(body):
                raise ValueError(f"Unclosed English display formula in chapter {chapter_number}")
            formula = "\n".join(formula_lines).strip()
            tagged = TAG_RE.search(formula)
            if tagged:
                output.extend([f'<a id="{anchor("eq", tagged.group(1).strip())}" class="equation-anchor"></a>', ""])
            output.extend(["$$", formula, "$$"])
        elif IMAGE_RE.match(stripped):
            caption_match = None
            caption_index = index + 1
            while caption_index < len(body) and not body[caption_index].strip():
                output.append("")
                caption_index += 1
            if caption_index < len(body) and ENGLISH_CAPTION_RE.match(body[caption_index].strip()):
                caption_match = re.search(r"Figure\s+(\d+\.\d+)", body[caption_index].strip())
            if caption_match:
                output.extend([f'<a id="{anchor("fig", caption_match.group(1))}" class="figure-anchor"></a>', ""])
            output.append(raw)
            if caption_match:
                output.extend([f"*{body[caption_index].strip()}*", "{ .figure-caption }", ""])
                index = caption_index
        else:
            output.extend([replace_english_references(raw), "{ .book-paragraph }", ""])
        index += 1
    return "\n".join(output).rstrip() + "\n"


def make_parallel_reader(chapter_number: str) -> str:
    """Create a single-page, side-by-side reader from the canonical pages."""

    return "\n".join(
        [
            f"# {PARALLEL_CHAPTER_TITLES[chapter_number]}",
            "",
            '<nav class="chapter-reader-links" aria-label="章节阅读模式">',
            f'<a href="../chapter-{chapter_number}/">中文单页</a>',
            f'<a href="../original-chapter-{chapter_number}/">English original</a>',
            "</nav>",
            "",
            '<div class="parallel-iframe-reader" data-parallel-reader>',
            '  <section class="parallel-iframe-pane" lang="zh-CN">',
            '    <h2 class="parallel-iframe-heading">中文译文</h2>',
            (
                f'    <iframe title="第 {chapter_number} 章中文译文" '
                f'src="../chapter-{chapter_number}/?embed=1" loading="eager"></iframe>'
            ),
            "  </section>",
            '  <section class="parallel-iframe-pane" lang="en">',
            '    <h2 class="parallel-iframe-heading">English original</h2>',
            (
                f'    <iframe title="Chapter {chapter_number} English original" '
                f'src="../original-chapter-{chapter_number}/?embed=1" loading="eager"></iframe>'
            ),
            "  </section>",
            "</div>",
            "",
        ]
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--english-source", type=Path, required=True)
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
    shutil.rmtree(output_dir / "original", ignore_errors=True)
    for chapter in chapters:
        destination = output_dir / chapter.name
        destination.write_text(
            transform_chapter(chapter, all_figures, all_formulas), encoding="utf-8"
        )
    english_lines = args.english_source.read_text(encoding="utf-8-sig").splitlines()
    for chapter_number in ("1", "2"):
        destination = output_dir / f"original-chapter-{chapter_number}.md"
        destination.write_text(
            transform_english_chapter(english_lines, chapter_number), encoding="utf-8"
        )
        (output_dir / f"parallel-chapter-{chapter_number}.md").write_text(
            make_parallel_reader(chapter_number), encoding="utf-8"
        )

    print(
        f"Prepared {len(chapters)} Chinese chapters, 2 English chapters, "
        f"{len(all_formulas)} equations, and {len(all_figures)} figures."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
