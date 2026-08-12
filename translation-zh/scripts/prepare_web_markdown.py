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
SUBFIGURE_LABEL_RE = re.compile(r"^\([a-z]\)\s*$", re.IGNORECASE)
SUBFIGURE_CAPTION_RE = re.compile(r"^\([a-z]\)(?:\s+.*)?$", re.IGNORECASE)
ENGLISH_FIGURE_REF_RE = re.compile(r"Figure\s+(?P<number>\d+\.\d+)\b")
ENGLISH_FORMULA_REF_RE = re.compile(
    r"(?P<label>Equation\s*\(?\s*(?P<number>\d+\.\d+[a-z]?)\s*\)?)"
)
TAG_RE = re.compile(r"\\tag\s*\{([^{}]+)\}")
CITATION_RE = re.compile(r"\[(\d+(?:\s*(?:[,;]|[-–—])\s*\d+)*)\]")
AUTHOR_RE = re.compile(r"^\*\*作者：\*\*")
FIGURE_REF_RE = re.compile(r"图\s*(?P<number>\d+\.\d+)\b")
FORMULA_REF_RE = re.compile(
    r"(?P<label>(?:式|公式)\s*[（(]\s*(?P<number>\d+\.\d+[a-z]?)\s*[）)])"
)
INLINE_MATH_RE = re.compile(r"(?<!\$)\$(?!\$).*?(?<!\$)\$")


def anchor(kind: str, number: str) -> str:
    return f"{kind}-{number.replace('.', '-')}"


def chapter_file(number: str) -> str:
    return f"chapter-{number.split('.', maxsplit=1)[0]}.md"


def display_formula_number(number: str) -> str:
    return f"式（{number}）"


def following_caption_index(lines: list[str], start: int) -> tuple[int | None, list[str]]:
    """Return the next caption candidate, retaining intervening subfigure labels."""
    labels: list[str] = []
    index = start
    while index < len(lines):
        candidate = lines[index].strip()
        if not candidate:
            index += 1
            continue
        if SUBFIGURE_LABEL_RE.match(candidate):
            labels.append(candidate)
            index += 1
            continue
        return index, labels
    return None, labels


def caption_after_figure_sequence(
    lines: list[str], start: int, caption_pattern: re.Pattern[str]
) -> int | None:
    """Find one caption shared by a sequence of subfigures.

    PDF extraction often puts the labels and six or more images before a
    single figure caption. The ordinary adjacent-caption path above should
    remain preferred, while this bounded scan covers those grouped figures.
    """
    for index in range(start, min(len(lines), start + 64)):
        candidate = lines[index].strip()
        if not candidate or IMAGE_RE.match(candidate) or SUBFIGURE_CAPTION_RE.match(candidate):
            continue
        if caption_pattern.match(candidate):
            return index
        return None
    return None


def discover_figures(lines: list[str]) -> list[str]:
    figures: list[str] = []
    for index, line in enumerate(lines):
        if IMAGE_RE.match(line.strip()):
            caption_index, _ = following_caption_index(lines, index + 1)
            caption = (
                CAPTION_RE.match(lines[caption_index].strip())
                if caption_index is not None
                else None
            )
            if caption is None:
                sequence_caption_index = caption_after_figure_sequence(
                    lines, index + 1, CAPTION_RE
                )
                caption = (
                    CAPTION_RE.match(lines[sequence_caption_index].strip())
                    if sequence_caption_index is not None
                    else None
                )
            if caption and caption.group(1) not in figures:
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
    if language == "zh":
        # Keep the Chinese chapter as the canonical page. The bilingual
        # reader opens in place instead of creating a duplicate nav entry.
        parallel = (
            f'<button type="button" class="counterpart-link bilingual-toggle" '
            f'data-bilingual-toggle data-english-url="../original-chapter-{chapter_number}/?embed=1" '
            'aria-expanded="false">中英对照阅读</button>'
        )
        return (
            '<nav class="chapter-reader-links" aria-label="章节阅读模式">'
            f"{counterpart}{parallel}"
            "</nav>"
        )
    return (
        '<nav class="chapter-reader-links" aria-label="章节阅读模式">'
        f"{counterpart}"
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
            caption_index, subfigure_labels = following_caption_index(lines, index + 1)
            caption = (
                CAPTION_RE.match(lines[caption_index].strip())
                if caption_index is not None
                else None
            )
            if caption:
                number = caption.group(1)
                output.extend([f'<a id="{anchor("fig", number)}" class="figure-anchor"></a>', ""])
            output.append(raw)
            if caption:
                for label in subfigure_labels:
                    output.extend(["", f"*{label}*"])
                output.extend(["", f"*{lines[caption_index].strip()}*"])
                index = caption_index + 1  # type: ignore[operator]
            else:
                index += 1
            continue

        if stripped == "":
            output.append("")
            index += 1
            continue

        caption = CAPTION_RE.match(stripped)
        if caption and caption.group(1) in local_figures:
            number = caption.group(1)
            output.extend(
                [
                    f'<a id="{anchor("fig", number)}" class="figure-anchor"></a>',
                    "",
                    f"*{raw}*",
                    "",
                ]
            )
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
        "3": "# Robustness to Incorrect Data Association and Outliers",
        "4": "Chen Wang, Krishna Murthy Jatavallabhula, and Mustafa Mukadam",
        "5": "Dense Map Representations",
        "6": "# Certifiably Optimal Solvers and Theoretical Properties of SLAM",
        "7": "Visual SLAM",
        "8": "LiDAR SLAM",
        "9": "Radar SLAM",
        "10": "Event-based SLAM",
        "11": "Inertial Odometry for SLAM",
        "12": "Leg Odometry for SLAM",
        "13": "# Boosting SLAM with Deep Learning",
        "14": "# Map Representations with Diferentiable Volume Rendering",
        "15": "# Dynamic and Deformable SLAM",
        "16": "Metric-Semantic SLAM",
        "17": "# 17 Towards Open-World Spatial AI",
        "18": "The Computational Structure of Spatial AI Systems Andrew J. Davison",
    }
    def matches_marker(line: str, marker: str) -> bool:
        value = line.strip()
        return value == marker or value.startswith(marker + " ")

    start = next(
        index
        for index, line in enumerate(lines)
        if matches_marker(line, chapter_titles[chapter_number]) and index > 500
    )
    next_title = (
        chapter_titles[str(int(chapter_number) + 1)]
        if chapter_number != "6" and chapter_number != "18"
        else ("PART II SLAM IN PRACTICE" if chapter_number == "6" else "# Epilogue")
    )
    end = next(
        index
        for index in range(start + 1, len(lines))
        if matches_marker(lines[index], next_title)
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
    "3": {
        "source_title": "# Robustness to Incorrect Data Association and Outliers",
        "title": "Robustness to Incorrect Data Association and Outliers",
        "authors": "Heng Yang, Josh Mangelson, Yun Chang, Jingnan Shi, Niko Sunderhauf, and Luca Carlone",
    },
    "4": {
        "source_title": "Chen Wang, Krishna Murthy Jatavallabhula, and Mustafa Mukadam",
        "title": "Differentiable Optimization",
        "authors": "Chen Wang, Krishna Murthy Jatavallabhula, and Mustafa Mukadam",
    },
    "5": {
        "source_title": "Dense Map Representations",
        "title": "Dense Map Representations",
        "authors": "Victor Reijgwart, Jens Behley, Teresa Vidal-Calleja, Helen Oleynikova, Lionel Ott, Cyrill Stachniss and Ayoung Kim",
    },
    "6": {
        "source_title": "# Certifiably Optimal Solvers and Theoretical Properties of SLAM",
        "title": "Certifiably Optimal Solvers and Theoretical Properties of SLAM",
        "authors": "David M. Rosen, Kasra Khosoussi, Connor Holmes, Gamini Dissanayake, Timothy Barfoot, and Luca Carlone",
    },
    "7": {
        "source_title": "Visual SLAM",
        "title": "Visual SLAM",
        "authors": "Jakob Engel, Juan D. Tardós, Javier Civera, Margarita Chli, Stefan Leutenegger, Frank Dellaert, and Daniel Cremers",
    },
    "8": {
        "source_title": "LiDAR SLAM",
        "title": "LiDAR SLAM",
        "authors": "Jens Behley, Maurice Fallon, Shibo Zhao, Giseop Kim, Ji Zhang, Fu Zhang, and Ayoung Kim",
    },
    "9": {
        "source_title": "Radar SLAM",
        "title": "Radar SLAM",
        "authors": "Martin Magnusson, Christoffer Heckman, Henrik Andreasson, Ayoung Kim, Timothy Barfoot, Michael Kaess, and Paul Newman",
    },
    "10": {
        "source_title": "Event-based SLAM",
        "title": "Event-based SLAM",
        "authors": "Guillermo Gallego, Javier Hidalgo-Carrió, and Davide Scaramuzza",
    },
    "11": {
        "source_title": "Inertial Odometry for SLAM",
        "title": "Inertial Odometry for SLAM",
        "authors": "Guoquan (Paul) Huang, Cédric Le Gentil, Teresa Vidal-Calleja, Davide Scaramuzza, Frank Dellaert, and Luca Carlone",
    },
    "12": {
        "source_title": "Leg Odometry for SLAM",
        "title": "Leg Odometry for SLAM",
        "authors": "Marco Camurri and Matías Mattamala",
    },
    "13": {
        "source_title": "# Boosting SLAM with Deep Learning",
        "title": "Boosting SLAM with Deep Learning",
        "authors": "Zachary Teed, Jia Deng, Boris Chidlovskii, Jérôme Revaud, Felix Wimbauer, and Daniel Cremers",
    },
    "14": {
        "source_title": "# Map Representations with Diferentiable Volume Rendering",
        "title": "Map Representations with Differentiable Volume Rendering",
        "authors": "Hidenobu Matsuki and Andrew J. Davison",
    },
    "15": {
        "source_title": "# Dynamic and Deformable SLAM",
        "title": "Dynamic and Deformable SLAM",
        "authors": "Lukas Schmid, Jose Maria Martinez Montiel, Shoudong Huang, Daniel Cremers, Jose Neira, and Javier Civera",
    },
    "16": {
        "source_title": "Metric-Semantic SLAM",
        "title": "Metric-Semantic SLAM",
        "authors": "Arash Asgharivaskasi, Kevin Doherty, Jens Behley, Nathan Hughes, Yun Chang, John Leonard, Henrik I. Christensen, Luca Carlone, and Nikolay Atanasov",
    },
    "17": {
        "source_title": "# 17 Towards Open-World Spatial AI",
        "title": "Towards Open-World Spatial AI",
        "authors": "Liam Paull, Sacha Morin, Dominic Maggio, Martin Büchner, Cesar Cadena, Abhinav Valada, and Luca Carlone",
    },
    "18": {
        "source_title": "The Computational Structure of Spatial AI Systems Andrew J. Davison",
        "title": "The Computational Structure of Spatial AI Systems",
        "authors": "Andrew J. Davison",
    },
}


def transform_english_chapter(lines: list[str], chapter_number: str) -> str:
    start, end = source_chapter_range(lines, chapter_number)
    chapter = lines[start:end]
    metadata = ENGLISH_CHAPTER_METADATA[chapter_number]
    title = metadata["title"]
    authors = metadata["authors"]
    source_title = metadata.get("source_title")
    if source_title and any(line.strip() == source_title for line in chapter):
        body_start = next(
            index for index, line in enumerate(chapter) if line.strip() == source_title
        )
        body = chapter[body_start + 1 :]
    else:
        body = chapter
    while body and not body[0].strip():
        body = body[1:]
    if body and body[0].strip() == authors:
        body = body[1:]
    # Repair unambiguous superscript artifacts introduced by PDF extraction.
    body = [
        re.sub(r"\$\^\s*\{\s*([xb])\s*,?\s*\}\$", r"$\\boldsymbol{\1}$", line)
        for line in body
    ]
    local_figures: set[str] = set()
    for image_index, line in enumerate(body):
        if not IMAGE_RE.match(line.strip()):
            continue
        caption_index, _ = following_caption_index(body, image_index + 1)
        if caption_index is not None:
            caption = re.search(r"Figure\s+(\d+\.\d+)\b", body[caption_index])
            if caption and ENGLISH_CAPTION_RE.match(body[caption_index].strip()):
                local_figures.add(caption.group(1))
        if caption_index is None or not ENGLISH_CAPTION_RE.match(body[caption_index].strip()):
            sequence_caption_index = caption_after_figure_sequence(
                body, image_index + 1, ENGLISH_CAPTION_RE
            )
            if sequence_caption_index is not None:
                caption = re.search(r"Figure\s+(\d+\.\d+)\b", body[sequence_caption_index])
                if caption:
                    local_figures.add(caption.group(1))
    # Some PDF extraction orders a short figure caption before a display
    # equation and its image (notably Figure 5.2). Treat it as a caption
    # only when an image follows immediately in the extracted block.
    for caption_index, line in enumerate(body):
        caption = re.match(r"^Figure\s+(\d+\.\d+)\b", line.strip())
        if not caption:
            continue
        if any(IMAGE_RE.match(candidate.strip()) for candidate in body[max(0, caption_index - 5) : caption_index]):
            continue
        lookahead = body[caption_index + 1 : caption_index + 9]
        if any(IMAGE_RE.match(candidate.strip()) for candidate in lookahead):
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
        elif (
            (caption := re.match(r"^Figure\s+(\d+\.\d+)\b", stripped))
            and caption.group(1) in local_figures
            and not any(IMAGE_RE.match(line.strip()) for line in body[max(0, index - 5) : index])
            and any(IMAGE_RE.match(line.strip()) for line in body[index + 1 : index + 9])
        ):
            output.extend(
                [
                    f'<a id="{anchor("fig", caption.group(1))}" class="figure-anchor"></a>',
                    "",
                    f"*{raw}*",
                    "{ .figure-caption }",
                    "",
                ]
            )
        elif (
            (caption := re.match(r"^Figure\s+(\d+\.\d+)\b", stripped))
            and caption.group(1) in local_figures
        ):
            output.extend(
                [
                    f'<a id="{anchor("fig", caption.group(1))}" class="figure-anchor"></a>',
                    "",
                    f"*{raw}*",
                    "{ .figure-caption }",
                    "",
                ]
            )
        elif IMAGE_RE.match(stripped):
            caption_match = None
            caption_index, subfigure_labels = following_caption_index(body, index + 1)
            if caption_index is not None and ENGLISH_CAPTION_RE.match(body[caption_index].strip()):
                caption_match = re.search(r"Figure\s+(\d+\.\d+)", body[caption_index].strip())
            if caption_match:
                output.extend([f'<a id="{anchor("fig", caption_match.group(1))}" class="figure-anchor"></a>', ""])
            output.append(raw)
            if caption_match:
                for label in subfigure_labels:
                    output.extend([f"*{label}*", ""])
                output.extend([f"*{body[caption_index].strip()}*", "{ .figure-caption }", ""])
                index = caption_index  # type: ignore[assignment]
        else:
            output.extend([replace_english_references(raw), "{ .book-paragraph }", ""])
        index += 1
    return "\n".join(output).rstrip() + "\n"


def make_parallel_reader(chapter_number: str) -> str:
    """Create a single-page, side-by-side reader from the canonical pages."""

    return "\n".join(
        [
            f"# 第 {chapter_number} 章中英对照阅读",
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
    parser.add_argument(
        "--chapters",
        nargs="+",
        help="Approved chapter numbers to publish. Defaults to every translated chapter.",
    )
    args = parser.parse_args()

    source_dir = args.source.resolve()
    output_dir = args.output.resolve()
    if source_dir == output_dir:
        raise ValueError("Website output directory must differ from the source directory.")
    available_chapters = {
        path.stem.rsplit("-", 1)[1]: path for path in source_dir.glob("chapter-*.md")
    }
    requested_chapters = args.chapters or available_chapters.keys()
    missing_chapters = [number for number in requested_chapters if number not in available_chapters]
    if missing_chapters:
        raise ValueError(
            "Approved chapter source is missing: " + ", ".join(sorted(missing_chapters, key=int))
        )
    chapters = [available_chapters[number] for number in requested_chapters]
    chapters.sort(key=lambda path: int(path.stem.rsplit("-", 1)[1]))
    if not chapters:
        raise ValueError(f"No chapter Markdown files found in {source_dir}")

    chapter_lines = {path: path.read_text(encoding="utf-8-sig").splitlines() for path in chapters}
    all_figures = {number for lines in chapter_lines.values() for number in discover_figures(lines)}
    all_formulas = {
        number for lines in chapter_lines.values() for number in discover_formulas("\n".join(lines))
    }
    if len(all_formulas) != len(set(all_formulas)):
        raise ValueError("Duplicate tagged equations found across translated chapters.")

    if output_dir.exists():
        shutil.rmtree(output_dir)
    shutil.copytree(source_dir, output_dir)
    shutil.rmtree(output_dir / "original", ignore_errors=True)
    published_numbers = {chapter.stem.rsplit("-", 1)[1] for chapter in chapters}
    for unpublished_chapter in output_dir.glob("chapter-*.md"):
        if unpublished_chapter.stem.rsplit("-", 1)[1] not in published_numbers:
            unpublished_chapter.unlink()
    for chapter in chapters:
        destination = output_dir / chapter.name
        destination.write_text(
            transform_chapter(chapter, all_figures, all_formulas), encoding="utf-8"
        )
    english_lines = args.english_source.read_text(encoding="utf-8-sig").splitlines()
    chapter_numbers = [path.stem.rsplit("-", 1)[1] for path in chapters]
    for chapter_number in chapter_numbers:
        if chapter_number not in ENGLISH_CHAPTER_METADATA:
            raise ValueError(f"English source metadata is missing for Chapter {chapter_number}.")
        destination = output_dir / f"original-chapter-{chapter_number}.md"
        destination.write_text(
            transform_english_chapter(english_lines, chapter_number), encoding="utf-8"
        )
        # Bilingual reading is embedded into the canonical Chinese page.
        parallel_page = output_dir / f"parallel-chapter-{chapter_number}.md"
        if parallel_page.exists():
            parallel_page.unlink()

    print(
        f"Prepared {len(chapters)} Chinese chapters, {len(chapter_numbers)} English chapters, "
        f"{len(all_formulas)} equations, and {len(all_figures)} figures."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
