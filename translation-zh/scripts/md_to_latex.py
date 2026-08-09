#!/usr/bin/env python3
"""Convert the chapter Markdown translation to standalone LaTeX chapter files."""

from __future__ import annotations

import argparse
import re
from pathlib import Path


IMAGE_RE = re.compile(r"^!\[\]\(([^)]+)\)\s*$")
TABLE_ROW_RE = re.compile(r"^\s*\|.*\|\s*$")
TABLE_SEPARATOR_RE = re.compile(r"^\s*\|?\s*:?-{3,}:?\s*(?:\|\s*:?-{3,}:?\s*)+\|?\s*$")
TAG_RE = re.compile(r"\\tag\s*\{([^{}]+)\}")
MATH_RE = re.compile(r"(?<!\$)\$(?!\$)(.+?)(?<!\$)\$")
BOLD_RE = re.compile(r"\*\*(.+?)\*\*")
SUP_RE = re.compile(r"<sup>(.+?)</sup>")
CITATION_RE = re.compile(r"\[(\d+(?:\s*[,;]\s*\d+)*)\]")


def escape_text(text: str) -> str:
    """Escape LaTeX text while preserving inline math and simple emphasis."""
    placeholders: list[str] = []

    def hold(value: str) -> str:
        marker = f"@@CODEXHOLD{len(placeholders)}@@"
        placeholders.append(value)
        return marker

    text = BOLD_RE.sub(lambda m: hold(r"\textbf{" + escape_text(m.group(1)) + "}"), text)
    text = SUP_RE.sub(lambda m: hold(r"\textsuperscript{" + escape_text(m.group(1)) + "}"), text)
    text = MATH_RE.sub(lambda m: hold("$" + normalize_math(m.group(1)) + "$"), text)
    # Keep literal numeric bibliography references while matching book-style superscripts.
    text = CITATION_RE.sub(lambda m: hold(r"\textsuperscript{[" + m.group(1) + "]}"), text)

    replacements = {
        "\\": r"\textbackslash{}",
        "%": r"\%",
        "&": r"\&",
        "#": r"\#",
        "_": r"\_",
        "{": r"\{",
        "}": r"\}",
    }
    for source, target in replacements.items():
        text = text.replace(source, target)

    for index, value in enumerate(placeholders):
        text = text.replace(f"@@CODEXHOLD{index}@@", value)
    return text


def normalize_formula(formula: str) -> str:
    return normalize_math(re.sub(r"\\tag\s*\{", r"\\tag{", formula.strip()))


def normalize_math(formula: str) -> str:
    """Repair shorthand combinations that fail with amsbsy and math alphabets."""
    # Always give math alphabet commands an explicit argument. This also protects
    # nested forms such as \boldsymbol\Delta_{\boldsymbol\mathcal X_i}.
    for alphabet in ("mathcal", "mathbb", "mathrm"):
        formula = re.sub(
            rf"\\{alphabet}\s+([A-Za-z])",
            rf"\\{alphabet}{{\1}}",
            formula,
        )
    for prefix in ("boldsymbol", "pmb"):
        for alphabet in ("mathcal", "mathbb", "mathrm"):
            # \boldsymbol\mathcal{X} is still invalid: \boldsymbol must
            # receive the complete math alphabet expression as its argument.
            formula = re.sub(
                rf"\\{prefix}\s*\\{alphabet}\{{\s*([A-Za-z])\s*\}}",
                rf"\\{prefix}{{\\{alphabet}{{\1}}}}",
                formula,
            )
    return formula


def heading_command(level: int, title: str) -> str:
    title = title.strip()
    chapter_match = re.match(r"第\s*(\d+)\s*章\s*(.*)$", title)
    if level == 2 and chapter_match:
        return rf"\chapter{{{escape_text(chapter_match.group(2).strip())}}}"

    numbered = re.match(r"\d+(?:\.\d+)+\s*(.*)$", title)
    if numbered:
        title = numbered.group(1).strip()
    if level == 2 and title == "历史注记":
        return r"\section*{历史注记}" + "\n" + r"\addcontentsline{toc}{section}{历史注记}"
    command = {2: "section", 3: "subsection", 4: "subsubsection"}.get(level)
    if command is None:
        raise ValueError(f"Unsupported heading level: {level}")
    return rf"\{command}{{{escape_text(title)}}}"


def table_cells(row: str) -> list[str]:
    """Parse a simple GitHub-flavored Markdown table row."""
    cells = row.strip().strip("|").split("|")
    return [cell.strip() for cell in cells]


def latex_table(rows: list[list[str]]) -> list[str]:
    """Render a compact, readable LaTeX table from Markdown rows."""
    column_count = max(len(row) for row in rows)
    widths = "".join("p{0.29\\linewidth}" for _ in range(column_count))
    output = [r"\begin{center}", r"\small", rf"\begin{{tabular}}{{{widths}}}", r"\toprule"]
    for index, row in enumerate(rows):
        padded = row + [""] * (column_count - len(row))
        cells = " & ".join(escape_text(cell) for cell in padded)
        output.append(cells + r" \\")
        if index == 0:
            output.append(r"\midrule")
    output.extend([r"\bottomrule", r"\end{tabular}", r"\end{center}", ""])
    return output


def convert_markdown(source: Path) -> str:
    lines = source.read_text(encoding="utf-8-sig").splitlines()
    output: list[str] = []
    index = 0

    while index < len(lines):
        raw = lines[index]
        stripped = raw.strip()

        if stripped == "":
            if output and output[-1] != "":
                output.append("")
            index += 1
            continue

        if stripped.startswith("# "):
            index += 1
            continue

        heading = re.match(r"^(#{2,4})\s+(.+)$", stripped)
        if heading:
            output.append(heading_command(len(heading.group(1)), heading.group(2)))
            output.append("")
            index += 1
            continue

        if stripped.startswith("<div"):
            # Web-only algorithm containers must not be emitted as raw HTML.
            # Preserve their line structure in a compact LaTeX quotation instead.
            output.extend([r"\begin{quote}", r"\small"])
            index += 1
            while index < len(lines) and lines[index].strip() != "</div>":
                algorithm_line = lines[index].strip()
                if algorithm_line:
                    output.append(escape_text(algorithm_line) + r"\\")
                index += 1
            if index >= len(lines):
                raise ValueError(f"Unclosed HTML container in {source} near line {index + 1}")
            output.extend([r"\end{quote}", ""])
            index += 1
            continue

        if TABLE_ROW_RE.match(stripped):
            table_rows: list[list[str]] = []
            while index < len(lines) and TABLE_ROW_RE.match(lines[index].strip()):
                row = lines[index].strip()
                if not TABLE_SEPARATOR_RE.match(row):
                    table_rows.append(table_cells(row))
                index += 1
            if table_rows:
                output.extend(latex_table(table_rows))
            continue

        if stripped == "$$":
            formula_lines: list[str] = []
            index += 1
            while index < len(lines) and lines[index].strip() != "$$":
                formula_lines.append(lines[index])
                index += 1
            if index >= len(lines):
                raise ValueError(f"Unclosed display formula in {source} near line {index + 1}")
            formula = normalize_formula("\n".join(formula_lines))
            environment = "equation" if TAG_RE.search(formula) else "equation*"
            output.extend([rf"\begin{{{environment}}}", formula, rf"\end{{{environment}}}", ""])
            index += 1
            continue

        image = IMAGE_RE.match(stripped)
        if image:
            image_path = image.group(1).replace("\\", "/").split("/")[-1]
            caption = ""
            caption_index = index + 1
            while caption_index < len(lines) and not lines[caption_index].strip():
                caption_index += 1
            if caption_index < len(lines) and lines[caption_index].strip().startswith("图"):
                caption = lines[caption_index].strip()
                index = caption_index
            stem = re.sub(r"[^A-Za-z0-9_-]", "-", Path(image_path).stem)
            figure = [
                r"\begin{figure}[htbp]",
                r"\centering",
                rf"\includegraphics[width=\linewidth]{{{image_path}}}",
            ]
            if caption:
                figure.append(rf"\caption{{{escape_text(caption)}}}")
            figure.extend([rf"\label{{fig:{stem}}}", r"\end{figure}", ""])
            output.extend(figure)
            index += 1
            continue

        output.append(escape_text(stripped))
        index += 1

    while output and output[-1] == "":
        output.pop()
    return "\n".join(output) + "\n"


def validate_markdown(paths: list[Path]) -> tuple[list[str], int, int]:
    errors: list[str] = []
    tags: list[str] = []
    images: list[str] = []
    for path in paths:
        text = path.read_text(encoding="utf-8-sig")
        if text.count("$$") % 2:
            errors.append(f"{path}: unmatched $$ display delimiters")
        tags.extend(match.group(1) for match in TAG_RE.finditer(text))
        images.extend(match.group(1) for match in re.finditer(r"!\[\]\(([^)]+)\)", text))
        if not re.search(r"^##\s+第\s*\d+\s*章", text, re.MULTILINE):
            errors.append(f"{path}: chapter heading not found")

    if len(tags) != len(set(tags)):
        errors.append("duplicate equation tags found")
    return errors, len(set(tags)), len(set(images))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", nargs="+", type=Path, required=True)
    parser.add_argument("--output", nargs="+", type=Path, required=True)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()

    if len(args.input) != len(args.output):
        parser.error("--input and --output must contain the same number of chapter files")

    errors, equation_count, figure_count = validate_markdown(args.input)
    if errors:
        for error in errors:
            print(f"ERROR: {error}")
        return 1
    if args.check:
        print(
            f"Validated {len(args.input)} chapters, {equation_count} equation tags, "
            f"and {figure_count} figures."
        )
        return 0

    for source, destination in zip(args.input, args.output):
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(convert_markdown(source), encoding="utf-8")
        print(f"Wrote {destination}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
