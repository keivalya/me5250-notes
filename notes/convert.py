import argparse
import re
import sys
from typing import Iterable

MATRIX_PATTERN = re.compile(r"\|([^|\n]+?)\|")


def _split_columns(content: str) -> str:
    """
    Split matrix columns using whitespace and return a LaTeX row.
    """
    cols = [c.strip() for c in re.split(r"\s+", content.strip()) if c.strip()]
    if not cols:
        return content.strip()
    return " & ".join(cols)


def convert_matrix_line(line: str) -> str:
    """Convert each pipe-delimited matrix snippet in a line into LaTeX bmatrix."""

    def repl(match: re.Match[str]) -> str:
        content = match.group(1)
        row = _split_columns(content)
        return f"\\begin{{bmatrix}} {row} \\end{{bmatrix}}"

    return MATRIX_PATTERN.sub(repl, line)


def convert_text(lines: Iterable[str]) -> str:
    return "".join(convert_matrix_line(line) for line in lines)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Convert pipe-delimited matrix snippets in markdown text to LaTeX bmatrix."
    )
    parser.add_argument(
        "input",
        nargs="?",
        help="Input file path. If omitted, read from stdin.",
    )
    parser.add_argument(
        "-o",
        "--output",
        help="Output file path. If omitted, write to stdout.",
    )
    args = parser.parse_args()

    if args.input:
        with open(args.input, "r", encoding="utf-8") as f:
            converted = convert_text(f)
    else:
        converted = convert_text(sys.stdin)

    if args.output:
        with open(args.output, "w", encoding="utf-8") as f:
            f.write(converted)
    else:
        sys.stdout.write(converted)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
