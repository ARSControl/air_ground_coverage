"""Render the deterministic LaTeX-to-Pages publication-flow SVG."""

from pathlib import Path


OUTPUT_PATH = Path("docs/assets/latex_publication_pipeline.svg")


def node(x: int, y: int, lines: tuple[str, str], color: str) -> str:
    text = "".join(
        f'<text x="{x + 130}" y="{y + 34 + 21 * index}" '
        f'text-anchor="middle">{line}</text>'
        for index, line in enumerate(lines)
    )
    return (
        f'<rect x="{x}" y="{y}" width="260" height="76" rx="9" '
        f'fill="{color}" stroke="#263238" stroke-width="1.5"/>{text}'
    )


def main() -> None:
    nodes = "".join(
        (
            node(30, 125, ("LaTeX source", "mathematical_formulation.tex"), "#d8ecff"),
            node(335, 125, ("GitHub Actions", "latex-action build"), "#e6f4d7"),
            node(640, 35, ("Pull request", "PDF artifact (30 days)"), "#fff1cc"),
            node(640, 215, ("Push to main", "Pages artifact"), "#fff1cc"),
            node(945, 215, ("GitHub Pages", "public PDF"), "#eadcff"),
        )
    )
    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="1235" height="355" viewBox="0 0 1235 355">
<defs><marker id="arrow" markerWidth="10" markerHeight="10" refX="9" refY="3" orient="auto"><path d="M0,0 L0,6 L9,3 z" fill="#455a64"/></marker></defs>
<style>text {{ font-family: Arial, sans-serif; font-size: 16px; fill: #263238; }} .note {{ font-size: 15px; }}</style>
{nodes}
<path d="M290 163 H330" stroke="#455a64" stroke-width="2" marker-end="url(#arrow)"/>
<path d="M595 150 L635 82" stroke="#455a64" stroke-width="2" marker-end="url(#arrow)"/>
<path d="M595 177 L635 251" stroke="#455a64" stroke-width="2" marker-end="url(#arrow)"/>
<path d="M900 253 H940" stroke="#455a64" stroke-width="2" marker-end="url(#arrow)"/>
<text class="note" x="617" y="335" text-anchor="middle">Generated PDFs are deployed as artifacts; the workflow never writes them into Git history.</text>
</svg>
'''
    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_PATH.write_text(svg)


if __name__ == "__main__":
    main()
