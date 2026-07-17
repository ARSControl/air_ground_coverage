"""Render the deterministic observer-only video recording data-flow SVG."""

from pathlib import Path


OUTPUT_PATH = Path("docs/assets/multifidelity_video_recording_milestone.svg")


def main() -> None:
    svg = '''<svg xmlns="http://www.w3.org/2000/svg" width="1180" height="300" viewBox="0 0 1180 300">
<defs><marker id="arrow" markerWidth="10" markerHeight="10" refX="9" refY="3" orient="auto"><path d="M0,0 L0,6 L9,3 z" fill="#455a64"/></marker></defs>
<style>text { font-family: Arial, sans-serif; font-size: 16px; fill: #263238; } .small { font-size: 14px; }</style>
<rect x="30" y="105" width="220" height="74" rx="9" fill="#d8ecff" stroke="#263238" stroke-width="1.5"/><text x="140" y="137" text-anchor="middle">Completed coupled step</text><text x="140" y="158" text-anchor="middle">posterior + positions</text>
<rect x="330" y="105" width="220" height="74" rx="9" fill="#e6f4d7" stroke="#263238" stroke-width="1.5"/><text x="440" y="137" text-anchor="middle">Frame interval gate</text><text x="440" y="158" text-anchor="middle">final step always included</text>
<rect x="630" y="105" width="220" height="74" rx="9" fill="#fff1cc" stroke="#263238" stroke-width="1.5"/><text x="740" y="137" text-anchor="middle">Video renderer</text><text x="740" y="158" text-anchor="middle">density, uncertainty, robots</text>
<rect x="930" y="105" width="220" height="74" rx="9" fill="#eadcff" stroke="#263238" stroke-width="1.5"/><text x="1040" y="137" text-anchor="middle">GIF / MP4 output</text><text x="1040" y="158" text-anchor="middle">observer-only artifact</text>
<path d="M255 142 H325 M555 142 H625 M855 142 H925" stroke="#455a64" stroke-width="2" marker-end="url(#arrow)"/>
<path d="M740 189 V230 H140 V184" fill="none" stroke="#b91c1c" stroke-width="1.8" stroke-dasharray="6 5" marker-end="url(#arrow)"/>
<text class="small" x="440" y="258" text-anchor="middle" fill="#b91c1c">No feedback path: rendering does not change sensing, estimation, control, or dynamics.</text>
</svg>'''
    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_PATH.write_text(svg)


if __name__ == "__main__":
    main()
