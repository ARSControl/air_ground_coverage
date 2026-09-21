"""Font settings for publication-safe Matplotlib PDF and PS output."""

from __future__ import annotations

import matplotlib


PDF_FONT_TYPE = 42
PS_FONT_TYPE = 42


def configure_publication_fonts() -> None:
    """Embed TrueType outlines instead of submission-rejected Type 3 glyphs."""
    matplotlib.rcParams["pdf.fonttype"] = PDF_FONT_TYPE
    matplotlib.rcParams["ps.fonttype"] = PS_FONT_TYPE
