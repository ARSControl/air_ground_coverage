"""Archive identity for closed-loop team-composition reconstruction studies."""

from evaluation.multifidelity_ablation_io import load_archive, save_archive


RAW_ARCHIVE_KIND = "multifidelity_composition_raw"
EVALUATED_ARCHIVE_KIND = "multifidelity_composition_evaluated"


__all__ = (
    "EVALUATED_ARCHIVE_KIND",
    "RAW_ARCHIVE_KIND",
    "load_archive",
    "save_archive",
)
