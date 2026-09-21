"""Archive identity for the standalone Zhang baseline."""

from evaluation.multifidelity_ablation_io import load_archive, save_archive


ZHANG_BASELINE_RAW_ARCHIVE_KIND = "zhang_baseline_raw"


__all__ = ("ZHANG_BASELINE_RAW_ARCHIVE_KIND", "load_archive", "save_archive")
