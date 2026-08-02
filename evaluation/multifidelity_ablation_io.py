"""Versioned, pickle-free archives for multi-fidelity ablation studies."""

from __future__ import annotations

import json
import os
from pathlib import Path
import tempfile
from typing import Any

import numpy as np


RAW_ARCHIVE_KIND = "multifidelity_ablation_raw"
EVALUATED_ARCHIVE_KIND = "multifidelity_ablation_evaluated"
ARCHIVE_SCHEMA_VERSION = 1


def save_archive(
    path: str | Path,
    *,
    kind: str,
    metadata: dict[str, Any],
    arrays: dict[str, np.ndarray],
) -> Path:
    """Atomically save one compressed archive without Python object arrays."""
    destination = Path(path)
    if destination.suffix != ".npz":
        raise ValueError("archive output path must end in .npz")
    destination.parent.mkdir(parents=True, exist_ok=True)
    document = dict(metadata)
    document["archive_kind"] = kind
    document["schema_version"] = ARCHIVE_SCHEMA_VERSION
    encoded = json.dumps(document, sort_keys=True, separators=(",", ":"))
    payload = {"metadata_json": np.asarray(encoded), **arrays}
    for name, values in payload.items():
        array = np.asarray(values)
        if array.dtype == object:
            raise TypeError(f"archive array {name!r} must not use object dtype")

    temporary_name: str | None = None
    try:
        with tempfile.NamedTemporaryFile(
            dir=destination.parent,
            prefix=f".{destination.stem}.",
            suffix=".npz",
            delete=False,
        ) as temporary:
            temporary_name = temporary.name
        np.savez_compressed(temporary_name, **payload)
        os.replace(temporary_name, destination)
    finally:
        if temporary_name is not None and os.path.exists(temporary_name):
            os.unlink(temporary_name)
    return destination


def load_archive(
    path: str | Path, *, expected_kind: str
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    """Load and validate archive identity while keeping pickle disabled."""
    source = Path(path)
    with np.load(source, allow_pickle=False) as archive:
        if "metadata_json" not in archive.files:
            raise ValueError("archive is missing metadata_json")
        raw_metadata = archive["metadata_json"]
        if raw_metadata.shape != ():
            raise ValueError("metadata_json must be a scalar string")
        metadata = json.loads(str(raw_metadata.item()))
        arrays = {
            name: np.array(archive[name], copy=True)
            for name in archive.files
            if name != "metadata_json"
        }
    if metadata.get("archive_kind") != expected_kind:
        raise ValueError(
            f"expected archive kind {expected_kind!r}; "
            f"got {metadata.get('archive_kind')!r}"
        )
    if metadata.get("schema_version") != ARCHIVE_SCHEMA_VERSION:
        raise ValueError("unsupported archive schema version")
    return metadata, arrays
