"""Small dependency-free terminal progress reporting for evaluation CLIs."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from time import perf_counter
from typing import TextIO
import sys


@dataclass
class TerminalProgress:
    """Render throttled progress updates interactively or in captured logs."""

    prefix: str
    width: int = 28
    target_updates: int = 100
    stream: TextIO = field(default_factory=lambda: sys.stdout)
    _started_at: float = field(default_factory=perf_counter, init=False)
    _last_completed: int = field(default=-1, init=False)
    _last_line_length: int = field(default=0, init=False)

    def __post_init__(self) -> None:
        if not isinstance(self.prefix, str) or not self.prefix:
            raise ValueError("prefix must be a nonempty string")
        if isinstance(self.width, bool) or self.width < 10:
            raise ValueError("width must be an integer of at least 10")
        if isinstance(self.target_updates, bool) or self.target_updates < 1:
            raise ValueError("target_updates must be a positive integer")

    def update(self, completed: int, total: int, label: str) -> None:
        """Render one update when it crosses the configured display interval."""
        if isinstance(completed, bool) or not isinstance(completed, int):
            raise TypeError("completed must be an integer")
        if isinstance(total, bool) or not isinstance(total, int):
            raise TypeError("total must be an integer")
        if total < 1 or completed < 0 or completed > total:
            raise ValueError("progress must satisfy 0 <= completed <= total")
        if not isinstance(label, str) or not label:
            raise ValueError("label must be a nonempty string")
        interactive = bool(getattr(self.stream, "isatty", lambda: False)())
        display_updates = self.target_updates if interactive else min(
            self.target_updates, 20
        )
        interval = max(1, math.ceil(total / display_updates))
        if (
            completed not in {0, total}
            and self._last_completed >= 0
            and completed - self._last_completed < interval
        ):
            return

        elapsed = max(0.0, perf_counter() - self._started_at)
        fraction = completed / total
        filled = min(self.width, int(round(self.width * fraction)))
        bar = "#" * filled + "-" * (self.width - filled)
        if completed == 0 or elapsed <= 0.0:
            eta = "--"
        else:
            remaining = elapsed * (total - completed) / completed
            eta = _duration(remaining)
        line = (
            f"{self.prefix} [{bar}] {100.0 * fraction:6.2f}% "
            f"({completed}/{total}) elapsed={_duration(elapsed)} eta={eta} | {label}"
        )
        if interactive:
            padding = " " * max(0, self._last_line_length - len(line))
            ending = "\n" if completed == total else "\r"
            print(f"{line}{padding}", end=ending, file=self.stream, flush=True)
            self._last_line_length = 0 if completed == total else len(line)
        else:
            print(line, file=self.stream, flush=True)
        self._last_completed = completed

    def close(self) -> None:
        """Terminate an unfinished interactive line after an exception."""
        interactive = bool(getattr(self.stream, "isatty", lambda: False)())
        if interactive and self._last_line_length:
            print(file=self.stream, flush=True)
            self._last_line_length = 0


def _duration(seconds: float) -> str:
    total_seconds = max(0, int(round(seconds)))
    hours, remainder = divmod(total_seconds, 3600)
    minutes, seconds_part = divmod(remainder, 60)
    if hours:
        return f"{hours:d}:{minutes:02d}:{seconds_part:02d}"
    return f"{minutes:02d}:{seconds_part:02d}"
