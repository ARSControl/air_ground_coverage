"""Sequential cached publication around expensive central model updates."""

from time import monotonic


class CentralUpdateLoop:
    """Publish valid cached state before compute and completion-stamped state after.

    No threads, tasks or background processes are used. A failed attempt stops
    subsequent cached refreshes until a successful result is produced again.
    """

    def __init__(
        self, runtime, *, clock, publish_target, publish_status, wall_clock=monotonic
    ):
        self.runtime = runtime
        self.clock = clock
        self.wall_clock = wall_clock
        self.publish_target = publish_target
        self.publish_status = publish_status
        self.cached = None
        self.failed = False
        self.in_progress = False
        self.last_started_at = None
        self.last_completed_at = None
        self.last_duration_s = 0.0

    @property
    def diagnostics(self):
        return dict(
            in_progress=self.in_progress,
            last_started_at=self.last_started_at,
            last_completed_at=self.last_completed_at,
            last_duration_s=self.last_duration_s,
            last_attempt_failed=self.failed,
        )

    def _publish(self, packet, timestamp, *, in_progress, model_updated):
        value = dict(
            packet,
            timestamp=timestamp,
            model_updated=model_updated,
            update_in_progress=in_progress,
            update_started_at=self.last_started_at,
            update_completed_at=None if in_progress else self.last_completed_at,
            update_duration_s=None if in_progress else self.last_duration_s,
        )
        self.publish_target(value)

    def tick(self):
        started = self.clock()
        if not self.runtime.estimator.update_due(started):
            return
        self.last_started_at = started
        self.in_progress = True
        self.publish_status()
        if self.cached is not None and not self.failed:
            self._publish(self.cached, started, in_progress=True, model_updated=False)
        wall_started = self.wall_clock()
        try:
            packet = self.runtime.tick(started)
            if packet is not None:
                self.failed = False
        except Exception:
            self.failed = True
            raise
        finally:
            completed = self.clock()
            self.runtime.estimator.complete_update(completed)
            self.last_completed_at = completed
            self.last_duration_s = max(0.0, self.wall_clock() - wall_started)
            self.in_progress = False
            self.publish_status()
        if packet is not None:
            self.cached = packet
            self.failed = False
            self._publish(
                packet,
                completed,
                in_progress=False,
                model_updated=packet["model_updated"],
            )
        elif self.cached is not None and not self.failed:
            self._publish(
                self.cached, completed, in_progress=False, model_updated=False
            )
