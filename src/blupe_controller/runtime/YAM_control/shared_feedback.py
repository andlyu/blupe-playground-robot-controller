"""Freshness-checked motor snapshots shared by all operator readers."""
import threading
import time


class FeedbackUnavailable(RuntimeError):
    """Telemetry is temporarily absent; this is not a motor fault."""


class SharedFeedback:
    def __init__(self):
        self.condition = threading.Condition()
        self.latest = None

    def publish(self, packet):
        with self.condition:
            if self.latest is None or packet['monotonic'] > self.latest['monotonic']:
                self.latest = packet
                self.condition.notify_all()

    def read(self, *, timeout=.5, max_age=.1):
        # Require a sample started after this request: a final-packet reader must
        # never receive a pre-completion pose merely because it is fairly recent.
        started = time.monotonic()
        with self.condition:
            while True:
                now = time.monotonic()
                p = self.latest
                if p is not None and started <= p['monotonic'] <= now and now-p['monotonic'] <= max_age:
                    return p
                remaining = timeout-(now-started)
                if remaining <= 0:
                    raise FeedbackUnavailable('Current joint feedback temporarily unavailable or stale')
                self.condition.wait(remaining)
