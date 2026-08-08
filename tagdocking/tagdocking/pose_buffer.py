"""Pose buffer with latency filtering.

Maintains a ring buffer of timestamped TagPose entries.
Drops poses whose sensor timestamp is older than max_latency_ns
from the current time. Controller always consumes the latest valid entry.
"""

import math
from .utils import TagPose


class PoseBuffer:
    """Timestamped ring buffer for tag poses with latency filtering.

    Usage:
        buf = PoseBuffer(max_size=30, max_latency_ns=200_000_000)  # 200ms
        buf.add(TagPose(dist=0.5, lat=0.02, yaw=-0.1, stamp_ns=...))
        pose = buf.get_latest(now_ns)  # returns None if buffer empty or all stale
    """

    def __init__(self, max_size: int = 30, max_latency_ns: int = 200_000_000):
        self._max_size = max_size
        self._max_latency_ns = max_latency_ns
        self._buffer = []

    def add(self, pose: TagPose):
        """Append a new pose. Evicts oldest if buffer is full."""
        self._buffer.append(pose)
        while len(self._buffer) > self._max_size:
            self._buffer.pop(0)

    def get_latest(self, now_ns: int) -> TagPose | None:
        """Return the most recent pose whose timestamp is within latency limit.

        Scans from newest to oldest. Returns None if buffer is empty
        or all entries exceed max_latency_ns.
        """
        if not self._buffer:
            return None
        # Walk backward (newest first) to find first entry within latency window
        for pose in reversed(self._buffer):
            latency_ns = now_ns - pose.stamp_ns
            if latency_ns < 0:
                # pose is from the future (clock skew) — accept it
                return pose
            if latency_ns <= self._max_latency_ns:
                return pose
        return None  # all entries too old

    def clear(self):
        """Remove all buffered poses."""
        self._buffer.clear()

    def set_max_latency_ns(self, max_latency_ns: int):
        """Update the staleness window at runtime (adaptive to frame rate)."""
        self._max_latency_ns = int(max_latency_ns)

    @property
    def size(self) -> int:
        return len(self._buffer)

    @property
    def empty(self) -> bool:
        return len(self._buffer) == 0

    @property
    def max_latency_ns(self) -> int:
        return self._max_latency_ns
