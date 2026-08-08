"""Quadruped robot adapter — SDK callback for discrete motion commands.

Bridges the BaseAdapter interface to a quadruped SDK's move(vx, vy, wz)
callback.  No ROS publisher is created — all commands go through the SDK.
"""

from .base_adapter import BaseAdapter


class QuadrupedAdapter(BaseAdapter):
    """Bridge from stop-and-go commands to a quadruped SDK.

    Args:
        move_callback: Callable(vx, vy, yaw_rate) called on every command.
        node: Optional ROS2 Node (for logging diagnostics only).
    """

    def __init__(self, move_callback=None, *, node=None):
        self._move_cb = move_callback or (lambda vx, vy, wz: None)
        self._node = node

    # ── BaseAdapter interface ──────────────────────────────────────

    def publish_jog(self, linear_rate: float):
        self._move_cb(float(linear_rate), 0.0, 0.0)
        self._log(linear_rate, 0.0, 0.0)

    def publish_turn(self, angular_rate: float):
        self._move_cb(0.0, 0.0, float(angular_rate))
        self._log(0.0, 0.0, angular_rate)

    def publish_arc(self, linear_rate: float, angular_rate: float):
        """Turn while creeping forward (arc). Legged robots rotate freely, but
        this keeps a uniform adapter interface with diff-drive."""
        self._move_cb(float(linear_rate), 0.0, float(angular_rate))
        self._log(linear_rate, 0.0, angular_rate)

    def publish_stop(self):
        self._move_cb(0.0, 0.0, 0.0)

    # ── Omni-equivalent ────────────────────────────────────────────

    def publish_lateral(self, lateral_rate: float):
        """Publish a pure-lateral velocity (positive = left, REP-103)."""
        self._move_cb(0.0, float(lateral_rate), 0.0)
        self._log(0.0, lateral_rate, 0.0)

    # ── Helpers ────────────────────────────────────────────────────

    def set_move_callback(self, move_callback):
        self._move_cb = move_callback

    def _log(self, vx, vy, wz):
        if self._node is not None:
            self._node.get_logger().debug(
                f'Quadruped cmd: vx={vx:.3f} vy={vy:.3f} wz={wz:.3f}',
                throttle_duration_sec=1.0)
