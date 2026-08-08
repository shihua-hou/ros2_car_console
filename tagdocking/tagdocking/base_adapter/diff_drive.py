"""Differential-drive adapter — forward jog and in-place turn via Twist.

Diff-drive has no lateral DOF.  Lateral error is handled by the
geometry planner's multi-phase oblique approach (turn → jog → turn).
"""

from geometry_msgs.msg import Twist
from .base_adapter import BaseAdapter


class DiffDriveAdapter(BaseAdapter):
    """Publish jog/turn/stop to cmd_vel for a differential-drive robot.

    Args:
        node: ROS2 Node (for creating the publisher).
        cmd_vel_topic: Topic name, default 'cmd_vel'.
    """

    def __init__(self, node, *, cmd_vel_topic: str = 'cmd_vel'):
        self._publisher = node.create_publisher(Twist, cmd_vel_topic, 10)

    # ── BaseAdapter interface ──────────────────────────────────────

    def publish_jog(self, linear_rate: float):
        msg = Twist()
        msg.linear.x = float(linear_rate)
        self._publisher.publish(msg)

    def publish_turn(self, angular_rate: float):
        msg = Twist()
        msg.angular.z = float(angular_rate)
        self._publisher.publish(msg)

    def publish_arc(self, linear_rate: float, angular_rate: float):
        """Turn while creeping forward — an arc.

        In-place rotation on a diff-drive fights large static friction (the
        wheels must break free with zero rolling momentum), so a small turn can
        stall or lurch. Adding a little linear.x keeps the wheels rolling, so the
        turn is smooth and actually happens. The modest forward translation is
        corrected by the next re-measure (iterative refine).
        """
        msg = Twist()
        msg.linear.x = float(linear_rate)
        msg.angular.z = float(angular_rate)
        self._publisher.publish(msg)

    def publish_stop(self):
        self._publisher.publish(Twist())
