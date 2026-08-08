"""Omni / Mecanum wheel adapter — full 3-DOF discrete motion commands.

Supports forward jog, in-place turn, AND lateral jog (for direct
normal-line correction without multi-phase oblique approach).
"""

from geometry_msgs.msg import Twist
from .base_adapter import BaseAdapter


class OmniAdapter(BaseAdapter):
    """Publish jog/turn/lateral/stop to cmd_vel for omni/mecanum platforms.

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
        """Turn while creeping forward (arc). Omni can rotate in place freely,
        but this keeps a uniform adapter interface with diff-drive."""
        msg = Twist()
        msg.linear.x = float(linear_rate)
        msg.angular.z = float(angular_rate)
        self._publisher.publish(msg)

    def publish_stop(self):
        self._publisher.publish(Twist())

    # ── Omni-specific ──────────────────────────────────────────────

    def publish_lateral(self, lateral_rate: float):
        """Publish a pure-lateral velocity (positive = left, REP-103)."""
        msg = Twist()
        msg.linear.y = float(lateral_rate)
        self._publisher.publish(msg)
