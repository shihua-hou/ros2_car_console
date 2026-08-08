"""Abstract base class for robot chassis adapters — stop-and-go paradigm.

In the stop-and-go paradigm the adapters no longer compute velocities from
errors.  They simply publish the constant-rate commands issued by the
ActionExecutor: jog (straight-line forward), turn (rotate in place),
lateral movement (omni only), and stop.
"""

from abc import ABC, abstractmethod


class BaseAdapter(ABC):
    """Minimal interface for publishing discrete motion commands.

    Subclasses:
        DiffDriveAdapter  — Twist(linear.x) / Twist(angular.z)
        OmniAdapter       — Twist(linear.x, linear.y, angular.z)
        QuadrupedAdapter  — SDK move() callback
    """

    @abstractmethod
    def publish_jog(self, linear_rate: float):
        """Publish a straight-line forward/reverse jog at the given rate (m/s)."""

    @abstractmethod
    def publish_turn(self, angular_rate: float):
        """Publish an in-place rotation at the given rate (rad/s)."""

    def publish_arc(self, linear_rate: float, angular_rate: float):
        """Publish a simultaneous forward creep + rotation (an arc).

        Default falls back to a pure in-place turn for adapters that do not
        override it. Diff-drive overrides this to beat static friction: rotating
        with the wheels already rolling avoids the stall/lurch of a stationary
        pivot.
        """
        self.publish_turn(angular_rate)

    @abstractmethod
    def publish_stop(self):
        """Publish zero velocity on all axes."""

    def emergency_stop(self):
        """Override for platform-specific brake/estop behaviour."""
        self.publish_stop()
