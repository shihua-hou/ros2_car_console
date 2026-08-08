"""Nav2 pre-docking navigation manager.

Wraps the Nav2 NavigateToPose action client. On success the robot is positioned
at a pre-dock point (default 1m in front of the AprilTag), facing it.

When navigation.enable=False (Mode B: Direct Docking), this module is bypassed
and the state machine goes directly to SEARCH_TAG.
"""

import math
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose


class NavigationManager:
    """Nav2 NavigateToPose action client wrapper.

    Usage:
        nav = NavigationManager(node)
        nav.send_goal(pre_dock_pose)
        # Poll in control loop:
        if nav.is_done():
            if nav.success():
                ... # proceed to SEARCH_TAG
            else:
                ... # NAV_FAILED
        if nav.timed_out(now_ns):
            ... # NAV_TIMEOUT
    """

    def __init__(self, node: Node,
                 action_name: str = 'navigate_to_pose',
                 timeout_sec: float = 60.0):
        self._node = node
        self._timeout_ns = int(timeout_sec * 1e9)

        self._client = ActionClient(node, NavigateToPose, action_name)
        self._goal_handle = None
        self._result = None  # None=pending, True=success, False=failed
        self._start_ns = 0
        self._cancelled = False

    def send_goal(self, pose: PoseStamped):
        """Send a navigation goal.

        Blocks until the action server is available (with 3s timeout).
        Call this from the state machine (not from a timer callback).
        """
        self._result = None
        self._cancelled = False
        self._goal_handle = None

        if not self._client.wait_for_server(timeout_sec=3.0):
            self._node.get_logger().error(
                'Nav2 action server 不可用 — Nav2 启动了吗？')
            self._result = False
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose
        self._start_ns = self._node.get_clock().now().nanoseconds

        self._node.get_logger().info(
            f'Nav2 目标：x={pose.pose.position.x:.2f}, '
            f'y={pose.pose.position.y:.2f}（坐标系 {pose.header.frame_id}）')

        send_future = self._client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._node.get_logger().error('Nav2 目标被拒绝')
            self._result = False
            return
        self._goal_handle = goal_handle
        self._node.get_logger().info('Nav2 目标已接受')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        from action_msgs.msg import GoalStatus
        result = future.result()
        if result is None:
            self._result = False
            return
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self._node.get_logger().info('Nav2 目标已到达')
            self._result = True
        else:
            self._node.get_logger().error(
                f'Nav2 目标失败（status={result.status}）')
            self._result = False

    def is_done(self) -> bool:
        """Return True if the navigation has completed (success or failure)."""
        return self._result is not None

    def success(self) -> bool:
        """Return True if navigation succeeded."""
        return self._result is True

    def failed(self) -> bool:
        """Return True if navigation failed."""
        return self._result is False

    def timed_out(self, now_ns: int) -> bool:
        """Return True if navigation has exceeded the timeout."""
        if self._start_ns == 0:
            return False
        return (now_ns - self._start_ns) > self._timeout_ns

    def cancel(self):
        """Cancel the current navigation goal."""
        self._cancelled = True
        if self._goal_handle is not None:
            self._node.get_logger().info('正在取消 Nav2 目标')
            self._goal_handle.cancel_async()
            self._goal_handle = None
        self._result = None

    @staticmethod
    def compute_pre_dock_pose(tag_map_x: float, tag_map_y: float,
                              tag_map_yaw: float,  # yaw of tag facing in map
                              pre_dock_distance: float = 1.0,
                              map_frame: str = 'map') -> PoseStamped:
        """Compute the pre-dock pose in map frame.

        The pre-dock pose is `pre_dock_distance` meters in front of the
        AprilTag, facing the tag (so the robot's camera sees the tag).

        Args:
            tag_map_x, tag_map_y: Tag position in map frame.
            tag_map_yaw: Yaw of the tag's facing direction in map frame (rad).
                         The tag points along this direction (its normal).
            pre_dock_distance: How far in front of the tag to stop (m).
            map_frame: Frame ID for the output pose (default 'map').

        Returns:
            PoseStamped in map frame.
        """
        pose = PoseStamped()
        pose.header.frame_id = map_frame
        # Robot stands at pre_dock_distance in front of the tag,
        # facing the tag (opposite direction of tag normal).
        pose.pose.position.x = tag_map_x - pre_dock_distance * math.cos(tag_map_yaw)
        pose.pose.position.y = tag_map_y - pre_dock_distance * math.sin(tag_map_yaw)
        pose.pose.position.z = 0.0

        # Robot yaw faces the tag: opposite of tag normal
        robot_yaw = tag_map_yaw + math.pi
        pose.pose.orientation = Quaternion()
        pose.pose.orientation.z = math.sin(robot_yaw / 2.0)
        pose.pose.orientation.w = math.cos(robot_yaw / 2.0)

        return pose
