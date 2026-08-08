"""Main docking node — integrates all subsystems (stop-and-go paradigm).

Architecture:
  Subscriptions:
    /detections      → AprilTag detection array
    /tf              → (via tf2_ros.Buffer) tag pose lookup
    /odom            → odometry feedback
    /amcl_pose       → robot pose in map (for Nav2 pre-dock computation)

  Publishers:
    /cmd_vel         → (via BaseAdapter) velocity commands
    ~/state          → std_msgs/String — current state name
    ~/error          → geometry_msgs/Vector3 — (error_x, error_y, error_yaw)

  Services:
    ~/start_docking  → std_srvs/Trigger — start docking
    ~/cancel_docking → std_srvs/Trigger — cancel docking

  Action:
    ~/dock           → Dock.action — full docking with feedback

  Control loop: 20 Hz timer

    Stop-and-go paradigm:
      if action active:
          odometry dead-reckoning → check if target reached
      else:
          wait visual settle (after turn)
          read fresh tag pose → geometry planner → start next action

    Every motion is measured by odometry and stops exactly when the
    target distance/angle is reached.  No continuous PID servoing.
"""

import math
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from apriltag_msgs.msg import AprilTagDetectionArray
import tf2_ros

from .utils import TagPose, yaw_from_quat, normalize_angle, tag_normal_angle
from .pose_buffer import PoseBuffer
from .geometry_planner import GeometryPlanner, ActionPlan
from .action_executor import ActionExecutor
from .navigation_manager import NavigationManager
from .state_machine import DockingStateMachine, DockingState


class DockingNode(Node):
    """AprilTag auto-docking controller node — stop-and-go paradigm."""

    def __init__(self):
        super().__init__('docking_node')

        # ── Parameters ────────────────────────────────────────────
        self._declare_params()

        # ── TF2 ───────────────────────────────────────────────────
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Pose buffer ───────────────────────────────────────────
        self._pose_buffer = PoseBuffer(
            max_size=self._p('pose_buffer.size'),
            max_latency_ns=int(self._p('camera.max_latency_ms') * 1_000_000))

        # ── Geometry planner (normal-line alignment) ──────────────
        self._planner = GeometryPlanner(
            target_distance=self._p('dock_target.distance'),
            lateral_threshold=self._p('stopgo.lateral_threshold'),
            yaw_threshold=math.radians(self._p('stopgo.yaw_threshold_deg')),
            tune_angle=self._p('stopgo.tune_angle'),
            jog_min=self._p('stopgo.jog_min'),
            jog_max=self._p('stopgo.jog_max'),
            position_tol=self._p('tolerance.position_m'),
            base_type=self._p('base.type'),
        )

        # ── Action executor (odometry dead-reckoning) ─────────────
        self._executor = ActionExecutor(
            turn_settle_sec=self._p('stopgo.turn_settle_sec'),
            turn_undershoot=self._p('stopgo.turn_undershoot'),
            max_turn_step=self._p('stopgo.max_turn_step'),
            small_turn_rad=self._p('stopgo.small_turn_rad'),
            final_approach_distance=self._p('final_servo.distance'),
            yaw_threshold=math.radians(self._p('stopgo.yaw_threshold_deg')),
        )

        # ── Navigation ────────────────────────────────────────────
        self._nav = NavigationManager(
            self,
            action_name=self._p('navigation.nav2_action_name'),
            timeout_sec=self._p('navigation.nav2_timeout_sec'))

        # ── State machine ─────────────────────────────────────────
        self._sm = DockingStateMachine(self)

        # ── Base adapter ──────────────────────────────────────────
        self._adapter = self._create_adapter()

        # ── Tag tracking state ────────────────────────────────────
        self._tag_frame = ''
        self._dock_tag_id = 0
        self._camera_frame = ''

        # Filtered pose
        self._filtered_dist: float | None = None
        self._filtered_lat: float | None = None
        self._filtered_yaw: float | None = None
        self._filtered_normal: float | None = None
        self._normal_sin = 0.0
        self._normal_cos = 0.0
        self._filter_init = False
        self._ema_alpha = 0.5
        self._max_pose_jump_m = 0.3
        self._jump_reject_count = 0
        self._max_jump_rejections = 10

        # Latest raw values
        self._raw_dist: float | None = None
        self._raw_lat: float | None = None
        self._raw_yaw: float | None = None
        self._raw_normal: float | None = None
        self._last_detection_ns = 0
        self._tf_fail_count = 0

        # Previous control-loop state, for detecting transitions that must
        # cancel any in-progress stop-and-go action (e.g. APPROACH→SEARCH_TAG
        # re-lock: a half-finished turn must not resume against a stale odom
        # reference when we return to APPROACH).
        self._prev_state = None

        # ── Blind turn-drive-turn maneuver ────────────────────────────
        # The planner computes a whole [turn1, drive, turn2] path from one good
        # measurement; the node runs the sub-steps back-to-back by odometry
        # (camera NOT consulted between them — the tag may leave view, that is
        # expected). Only after the full sequence completes do we settle,
        # re-measure, and iterate (iterative refine). This replaces the fragile
        # incremental aim-and-go that lost the tag at the FOV edge.
        self._maneuver_queue: list = []
        self._maneuver_active = False
        self._maneuver_iters = 0
        # 检测冻结标志：机动（盲转/盲走）期间为 True，此时 _on_detections 直接
        # 丢弃所有帧（运动模糊、视野边缘的坏帧绝不能污染规划用的位姿）。停稳
        # settle 结束后解冻，并清空滤波/缓冲，强制下一次规划只用停稳后的新鲜帧。
        self._frozen = False
        # Each iteration advances at most jog_max (~0.15 m) on the straight leg,
        # so covering a metre-plus approach plus refinement turns needs a
        # generous ceiling. This is only a runaway backstop — normal docking
        # converges (drive shrinks, no more clamping) well before it. APPROACH's
        # own timeout bounds wall-clock independently.
        self._max_maneuver_iters = 40

        # Recovery-search state: remember which side the tag was last seen on
        # (sign of lat) so a bidirectional sweep starts toward it, and track the
        # rotate-phase boundary to alternate + widen each sweep.
        self._last_seen_lat = 0.0
        self._search_sweep_idx = 0
        self._search_last_phase_start = 0

        # Adaptive detection-rate tracking. The pose-buffer staleness window is
        # derived from the measured inter-detection interval, so the controller
        # self-tunes to whatever rate the camera actually delivers (6 Hz or
        # 30 Hz). In stop-and-go 6 Hz is plenty; we just must not discard a
        # pose as "stale" faster than a new one can arrive.
        self._det_interval_ns: float | None = None   # EMA of gaps between detections
        self._latency_floor_ns = int(self._p('camera.max_latency_ms') * 1_000_000)
        self._latency_margin = self._p('camera.latency_interval_margin')

        # Odometry (updated in callback)
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self._has_odom = False

        # AMCL pose (for Nav2 pre-dock)
        self._amcl_pose = None

        # Control period
        self._dt = 0.05  # 20 Hz

        # ── ROS interfaces ─────────────────────────────────────────
        self._init_ros_interfaces()

        # ── Timer ──────────────────────────────────────────────────
        self._timer = self.create_timer(self._dt, self._control_loop)

        # ── Shutdown ───────────────────────────────────────────────
        rclpy.get_default_context().on_shutdown(self._safe_stop)
        self._install_signal_handlers()

        self.get_logger().info(
            f'停靠节点就绪 | 底盘={self._p("base.type")} | '
            f'导航={self._p("navigation.enable")} | 走停模式')

    # ── Parameter helpers ──────────────────────────────────────────

    def _declare_params(self):
        """Declare all ROS2 parameters with defaults."""
        # Navigation
        self.declare_parameter('navigation.enable', True)
        self.declare_parameter('navigation.pre_dock_distance', 1.0)
        self.declare_parameter('navigation.position_tolerance', 0.1)
        self.declare_parameter('navigation.yaw_tolerance_deg', 5.0)
        self.declare_parameter('navigation.nav2_action_name', 'navigate_to_pose')
        self.declare_parameter('navigation.nav2_timeout_sec', 60.0)
        self.declare_parameter('navigation.pre_dock_frame', 'map')

        # Camera / timing
        self.declare_parameter('camera.max_latency_ms', 200)
        self.declare_parameter('camera.expected_fps', 30)
        # Adaptive staleness: window = measured detection interval × this margin,
        # clamped to be at least max_latency_ms. Tolerates a few dropped frames.
        self.declare_parameter('camera.latency_interval_margin', 3.0)

        # Tag
        self.declare_parameter('tag.family', '36h11')
        self.declare_parameter('tag.size', 0.16)
        self.declare_parameter('tag.frame', 'tag36h11:0')
        self.declare_parameter('tag.id', 0)
        self.declare_parameter('tag.fresh_timeout_sec', 1.0)
        self.declare_parameter('tag.ema_alpha', 0.5)
        self.declare_parameter('tag.max_pose_jump_m', 0.3)

        # TF
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('measure_frame', '')

        # Dock target
        self.declare_parameter('dock_target.distance', 0.30)
        self.declare_parameter('dock_target.lateral_offset', 0.0)
        self.declare_parameter('dock_target.yaw_offset_deg', 0.0)

        # Base
        self.declare_parameter('base.type', 'diff_drive')
        self.declare_parameter('base.cmd_vel_topic', 'cmd_vel')

        # Tolerance
        self.declare_parameter('tolerance.position_m', 0.03)
        self.declare_parameter('tolerance.yaw_deg', 3.0)
        self.declare_parameter('tolerance.stable_time_sec', 1.0)

        # Safety
        self.declare_parameter('safety.minimum_distance_m', 0.15)
        self.declare_parameter('timeout_sec', 120.0)

        # Search
        self.declare_parameter('search.angular_speed', 0.3)
        self.declare_parameter('search.rotate_time_sec', 0.8)
        self.declare_parameter('search.pause_time_sec', 1.5)
        self.declare_parameter('search.hold_time_sec', 0.5)
        self.declare_parameter('search.search_direction', 1)
        self.declare_parameter('search.timeout_sec', 60.0)

        # Pose buffer
        self.declare_parameter('pose_buffer.size', 30)

        # State timeouts
        self.declare_parameter('align_timeout_sec', 15.0)
        self.declare_parameter('approach_timeout_sec', 60.0)
        self.declare_parameter('final_servo_timeout_sec', 30.0)

        # Final servo
        self.declare_parameter('final_servo.distance', 0.20)
        self.declare_parameter('final_servo.max_linear_speed', 0.05)
        self.declare_parameter('final_servo.max_yaw_speed', 0.2)

        # Stop-and-go params
        self.declare_parameter('stopgo.lateral_threshold', 0.04)
        self.declare_parameter('stopgo.yaw_threshold_deg', 3.0)
        self.declare_parameter('stopgo.tune_angle', 0.0)
        self.declare_parameter('stopgo.jog_min', 0.05)
        self.declare_parameter('stopgo.jog_max', 0.50)
        self.declare_parameter('stopgo.jog_linear_rate', 0.08)
        self.declare_parameter('stopgo.jog_angular_rate', 0.3)
        self.declare_parameter('stopgo.turn_creep_linear', 0.0)  # 已弃用，固定纯原地转
        self.declare_parameter('stopgo.lateral_rate', 0.08)
        self.declare_parameter('stopgo.turn_settle_sec', 0.5)
        self.declare_parameter('stopgo.turn_undershoot', 0.75)
        self.declare_parameter('stopgo.max_turn_step', 0.3)
        self.declare_parameter('stopgo.small_turn_rad', 0.1)
        self.declare_parameter('stopgo.theta_shrink_ratio', 2.0)
        self.declare_parameter('stopgo.drift_tol', 0.15)

        # Detection topic
        self.declare_parameter('detection_topic', '/detections')
        self.declare_parameter('odom_topic', '/odom')

    def _p(self, name: str):
        return self.get_parameter(name).value

    # ── ROS interfaces ─────────────────────────────────────────────

    def _init_ros_interfaces(self):
        """Create all ROS2 subscriptions, publishers, services, actions."""
        det_topic = self._p('detection_topic')
        self._det_sub = self.create_subscription(
            AprilTagDetectionArray, det_topic, self._on_detections, 10)

        odom_topic = self._p('odom_topic')
        self._odom_sub = self.create_subscription(
            Odometry, odom_topic, self._on_odom, 10)

        self._amcl_sub = self.create_subscription(
            Odometry, '/amcl_pose', self._on_amcl_pose, 10)

        self._state_pub = self.create_publisher(String, '~/state', 10)
        self._error_pub = self.create_publisher(Vector3, '~/error', 10)

        self._srv_start = self.create_service(
            Trigger, '~/start_docking', self._on_start_docking)
        self._srv_cancel = self.create_service(
            Trigger, '~/cancel_docking', self._on_cancel_docking)

        try:
            from tagdocking.action import Dock
        except ImportError:
            Dock = None
        if Dock is not None:
            self._action_server = ActionServer(
                self, Dock, '~/dock',
                execute_callback=self._execute_dock_cb,
                cancel_callback=self._dock_cancel_cb)
        else:
            self._action_server = None

    # ── Adapter factory ────────────────────────────────────────────

    def _create_adapter(self):
        """Create the appropriate BaseAdapter based on base.type parameter."""
        base_type = self._p('base.type')
        cmd_vel_topic = self._p('base.cmd_vel_topic')

        if base_type == 'omni':
            from .base_adapter import OmniAdapter
            return OmniAdapter(self, cmd_vel_topic=cmd_vel_topic)
        elif base_type == 'quadruped':
            from .base_adapter import QuadrupedAdapter
            return QuadrupedAdapter(node=self)
        else:  # diff_drive
            from .base_adapter import DiffDriveAdapter
            return DiffDriveAdapter(self, cmd_vel_topic=cmd_vel_topic)

    # ── Callbacks ──────────────────────────────────────────────────

    def _on_detections(self, msg: AprilTagDetectionArray):
        """Store detection timestamps; actual TF query happens in control loop."""
        # 机动期间冻结检测：盲转/盲走过程中相机帧运动模糊、二维码常在视野边缘，
        # 这些坏帧一律丢弃，绝不更新 _filtered_*、_pose_buffer 或 _last_detection_ns。
        # 规划器因此只会读到小车停稳后新采的帧。
        if self._frozen:
            return

        state = self._sm.state
        if state in (DockingState.DOCKED, DockingState.TAG_LOST,
                     DockingState.TIMEOUT, DockingState.MOTION_FAILED,
                     DockingState.CANCELLED):
            return

        dock_id = int(self._p('tag.id'))
        tag_frame = self._p('tag.frame')

        for det in msg.detections:
            if det.id == dock_id:
                self._tag_frame = tag_frame
                self._dock_tag_id = dock_id
                self._lookup_tag_pose()
                return

    def _on_odom(self, msg: Odometry):
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        self._odom_yaw = yaw_from_quat(msg.pose.pose.orientation)
        self._has_odom = True

    def _on_amcl_pose(self, msg: Odometry):
        self._amcl_pose = msg

    # ── TF tag pose lookup ─────────────────────────────────────────

    def _lookup_tag_pose(self):
        """Query TF for tag pose in base_link frame.

        Converts from camera optical frame (z-forward, x-right) to
        base_link convention (x-forward, y-left, REP-103).

        Results are EMA-filtered and stored in self._raw_* and self._filtered_*.
        """
        base_frame = self._p('base_frame')
        measure_frame = self._p('measure_frame')

        src_frame = measure_frame if measure_frame else base_frame

        if not self._tag_frame:
            return False

        try:
            t = self._tf_buffer.lookup_transform(
                src_frame, self._tag_frame,
                rclpy.time.Time(seconds=0),
                rclpy.duration.Duration(seconds=0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            self._tf_fail_count += 1
            return False

        self._tf_fail_count = 0

        # TF 查询 src_frame→tag 的结果已经是 src_frame 坐标系的表示。
        # base_link 遵循 REP-103: x=前, y=左, z=上。
        # 直接取 x 为距离, y 为横向，不需要做光学坐标系转换。
        raw_dist = t.transform.translation.x
        raw_lat = t.transform.translation.y

        # Tag yaw = 方位角 (bearing): 机器人需要转多少弧度才能正对 Tag。
        # atan2(lat, dist) 是 tag 在机器人坐标系中的方向角，正=左边。
        tag_yaw_raw = math.atan2(raw_lat, raw_dist) if raw_dist > 0.001 else 0.0

        # Tag OUTWARD-NORMAL direction (rad, base_link ground plane). Needed for
        # the turn-drive-turn maneuver: the robot must reach the tag's normal
        # line and face the tag squarely, which requires the tag's orientation,
        # not just the bearing. Self-corrects the solvePnP flip ambiguity.
        tag_normal_raw = tag_normal_angle(t.transform.rotation, raw_dist, raw_lat)

        # Jump rejection
        if self._filter_init:
            jump_d = abs(raw_dist - self._filtered_dist) > self._max_pose_jump_m
            jump_l = abs(raw_lat - self._filtered_lat) > self._max_pose_jump_m
            if jump_d or jump_l:
                self._jump_reject_count += 1
                if self._jump_reject_count >= self._max_jump_rejections:
                    self._filtered_dist = raw_dist
                    self._filtered_lat = raw_lat
                    self._jump_reject_count = 0
                return False
            else:
                self._jump_reject_count = 0

        # EMA filtering
        alpha = self._ema_alpha
        if self._filter_init:
            self._filtered_dist = alpha * raw_dist + (1.0 - alpha) * self._filtered_dist
            self._filtered_lat = alpha * raw_lat + (1.0 - alpha) * self._filtered_lat
            self._filtered_yaw = alpha * tag_yaw_raw + (1.0 - alpha) * self._filtered_yaw
            # Circular EMA for the normal (wraps at ±pi): filter the sin/cos.
            self._normal_sin = alpha * math.sin(tag_normal_raw) + (1.0 - alpha) * self._normal_sin
            self._normal_cos = alpha * math.cos(tag_normal_raw) + (1.0 - alpha) * self._normal_cos
            self._filtered_normal = math.atan2(self._normal_sin, self._normal_cos)
        else:
            self._filtered_dist = raw_dist
            self._filtered_lat = raw_lat
            self._filtered_yaw = tag_yaw_raw
            self._normal_sin = math.sin(tag_normal_raw)
            self._normal_cos = math.cos(tag_normal_raw)
            self._filtered_normal = tag_normal_raw
            self._filter_init = True

        self._raw_dist = raw_dist
        self._raw_lat = raw_lat
        self._raw_yaw = tag_yaw_raw
        self._raw_normal = tag_normal_raw

        now_ns = self.get_clock().now().nanoseconds
        # Track the inter-detection interval and adapt the staleness window.
        if self._last_detection_ns != 0:
            gap = now_ns - self._last_detection_ns
            # Ignore huge gaps (tag was out of view): they are not the frame
            # rate, only genuine consecutive detections estimate the cadence.
            if 0 < gap < 2_000_000_000:  # < 2s
                if self._det_interval_ns is None:
                    self._det_interval_ns = float(gap)
                else:
                    self._det_interval_ns = 0.3 * gap + 0.7 * self._det_interval_ns
                # Window = a few detection intervals, never below the floor,
                # so one dropped frame at low rate does not orphan the buffer.
                adaptive = self._det_interval_ns * self._latency_margin
                self._pose_buffer.set_max_latency_ns(
                    max(self._latency_floor_ns, int(adaptive)))
        self._last_detection_ns = now_ns

        stamp = self._last_detection_ns
        pose = TagPose(dist=self._filtered_dist, lat=self._filtered_lat,
                       yaw=self._filtered_yaw, normal=self._filtered_normal,
                       stamp_ns=stamp)
        self._pose_buffer.add(pose)
        # Remember the side the tag was last seen on, to bias recovery search.
        if abs(self._filtered_lat) > 1e-3:
            self._last_seen_lat = self._filtered_lat
        return True

    def _tag_fresh(self) -> bool:
        """Check if tag detection is within freshness window."""
        if self._last_detection_ns == 0:
            return False
        timeout_ns = int(self._p('tag.fresh_timeout_sec') * 1e9)
        now_ns = self.get_clock().now().nanoseconds
        return (now_ns - self._last_detection_ns) < timeout_ns

    def _get_latest_pose(self):
        """Get latest valid pose from buffer (with latency check)."""
        now_ns = self.get_clock().now().nanoseconds
        return self._pose_buffer.get_latest(now_ns)

    # ── Control loop (20 Hz) ───────────────────────────────────────

    def _control_loop(self):
        """Main 20 Hz control loop — stop-and-go paradigm."""
        now_ns = self.get_clock().now().nanoseconds

        # Gather inputs
        tag_visible = self._tag_fresh()
        tag_pose = self._get_latest_pose()
        base_type = self._p('base.type')

        # Compute errors (for publishing and state machine)
        error_x, error_y, error_yaw = 0.0, 0.0, 0.0
        if tag_pose is not None:
            error_x = tag_pose.dist - self._p('dock_target.distance')
            error_y = tag_pose.lat - self._p('dock_target.lateral_offset')
            error_yaw = normalize_angle(
                tag_pose.yaw - math.radians(self._p('dock_target.yaw_offset_deg')))

        # Navigation status
        nav_done = self._nav.is_done()
        nav_success = self._nav.success()

        # State machine evaluation
        params = self._build_params_dict()
        self._sm.evaluate(
            tag_pose=tag_pose,
            tag_visible=tag_visible,
            odom_x=self._odom_x,
            odom_y=self._odom_y,
            odom_yaw=self._odom_yaw,
            cmd_vx=0.0, cmd_vy=0.0, cmd_wz=0.0,  # not used in stop-and-go
            motion_stalled=False,  # handled by action_executor
            nav_done=nav_done,
            nav_success=nav_success,
            now_ns=now_ns,
            params=params,
            maneuver_active=self.maneuver_active,
        )

        state = self._sm.state

        # On leaving the stop-and-go states (e.g. APPROACH→SEARCH_TAG re-lock,
        # or any error/terminal transition), abort any half-finished action so
        # it cannot resume later against a stale odometry reference.
        stopgo_states = (DockingState.ALIGN, DockingState.APPROACH,
                         DockingState.FINAL_SERVO)
        if (self._prev_state in stopgo_states and state not in stopgo_states):
            self._executor.cancel()
            self._planner.reset()
            self._reset_maneuver()
        # Entering SEARCH_TAG (fresh start or re-lock): restart the widening
        # bidirectional sweep from scratch so each search begins narrow and
        # grows, rather than resuming a wide sweep left over from a prior attempt.
        if (state == DockingState.SEARCH_TAG
                and self._prev_state != DockingState.SEARCH_TAG):
            self._search_sweep_idx = 0
            self._search_last_phase_start = 0
        self._prev_state = state

        # ── Per-state behaviour ───────────────────────────────────
        if state == DockingState.NAVIGATING:
            self._adapter.publish_stop()

        elif state == DockingState.SEARCH_TAG:
            vx, vy, wz = self._compute_search_velocity(now_ns)
            if abs(wz) > 0:
                # Rotate IN PLACE to find the tag. No forward creep here: search
                # is a large continuous rotation (momentum already beats static
                # friction), and creeping forward over a 60 s search would walk
                # the robot across the room. Arc-creep is only for the small,
                # stall-prone APPROACH turns.
                self._adapter.publish_turn(wz)
            elif abs(vx) > 0:
                self._adapter.publish_jog(vx)
            else:
                self._adapter.publish_stop()

        elif state in (DockingState.ALIGN, DockingState.APPROACH,
                       DockingState.FINAL_SERVO):
            self._run_stop_and_go(tag_visible, tag_pose, base_type, now_ns)

        else:  # IDLE, DOCKED, error states
            self._adapter.publish_stop()

        # Publish state and error
        self._publish_state(state)
        self._publish_error(error_x, error_y, error_yaw)

    # ── Stop-and-go loop ───────────────────────────────────────────

    def _run_stop_and_go(self, tag_visible: bool, tag_pose, base_type: str,
                         now_ns: int):
        """One tick of the turn-drive-turn stop-and-go loop.

        Three nested cases:
          1. An executor action is running  → update its odometry; when it
             finishes, immediately start the NEXT queued sub-step (no settle,
             no re-measure — the maneuver is blind).
          2. The maneuver queue just emptied → settle, then re-measure + re-plan.
          3. Idle & settled                 → measure the tag and plan a fresh
             turn-drive-turn sequence (or declare done).
        """
        target_distance = self._p('dock_target.distance')

        # ── Case 1: a sub-step is executing ───────────────────────────
        if self._executor.is_active:
            done = self._executor.update(
                self._odom_x, self._odom_y, self._odom_yaw,
                tag_visible, self._raw_dist,
                self._bearing_fn,
                self._theta_bounds_fn,
                target_distance,
                self._p('stopgo.drift_tol'),
                now_ns,
            )
            if done:
                if self._maneuver_queue:
                    # Chain straight into the next sub-step by odometry — no
                    # settle, tag not consulted. This is the whole point: the
                    # maneuver runs open-loop on odometry so a narrow FOV losing
                    # the tag mid-turn cannot derail it.
                    self._start_next_maneuver_step(base_type)
                else:
                    # Whole sequence finished → settle before re-measuring.
                    self._maneuver_active = False
                    self._executor.mark_stop_time(now_ns)
            self._publish_action_cmd(base_type)
            return

        # ── Case 2: settle after a completed maneuver ─────────────────
        if self._executor.wait_visual_settle(tag_visible, now_ns):
            self._adapter.publish_stop()
            self.get_logger().info(
                '走停：机动结束后等待稳定（已过 '
                f'{(now_ns - (self._executor._last_stop_ns or now_ns))*1e-9:.2f}s）',
                throttle_duration_sec=1.0)
            return

        # ── settle 窗口刚结束：解冻并清空运动期的一切旧数据 ────────────
        # 强制下一次规划只用小车停稳后新采的新鲜帧。清空后本 tick 不规划，
        # 等 _on_detections（已解冻）收到一帧停稳后的检测重新播种滤波。
        if self._frozen:
            self._frozen = False
            self._filter_init = False          # EMA 重新播种（首帧新鲜帧作种子）
            self._last_detection_ns = 0        # _tag_fresh() 归零，等待新帧
            self._pose_buffer.clear()          # 丢弃所有历史缓冲位姿
            self._adapter.publish_stop()
            self.get_logger().info('走停：机动结束，已清空旧位姿，等待停稳后的新鲜帧重新测量')
            return

        # ── Case 3: idle & settled — measure and plan a fresh sequence ─
        if not tag_visible or tag_pose is None:
            self._adapter.publish_stop()
            self.get_logger().info(
                f'走停：空闲停车（二维码可见={tag_visible}, '
                f'位姿={"无" if tag_pose is None else "有"}）',
                throttle_duration_sec=1.0)
            return

        if self._maneuver_iters >= self._max_maneuver_iters:
            self.get_logger().warn(
                f'走停：已达最大机动迭代次数（{self._max_maneuver_iters}），'
                '停止；最后位姿已在可达范围内')
            self._adapter.publish_stop()
            return

        yaw_tol = math.radians(self._p('tolerance.yaw_deg'))
        seq = self._planner.plan_sequence(
            tag_pose.dist, tag_pose.lat, tag_pose.normal, yaw_tol=yaw_tol)

        # 移动前打印：二维码相对位姿 + 完整规划路径，仅凭日志即可诊断丢标问题。
        # bearing = 指向二维码的方向；normal = 二维码朝外法线方向；
        # 每一步以带符号量显示（转向单位度，前进单位米）。
        bearing_deg = math.degrees(math.atan2(tag_pose.lat, tag_pose.dist))
        steps = []
        for p in seq:
            if p.kind == 'yaw':
                steps.append(f'转 {math.degrees(p.turn_angle):+.1f}°')
            elif p.kind == 'forward':
                steps.append(f'前进 {p.jog_distance:+.3f}m')
            else:
                steps.append(p.kind)
        self.get_logger().info(
            f'走停 规划 iter={self._maneuver_iters}: '
            f'二维码 距离={tag_pose.dist:.3f}m 横向={tag_pose.lat:+.3f}m '
            f'方位={bearing_deg:+.1f}° 法线={math.degrees(tag_pose.normal):+.1f}° '
            f'| 原始 距离={self._raw_dist:.3f} 横向={self._raw_lat:+.3f} '
            f'法线={math.degrees(self._raw_normal):+.1f}° '
            f'| 路径 [{", ".join(steps)}]')

        if len(seq) == 1 and seq[0].kind == 'done':
            self._adapter.publish_stop()
            # Leave APPROACH→FINAL_SERVO/DOCKED to the state machine (it checks
            # the same tolerance on tag_pose).
            return

        # Load the sequence and launch its first sub-step.
        self._maneuver_queue = list(seq)
        self._maneuver_active = True
        self._frozen = True          # 开始盲动：冻结检测，运动期丢弃所有帧
        self._maneuver_iters += 1
        self._start_next_maneuver_step(base_type)

    def _start_next_maneuver_step(self, base_type: str):
        """Pop and start the next queued sub-step, re-referencing odometry.

        Skips over sub-steps too small to actually move (a sub-degree turn or a
        sub-millimetre jog): those would leave the executor idle, so without the
        skip the blind chain would stall (Case 1 only advances the queue when an
        action was running). Keeps popping until one step starts or the queue
        empties.
        """
        while self._maneuver_queue:
            step = self._maneuver_queue.pop(0)
            if self._launch_step(step, base_type):
                return
        # Queue drained without launching anything → maneuver is over.
        self._maneuver_active = False
        self._executor.mark_stop_time(self.get_clock().now().nanoseconds)

    def _launch_step(self, plan: ActionPlan, base_type: str) -> bool:
        """Start one executor action from an ActionPlan and re-ref odometry.

        Returns True if an action actually started, False if the step was too
        small to move (caller advances to the next queued step).
        """
        if plan.kind == 'yaw':
            # Full computed turn — no undershoot, no per-step cap. The whole
            # turn-drive-turn path was computed together; a clamped turn1 would
            # drive the full leg along the wrong heading.
            if not self._executor.start_turn(
                    plan.turn_angle, self._p('stopgo.jog_angular_rate'),
                    full=True):
                return False
            self._executor.set_odom_ref(
                self._odom_x, self._odom_y, self._odom_yaw)
            self.get_logger().info(
                f'  子步：原地转 {math.degrees(plan.turn_angle):+.1f}°（盲转，里程计校准）')
        elif plan.kind == 'forward':
            if abs(plan.lateral_distance) > 1e-4 and self._is_omni(base_type):
                self._executor.start_jog_lateral(
                    plan.lateral_distance, self._p('stopgo.lateral_rate'))
                if not self._executor.is_active:
                    return False
                self._executor.set_odom_ref(
                    self._odom_x, self._odom_y, self._odom_yaw)
                self.get_logger().info(
                    f'  子步：横移 {plan.lateral_distance:+.3f}m')
            else:
                # Blind straight leg — odometry only, no visual early-stop.
                self._executor.start_jog(
                    plan.jog_distance, self._p('stopgo.jog_linear_rate'),
                    blind=True)
                if not self._executor.is_active:
                    return False
                self._executor.set_odom_ref(
                    self._odom_x, self._odom_y, self._odom_yaw, bearing=0.0)
                self.get_logger().info(
                    f'  子步：前进 {plan.jog_distance:+.3f}m（盲走，里程计校准）')
        else:
            return False
        self._publish_action_cmd(base_type)
        return True

    @property
    def maneuver_active(self) -> bool:
        """True while a blind turn-drive-turn maneuver is executing.

        During this window the tag legitimately leaves view, so the state
        machine must NOT count tag-loss toward the SEARCH_TAG fallback.
        """
        return self._maneuver_active or self._executor.is_active

    def _publish_action_cmd(self, base_type: str):
        """Publish the current action's velocity command via the adapter."""
        kind = self._executor.action_kind
        if kind == 'jogging':
            self._adapter.publish_jog(self._executor.linear_cmd)
        elif kind == 'turning':
            # 纯原地转弯。之前的"边走边转"(arc)方案会叠加前进速度，可能让小车
            # 驶出目标的横向范围——已弃用。转向精度靠里程计校准（full=True 全量
            # 盲转），转得慢一点没关系；若差速轮原地转需克服静摩擦，宁可加大
            # jog_angular_rate，也不叠加前向速度。
            self._adapter.publish_turn(self._executor.angular_cmd)
        elif kind == 'lateral':
            if hasattr(self._adapter, 'publish_lateral'):
                self._adapter.publish_lateral(self._executor.lateral_cmd)
            else:
                self._adapter.publish_stop()
        else:
            self._adapter.publish_stop()

    def _reset_maneuver(self):
        """Clear any queued/active blind maneuver and its iteration counter.

        Called on cancel/abort and whenever we leave the stop-and-go states, so
        a fresh docking attempt always re-plans from a new measurement and no
        stale sub-step can resume against an outdated odometry reference.
        """
        self._maneuver_queue = []
        self._maneuver_active = False
        self._maneuver_iters = 0
        self._frozen = False

    # ── Helpers for the action executor ────────────────────────────

    def _bearing_fn(self) -> float:
        """Current tag bearing from robot (rad)."""
        if self._raw_dist is None or self._raw_lat is None:
            return 0.0
        return math.atan2(self._raw_lat, self._raw_dist)

    def _theta_bounds_fn(self) -> float:
        """Dynamic theta tolerance — tighter when closer."""
        if self._raw_dist is None or self._raw_dist <= 0.0:
            return math.radians(self._p('stopgo.yaw_threshold_deg'))
        ratio = self._p('stopgo.theta_shrink_ratio')
        return max(
            math.radians(self._p('stopgo.yaw_threshold_deg')),
            self._raw_dist / max(ratio, 0.1),
        )

    def _blind_cap_for_turn(self) -> float | None:
        """Maximum turn angle when tag is not visible."""
        return min(
            self._p('stopgo.max_turn_step'),
            2.0 * self._theta_bounds_fn(),
        )

    @staticmethod
    def _is_omni(base_type: str) -> bool:
        return base_type in ('omni', 'quadruped')

    # ── Search velocity ────────────────────────────────────────────

    def _compute_search_velocity(self, now_ns: int) -> tuple[float, float, float]:
        """Velocity for SEARCH_TAG — bidirectional widening sweep.

        The old single-direction scan could never recover a tag lost off the
        opposite FOV edge. This alternates direction each rotate phase and
        widens the sweep, so both sides are covered:

            sweep 0: toward last-seen side, 1× base width
            sweep 1: opposite side,         2× width
            sweep 2: back,                  3× width
            ...

        Starting toward the side where the tag was last seen (sign of
        _last_seen_lat, +lat = left = CCW = +wz) finds it fastest in the common
        case where a turn just nudged it past the edge.
        """
        base_speed = self._p('search.angular_speed')
        rotate_time = self._p('search.rotate_time_sec')

        # Detect entry into a new rotate phase → advance the sweep index.
        if self._sm._search_phase == 'rotate':
            if self._sm._search_phase_start_ns != self._search_last_phase_start:
                self._search_last_phase_start = self._sm._search_phase_start_ns
                self._search_sweep_idx += 1
        else:
            return 0.0, 0.0, 0.0

        # First sweep direction: toward last-seen side (+lat → CCW → +).
        first_dir = 1.0 if self._last_seen_lat >= 0.0 else -1.0
        # Alternate each sweep.
        direction = first_dir * (1.0 if (self._search_sweep_idx % 2 == 1) else -1.0)
        # Widen: sweep 1→1×, 2→2×, 3→3×, capped at 4×.
        width = min(self._search_sweep_idx, 4)

        phase_ns = now_ns - self._sm._search_phase_start_ns
        if phase_ns * 1e-9 < rotate_time * width:
            return 0.0, 0.0, base_speed * direction
        return 0.0, 0.0, 0.0

    # ── Params dict ────────────────────────────────────────────────

    def _build_params_dict(self) -> dict:
        return {
            'timeout_sec': self._p('timeout_sec'),
            'navigation': {
                'nav2_timeout_sec': self._p('navigation.nav2_timeout_sec'),
                'pre_dock_distance': self._p('navigation.pre_dock_distance'),
            },
            'search': {
                'angular_speed': self._p('search.angular_speed'),
                'rotate_time_sec': self._p('search.rotate_time_sec'),
                'pause_time_sec': self._p('search.pause_time_sec'),
                'hold_time_sec': self._p('search.hold_time_sec'),
                'search_direction': self._p('search.search_direction'),
                'timeout_sec': self._p('search.timeout_sec'),
            },
            'tolerance': {
                'position_m': self._p('tolerance.position_m'),
                'yaw_deg': self._p('tolerance.yaw_deg'),
                'stable_time_sec': self._p('tolerance.stable_time_sec'),
            },
            'dock_target': {
                'distance': self._p('dock_target.distance'),
                'lateral_offset': self._p('dock_target.lateral_offset'),
                'yaw_offset_deg': self._p('dock_target.yaw_offset_deg'),
            },
            'safety': {
                'minimum_distance_m': self._p('safety.minimum_distance_m'),
            },
            'align_timeout_sec': self._p('align_timeout_sec'),
            'approach_timeout_sec': self._p('approach_timeout_sec'),
            'final_servo_timeout_sec': self._p('final_servo_timeout_sec'),
        }

    # ── Publishing ─────────────────────────────────────────────────

    def _publish_state(self, state: DockingState):
        msg = String()
        msg.data = state.name.lower()
        self._state_pub.publish(msg)

    def _publish_error(self, ex: float, ey: float, eyaw: float):
        msg = Vector3()
        msg.x = ex
        msg.y = ey
        msg.z = eyaw
        self._error_pub.publish(msg)

    # ── Service callbacks ──────────────────────────────────────────

    def _on_start_docking(self, request, response):
        nav_enable = self._p('navigation.enable')
        ok = self._sm.start(enable_navigation=nav_enable)
        response.success = ok
        response.message = f'state={self._sm.state_name}' if ok else 'already active'
        if ok and nav_enable:
            self._start_navigation()
        self._planner.reset()
        self._executor.cancel()
        self._reset_maneuver()
        return response

    def _on_cancel_docking(self, request, response):
        self._sm.cancel()
        self._nav.cancel()
        self._planner.reset()
        self._executor.cancel()
        self._reset_maneuver()
        self._adapter.publish_stop()
        response.success = True
        response.message = 'cancelled'
        return response

    # ── Action callbacks ───────────────────────────────────────────

    def _execute_dock_cb(self, goal_handle):
        """Action execute callback — blocks until docking completes."""
        from tagdocking.action import Dock

        nav_enable = self._p('navigation.enable')
        ok = self._sm.start(enable_navigation=nav_enable)
        if not ok:
            goal_handle.abort()
            return Dock.Result(success=False, message='already active')
        if nav_enable:
            self._start_navigation()

        self._planner.reset()
        self._executor.cancel()
        self._reset_maneuver()

        feedback = Dock.Feedback()

        while rclpy.ok() and self._sm.is_active:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._sm.cancel()
                self._nav.cancel()
                self._planner.reset()
                self._executor.cancel()
                self._reset_maneuver()
                self._adapter.publish_stop()
                return Dock.Result(success=False, message='cancelled')

            tag_pose = self._get_latest_pose()
            if tag_pose is not None:
                target_dist = self._p('dock_target.distance')
                feedback.distance_error = abs(tag_pose.dist - target_dist)
                feedback.yaw_error = abs(tag_pose.yaw)
            feedback.state = self._sm.state_name.lower()
            goal_handle.publish_feedback(feedback)

            rclpy.spin_once(self, timeout_sec=0.05)

        if self._sm.is_success:
            goal_handle.succeed()
            return Dock.Result(success=True, message='docked')
        else:
            goal_handle.abort()
            return Dock.Result(success=False, message=self._sm.state_name.lower())

    def _dock_cancel_cb(self, cancel_request):
        self._sm.cancel()
        self._nav.cancel()
        self._planner.reset()
        self._executor.cancel()
        self._reset_maneuver()
        self._adapter.publish_stop()
        return CancelResponse.ACCEPT

    # ── Navigation ─────────────────────────────────────────────────

    def _start_navigation(self):
        """Compute and send Nav2 pre-dock goal."""
        from geometry_msgs.msg import PoseStamped, Quaternion

        pre_dock_x = 0.0
        pre_dock_y = 0.0
        pre_dock_yaw = 0.0
        pre_dock_frame = self._p('navigation.pre_dock_frame')

        if self._amcl_pose is not None:
            pre_dock_x = self._amcl_pose.pose.pose.position.x
            pre_dock_y = self._amcl_pose.pose.pose.position.y
            pre_dock_yaw = yaw_from_quat(self._amcl_pose.pose.pose.orientation)

        pose = PoseStamped()
        pose.header.frame_id = pre_dock_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = pre_dock_x
        pose.pose.position.y = pre_dock_y
        pose.pose.position.z = 0.0
        pose.pose.orientation = Quaternion()
        pose.pose.orientation.z = math.sin(pre_dock_yaw / 2.0)
        pose.pose.orientation.w = math.cos(pre_dock_yaw / 2.0)

        self._nav.send_goal(pose)

    # ── Emergency stop ─────────────────────────────────────────────

    def _install_signal_handlers(self):
        stopping = {'flag': False}

        def _handle_signal(signum, frame):
            if stopping['flag']:
                return
            stopping['flag'] = True
            try:
                self.get_logger().info(f'紧急停止（信号 {signum}）')
            except Exception:
                pass
            for _ in range(100):
                try:
                    self._adapter.publish_stop()
                except Exception:
                    pass
                time.sleep(0.01)
            try:
                rclpy.shutdown()
            except Exception:
                pass

        signal.signal(signal.SIGINT, _handle_signal)
        signal.signal(signal.SIGTERM, _handle_signal)

    def _safe_stop(self):
        try:
            self._adapter.publish_stop()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = DockingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        # The signal handler may have already shut down the context; calling
        # rclpy.shutdown() again raises "Context must be initialized". Guard it
        # so launch gets a clean exit instead of SIGKILL-ing us into zombies.
        if rclpy.ok():
            rclpy.shutdown()
