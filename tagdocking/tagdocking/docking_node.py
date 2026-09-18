"""Main docking node — integrates all subsystems (stop-and-go paradigm).

Architecture:
  Subscriptions:
    /detections      → AprilTag detection array
    /tf              → (via tf2_ros.Buffer) tag pose lookup
    /odom            → odometry feedback

  Publishers:
    /cmd_vel         → (via BaseAdapter) velocity commands
    ~/state          → std_msgs/String — current state name
    ~/error          → geometry_msgs/Vector3 — (error_x, error_y, error_yaw)

  Services:
    ~/start_docking  → std_srvs/Trigger — start docking
    ~/cancel_docking → std_srvs/Trigger — cancel docking
    ~/start_undock   → std_srvs/Trigger — pull out (reverse + 180° turn)

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

        # ── State machine ─────────────────────────────────────────
        self._sm = DockingStateMachine(self)
        self._sm._max_retries = int(self._p('retry.max_retries'))

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
        # 直行失败标志：进入直行距离时方位误差超门槛只判一次（首次进入），
        # 防止直行中方位自然漂动误触发失败。_reset_maneuver 时清零。
        self._straight_failed = False
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

        # ── Undock (泊出) sub-phase ──────────────────────────────────
        # 0 = 盲退 undock.backup_distance, 1 = 原地转 180°, 2 = 完成。
        # 由 _run_undock 在 UNDOCKING 态驱动, 纯里程计闭环, 不看 tag。
        self._undock_phase = 0

        # Recovery-search state: remember which side the tag was last seen on
        # (sign of lat) so the angle-stepped sweep starts toward it. The node
        # rotates a fixed angle (odometry-closed), stops, detects, repeats —
        # sweeping a full 360° in one direction until the tag is found.
        self._last_seen_lat = 0.0
        self._search_step = 0
        self._search_detect_start = 0
        self._search_direction = 1.0

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

        # Control period
        self._dt = 0.05  # 20 Hz

        # Quiescent cmd_vel policy: brake briefly on entering an idle/terminal
        # state, then release /cmd_vel so teleop can drive the robot.
        # Continuously publishing zero at 20 Hz otherwise monopolises the topic
        # and locks out manual control after docking. The chassis watchdog stops
        # the robot if nobody publishes.
        self._quiescent = False
        self._brake_until_ns = 0
        self._BRAKE_WINDOW_NS = int(0.3 * 1e9)   # ~0.3 s firm brake on entry

        # ── ROS interfaces ─────────────────────────────────────────
        self._init_ros_interfaces()

        # ── Timer ──────────────────────────────────────────────────
        self._timer = self.create_timer(self._dt, self._control_loop)

        # ── Shutdown ───────────────────────────────────────────────
        rclpy.get_default_context().on_shutdown(self._safe_stop)
        self._install_signal_handlers()

        self.get_logger().info(
            f'停靠节点就绪 | 底盘={self._p("base.type")} | 走停模式')

    # ── Parameter helpers ──────────────────────────────────────────

    def _declare_params(self):
        """Declare all ROS2 parameters with defaults."""
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

        # Retry (失败后倒车一段距离再重新 dock)
        self.declare_parameter('retry.max_retries', 2)
        self.declare_parameter('retry.backup_distance', 0.5)
        self.declare_parameter('retry.linear_rate', 0.08)
        self.declare_parameter('retry.timeout_sec', 15.0)

        # Undock (泊出: 盲退一段距离 → 原地转 180° → 完成)
        self.declare_parameter('undock.backup_distance', 0.5)
        self.declare_parameter('undock.linear_rate', 0.08)
        self.declare_parameter('undock.turn_angle_deg', 180.0)   # 正=CCW, 负=CW
        self.declare_parameter('undock.angular_rate', 0.3)
        self.declare_parameter('undock.timeout_sec', 30.0)

        # Search
        self.declare_parameter('search.angular_speed', 0.3)
        self.declare_parameter('search.step_angle_deg', 30.0)
        self.declare_parameter('search.rotate_time_sec', 0.8)  # deprecated, unused
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

        # Final straight (两阶段停泊: 85cm 对准 → 55cm 纯直行)
        self.declare_parameter('final_straight.enable', True)
        self.declare_parameter('final_straight.start_distance', 0.85)
        self.declare_parameter('final_straight.yaw_threshold_deg', 3.0)

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
        self.declare_parameter('odom_topic', '/odom_combined')

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

        self._state_pub = self.create_publisher(String, '~/state', 10)
        self._error_pub = self.create_publisher(Vector3, '~/error', 10)

        self._srv_start = self.create_service(
            Trigger, '~/start_docking', self._on_start_docking)
        self._srv_cancel = self.create_service(
            Trigger, '~/cancel_docking', self._on_cancel_docking)
        self._srv_undock = self.create_service(
            Trigger, '~/start_undock', self._on_start_undock)

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
        # 出 UNDOCKING: 终止可能还在跑的盲退/盲转, 清掉 _frozen, 否则下一次
        # 停泊会带着冻结态启动、丢弃所有检测帧。
        if (self._prev_state == DockingState.UNDOCKING
                and state != DockingState.UNDOCKING):
            self._executor.cancel()
            self._reset_maneuver()
        # 出 SEARCH_TAG：终止可能正在转的搜索步 + 解冻
        if (self._prev_state == DockingState.SEARCH_TAG
                and state != DockingState.SEARCH_TAG):
            self._reset_search()
        # 入 SEARCH_TAG (全新开始或丢标重锁)：从头开始角度步进扫描
        if (state == DockingState.SEARCH_TAG
                and self._prev_state != DockingState.SEARCH_TAG):
            self._reset_search()
        self._prev_state = state

        # ── Per-state behaviour ───────────────────────────────────
        # Active motion states own /cmd_vel and command motion each tick.
        # Quiescent states (IDLE/DOCKED/errors) must NOT keep publishing zero —
        # that monopolises /cmd_vel and locks out teleop. Brake briefly on
        # entry, then release the topic so other publishers can drive the robot.
        if state == DockingState.SEARCH_TAG:
            self._quiescent = False
            self._run_search(tag_visible, tag_pose, base_type, now_ns)

        elif state in (DockingState.ALIGN, DockingState.APPROACH,
                       DockingState.FINAL_SERVO):
            self._quiescent = False
            self._run_stop_and_go(tag_visible, tag_pose, base_type, now_ns)

        elif state == DockingState.RETRYING:
            self._quiescent = False
            self._run_retry(base_type, now_ns)

        elif state == DockingState.UNDOCKING:
            self._quiescent = False
            self._run_undock(base_type, now_ns)

        else:  # IDLE, DOCKED, UNDOCKED, error states
            self._brake_and_release(now_ns)

        # Publish state and error
        self._publish_state(state)
        self._publish_error(error_x, error_y, error_yaw)

    # ── Quiescent cmd_vel policy ────────────────────────────────────

    def _brake_and_release(self, now_ns: int):
        """Quiescent-state /cmd_vel policy: brake briefly on entry, then release.

        Continuously publishing Twist() at 20 Hz while idle/docked monopolises
        /cmd_vel — every teleop command is overwritten by zero within 50 ms, so
        the robot appears "locked" until the docking launch is killed. Instead
        publish a short burst of stops on the entry edge (firm brake in case the
        robot still has residual velocity), then publish nothing and let teleop
        own the topic. The chassis watchdog stops the robot if nobody publishes.
        """
        if not self._quiescent:
            self._quiescent = True
            self._brake_until_ns = now_ns + self._BRAKE_WINDOW_NS
        if now_ns < self._brake_until_ns:
            self._adapter.publish_stop()

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

        # ── 两阶段停泊门控 ──────────────────────────────────────
        # 阶段1(带角度修正)为默认：用 plan_sequence 转向对准+前进, 尽量对准。
        # 阶段2(纯直行, 不修 yaw/横向)：dist ≤ start_distance 即无条件激活——
        # 进入直行距离后像停车入库, 不再打方向(再往前已无空间调位姿)。
        # 进入时做一次性失败检查：方位误差超 yaw_threshold_deg(默认10°)说明
        # 对准过差、入库会撞偏 → 报 MOTION_FAILED。_straight_failed 保证只判一次,
        # 防止直行中方位自然漂动(tag 抖动+车体微偏)误触发失败。
        # start_distance ≤ target_distance 视为误配置, 静默回退单阶段。
        straight_enabled = self._p('final_straight.enable')
        straight_start = self._p('final_straight.start_distance')
        straight_yaw_tol = math.radians(self._p('final_straight.yaw_threshold_deg'))

        # 方位误差用车体朝向(bearing=atan2(lat,dist))，不用方阵误差(square_err)。
        # square_err 依赖 tag 法线(normal)，而 normal 是 AprilTag 最不可靠的自由度
        # ——近场时在 ±180° 附近抖动，经 ±π 归一化后误差被放大到 4~5°，即使车已
        # 正对标签(方位角<1°)也会误判超差。bearing 只取决于标签在画面中的位置，稳定可靠。
        bearing = math.atan2(tag_pose.lat, tag_pose.dist)
        bearing_err = abs(normalize_angle(bearing))

        go_straight = False
        if straight_enabled and straight_start > target_distance:
            if tag_pose.dist <= straight_start:
                # 进入直行距离 → 无条件直行（不再调角）。
                # 仅首次进入时做一次失败检查：方位误差超门槛 → 报导航失败。
                if not self._straight_failed and bearing_err > straight_yaw_tol:
                    self._straight_failed = True
                    self.get_logger().error(
                        f'直行失败：进入直行距离({tag_pose.dist:.2f}m)时方位误差 '
                        f'{math.degrees(bearing_err):.1f}° > 门槛 '
                        f'{math.degrees(straight_yaw_tol):.1f}°，对准过差无法入库')
                    self._sm.fail()
                    self._adapter.publish_stop()
                    return
                go_straight = True
            elif (bearing_err <= straight_yaw_tol
                  and abs(tag_pose.lat) <= self._p('stopgo.lateral_threshold')):
                # 阶段1 已基本对准(方位+横向都在容差内) → 直接直行, 不再做
                # turn-drive-turn。plan_sequence 的 turn1/turn2 依赖 tag 法线
                # normal, 而 normal 是 AprilTag 最不可靠的自由度(近场 ±180° 抖动,
                # 实测已对正标签仍被算成 ~8° 偏角), 会平白多一次 [转+10° 前进 转-10°]
                # 的摆头。方位(bearing)只依赖标签在画面中的位置, 稳定可靠。
                go_straight = True

        if go_straight:
            seq = self._planner.plan_straight(tag_pose.dist)
            self.get_logger().info(
                f'走停 直线阶段 iter={self._maneuver_iters}: '
                f'距离={tag_pose.dist:.3f}m → 目标={target_distance:.3f}m',
                throttle_duration_sec=2.0)
        else:
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
            f'| 直行={go_straight} 方位误差={math.degrees(bearing_err):.1f}°'
            f'(失败门槛{math.degrees(straight_yaw_tol):.1f}°) '
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

    def _run_retry(self, base_type: str, now_ns: int):
        """重试倒车的一个 tick: 盲退 retry.backup_distance, 到位后 → SEARCH_TAG。

        镜像 _run_search 的 Case 1/2 结构。倒车纯里程计闭环(blind), 不看 tag;
        退够距离后调状态机 retry_search() 转 SEARCH_TAG 重新锁定靠近。
        进入 RETRYING 时控制循环的 cleanup(离开 stopgo 态)已 cancel 旧 executor
        + _reset_maneuver, 故首 tick executor 空闲 → Case 2 启动盲退。
        """
        # Case 1: 倒车执行中
        if self._executor.is_active:
            done = self._executor.update(
                self._odom_x, self._odom_y, self._odom_yaw,
                False, None,   # blind: 不看 tag
                self._bearing_fn, self._theta_bounds_fn,
                self._p('dock_target.distance'),
                self._p('stopgo.drift_tol'), now_ns,
            )
            if done:
                self._maneuver_active = False
                self._executor.mark_stop_time(now_ns)
                self._sm.retry_search()
            self._publish_action_cmd(base_type)
            return

        # Case 2: 倒车未开始 → 启动盲退(负距离 = 后退)
        dist = abs(float(self._p('retry.backup_distance')))
        rate = float(self._p('retry.linear_rate'))
        if dist < 1e-3:
            self._sm.retry_search()
            self._adapter.publish_stop()
            return
        self._executor.start_jog(-dist, rate, blind=True)
        self._executor.set_odom_ref(self._odom_x, self._odom_y, self._odom_yaw)
        self._maneuver_active = True
        self._frozen = True
        self.get_logger().info(
            f'重试：盲退 {-dist:+.3f}m (速率 {rate:.2f}m/s) 后重新锁定')
        self._publish_action_cmd(base_type)

    def _run_undock(self, base_type: str, now_ns: int):
        """泊出的一个 tick: 盲退 → 原地转 180° → UNDOCKED。

        两段纯里程计闭环盲动顺序执行 (镜像 _run_retry 的 Case 结构)：
          phase 0: 盲退 undock.backup_distance (负 jog, 同重试倒车)
          phase 1: 原地转 undock.turn_angle_deg (默认 180°)
        两段都到位后调状态机 finish_undock() → UNDOCKED。
        不看 tag；运动期冻结检测 (与停泊盲动一致)。
        """
        # Case 1: 子动作执行中
        if self._executor.is_active:
            done = self._executor.update(
                self._odom_x, self._odom_y, self._odom_yaw,
                False, None,   # blind: 不看 tag
                self._bearing_fn, self._theta_bounds_fn,
                self._p('dock_target.distance'),
                self._p('stopgo.drift_tol'), now_ns,
            )
            if done:
                self._maneuver_active = False
                self._executor.mark_stop_time(now_ns)
                self._undock_phase += 1
                if not self._start_undock_step(base_type):
                    # 两段盲动均完成 → 泊出成功
                    self._adapter.publish_stop()
                    self._sm.finish_undock()
                    return
            self._publish_action_cmd(base_type)
            return

        # Case 2: 首次进入 → 启动第一段(盲退)
        if not self._has_odom:
            self._adapter.publish_stop()
            self.get_logger().warn('泊出：等待里程计...', throttle_duration_sec=1.0)
            return
        self._undock_phase = 0
        if not self._start_undock_step(base_type):
            self._adapter.publish_stop()
            self._sm.finish_undock()
            return
        self._publish_action_cmd(base_type)

    def _start_undock_step(self, base_type: str) -> bool:
        """启动 _undock_phase 指示的泊出子动作。

        phase 0 = 盲退, phase 1 = 原地转 180°。
        返回 True = 已启动一个动作; False = 该相位空跳或已全部完成(调用方据此收尾)。
        空跳(距离/角度过小)时自动推进到下一相位再试, 与 _start_next_maneuver_step
        的"跳过过小子步"行为一致。
        """
        if self._undock_phase == 0:
            dist = abs(float(self._p('undock.backup_distance')))
            rate = float(self._p('undock.linear_rate'))
            if dist < 1e-3:
                self._undock_phase += 1   # 距离为 0, 跳过盲退直接转
            else:
                self._executor.start_jog(-dist, rate, blind=True)
                self._executor.set_odom_ref(
                    self._odom_x, self._odom_y, self._odom_yaw)
                self._maneuver_active = True
                self._frozen = True
                self.get_logger().info(
                    f'泊出：盲退 {-dist:+.3f}m (速率 {rate:.2f}m/s)')
                return True
        if self._undock_phase == 1:
            angle = math.radians(float(self._p('undock.turn_angle_deg')))
            rate = float(self._p('undock.angular_rate'))
            if self._executor.start_turn(angle, rate, full=True):
                self._executor.set_odom_ref(
                    self._odom_x, self._odom_y, self._odom_yaw)
                self._maneuver_active = True
                self._frozen = True
                self.get_logger().info(
                    f'泊出：原地转 {math.degrees(angle):+.1f}°')
                return True
            self._undock_phase += 1   # 角度过小未启动 → 视为完成
        return False

    # ── Angle-stepped search loop ─────────────────────────────────

    def _run_search(self, tag_visible: bool, tag_pose, base_type: str,
                    now_ns: int):
        """角度步进搜索的一个 tick：转固定角度(里程计闭环)→停稳→检测→再转。

        转满 360° 直到找到二维码或状态机超时。结构镜像 _run_stop_and_go 的
        Case 级联（执行中→等待稳定→解冻清旧数据→空闲检测），区别是检测期
        不规划靠近动作，而是累计 pause_time_sec 不可见就再转一个 step_angle。
        冻结机制保证转动期 tag_visible 恒为 False，状态机的 tag-lock 不会误触发。
        """
        # ── Case 1: 搜索步正在执行（里程计闭环盲转）──────────────
        if self._executor.is_active:
            done = self._executor.update(
                self._odom_x, self._odom_y, self._odom_yaw,
                tag_visible, self._raw_dist,
                self._bearing_fn, self._theta_bounds_fn,
                self._p('dock_target.distance'),
                self._p('stopgo.drift_tol'), now_ns,
            )
            if done:
                self._maneuver_active = False
                self._executor.mark_stop_time(now_ns)
            self._publish_action_cmd(base_type)
            return

        # ── Case 2: 转完后等待图像稳定 ────────────────────────────
        if self._executor.wait_visual_settle(tag_visible, now_ns):
            self._adapter.publish_stop()
            return

        # ── Case 2.5: 稳定窗口刚结束 → 解冻，强制下一帧只用停稳后的新鲜帧
        if self._frozen:
            self._frozen = False
            self._filter_init = False
            self._last_detection_ns = 0
            self._pose_buffer.clear()
            self._search_detect_start = 0
            self._adapter.publish_stop()
            return

        # ── Case 3: 空闲且已稳定 — 检测期 ─────────────────────────
        if not self._has_odom:
            self._adapter.publish_stop()
            self.get_logger().warn('搜索：等待里程计...', throttle_duration_sec=1.0)
            return

        # 二维码可见 → 原地停住，状态机累积 hold 转 APPROACH。
        # 重置检测停留计数，让闪烁的二维码每次消失都重获完整停留窗口。
        if tag_visible and tag_pose is not None:
            self._search_detect_start = 0
            self._adapter.publish_stop()
            return

        # 检测停留：累计 pause_time_sec 的持续不可见，然后转下一步
        pause_time = self._p('search.pause_time_sec')
        if self._search_detect_start == 0:
            self._search_detect_start = now_ns
            self._adapter.publish_stop()
            self.get_logger().info(
                f'搜索：检测停留 (步数={self._search_step})',
                throttle_duration_sec=1.0)
            return

        if (now_ns - self._search_detect_start) * 1e-9 < pause_time:
            self._adapter.publish_stop()
            return

        # 停留期满仍未见到 → 转下一步
        self._search_detect_start = 0
        self._search_step += 1
        step_angle = math.radians(self._p('search.step_angle_deg'))
        angle = step_angle * self._search_direction
        rate = self._p('search.angular_speed')
        if self._executor.start_turn(angle, rate, full=True):
            self._executor.set_odom_ref(
                self._odom_x, self._odom_y, self._odom_yaw)
            self._maneuver_active = True
            self._frozen = True
            self.get_logger().info(
                f'搜索：第{self._search_step}步 原地转 '
                f'{math.degrees(angle):+.1f}° '
                f'(方向={"CCW" if angle > 0 else "CW"})')
        else:
            self._frozen = False
            self._maneuver_active = False
        self._publish_action_cmd(base_type)

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
        self._straight_failed = False

    def _reset_search(self):
        """重置角度步进搜索：计数器、方向、执行器、冻结态。

        进入 SEARCH_TAG 时按最后见到二维码的一侧选初始方向（+lat=左=CCW→+1），
        从未见过则退回 search.search_direction；之后始终同向，12 步转满 360°。
        出 SEARCH_TAG 时也调用，终止可能正在转的搜索步并解冻。
        """
        self._search_step = 0
        self._search_detect_start = 0
        if self._last_seen_lat > 0.0:
            self._search_direction = 1.0
        elif self._last_seen_lat < 0.0:
            self._search_direction = -1.0
        else:
            self._search_direction = 1.0 if self._p('search.search_direction') >= 0 else -1.0
        self._executor.cancel()
        self._frozen = False
        self._maneuver_active = False

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

    # ── Params dict ────────────────────────────────────────────────

    def _build_params_dict(self) -> dict:
        return {
            'timeout_sec': self._p('timeout_sec'),
            'search': {
                'angular_speed': self._p('search.angular_speed'),
                'step_angle_deg': self._p('search.step_angle_deg'),
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
            'retry': {
                'timeout_sec': self._p('retry.timeout_sec'),
            },
            'undock': {
                'timeout_sec': self._p('undock.timeout_sec'),
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
        ok = self._sm.start()
        response.success = ok
        response.message = f'state={self._sm.state_name}' if ok else 'already active'
        self._planner.reset()
        self._executor.cancel()
        self._reset_maneuver()
        return response

    def _on_cancel_docking(self, request, response):
        self._sm.cancel()
        self._planner.reset()
        self._executor.cancel()
        self._reset_maneuver()
        self._adapter.publish_stop()
        response.success = True
        response.message = 'cancelled'
        return response

    def _on_start_undock(self, request, response):
        ok = self._sm.start_undock()
        response.success = ok
        response.message = f'state={self._sm.state_name}' if ok else 'docking active'
        if ok:
            self._planner.reset()
            self._executor.cancel()
            self._reset_maneuver()
            self._undock_phase = 0
        return response

    # ── Action callbacks ───────────────────────────────────────────

    def _execute_dock_cb(self, goal_handle):
        """Action execute callback — blocks until docking completes."""
        from tagdocking.action import Dock

        ok = self._sm.start()
        if not ok:
            goal_handle.abort()
            return Dock.Result(success=False, message='already active')

        self._planner.reset()
        self._executor.cancel()
        self._reset_maneuver()

        feedback = Dock.Feedback()

        while rclpy.ok() and self._sm.is_active:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._sm.cancel()
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
        self._planner.reset()
        self._executor.cancel()
        self._reset_maneuver()
        self._adapter.publish_stop()
        return CancelResponse.ACCEPT

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
    except Exception:
        # 信号处理器(_handle_signal)会调用 rclpy.shutdown() 以便打断 spin,
        # 但这会让正在转的 spin 下一轮 wait_set 初始化抛 RCLError
        # ("context is not valid")。上下文已被有意关闭时属正常退出路径,
        # 吞掉以免 launch 报 process died; 仅真异常(rclpy 仍 ok)才重新抛出。
        if rclpy.ok():
            raise
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
