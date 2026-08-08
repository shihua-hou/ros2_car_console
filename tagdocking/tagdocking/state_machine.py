"""Docking state machine — full lifecycle management.

States:
    IDLE         — waiting for start command
    NAVIGATING   — Nav2 pre-dock navigation (optional, skipped if nav disabled)
    SEARCH_TAG   — rotating/pausing to find the AprilTag
    ALIGN        — (legacy, skipped — absorbed into APPROACH stop-and-go)
    APPROACH     — stop-and-go: lateral→yaw→forward via geometry planner
    FINAL_SERVO  — stability confirmation (tag pose stable + velocity zero)
    DOCKED       — success: position + yaw within tolerance, robot stable

Error terminal states:
    TAG_LOST     — tag not visible for > timeout
    TIMEOUT      — overall docking timeout exceeded
    MOTION_FAILED— commanded motion not reflected in odometry
    CANCELLED    — user cancelled the docking
"""

import enum
import math
import time
from .utils import normalize_angle


class DockingState(enum.IntEnum):
    IDLE = 0
    NAVIGATING = 1
    SEARCH_TAG = 2
    ALIGN = 3
    APPROACH = 4
    FINAL_SERVO = 5
    DOCKED = 6
    # Error states
    TAG_LOST = 10
    TIMEOUT = 11
    MOTION_FAILED = 12
    CANCELLED = 13


# States that are considered "active" (not terminal)
_ACTIVE_STATES = {
    DockingState.NAVIGATING,
    DockingState.SEARCH_TAG,
    DockingState.ALIGN,
    DockingState.APPROACH,
    DockingState.FINAL_SERVO,
}

# Terminal success state
_SUCCESS_STATE = DockingState.DOCKED

# Terminal error states
_ERROR_STATES = {
    DockingState.TAG_LOST,
    DockingState.TIMEOUT,
    DockingState.MOTION_FAILED,
    DockingState.CANCELLED,
}


class DockingStateMachine:
    """Manages state transitions, timeouts, and docking logic.

    The state machine is EVALUATED (transition decisions) in the control loop
    but ACTUATED (velocity commands) by the main node based on current state.

    Usage:
        sm = DockingStateMachine(params)
        sm.start(enable_navigation=False)
        # In control loop:
        sm.evaluate(tag_pose, tag_visible, odom_data, now_ns)
        state = sm.state
    """

    def __init__(self, node):
        self._node = node
        self._state = DockingState.IDLE
        self._state_start_ns = 0
        self._docking_start_ns = 0

        # Per-state sub-phase tracking
        self._search_phase = 'rotate'     # 'rotate' | 'pause'
        self._search_phase_start_ns = 0
        self._search_tag_hold_start_ns = 0
        self._align_hold_count = 0
        self._tag_lost_count = 0

        # Final servo stability
        self._stable_since_ns = 0
        self._last_velocity = (0.0, 0.0, 0.0)

    # ── Properties ──────────────────────────────────────────────────

    @property
    def state(self) -> DockingState:
        return self._state

    @property
    def state_name(self) -> str:
        return self._state.name

    @property
    def is_active(self) -> bool:
        return self._state in _ACTIVE_STATES

    @property
    def is_terminal(self) -> bool:
        return self._state in _ERROR_STATES or self._state == _SUCCESS_STATE

    @property
    def is_success(self) -> bool:
        return self._state == _SUCCESS_STATE

    @property
    def is_error(self) -> bool:
        return self._state in _ERROR_STATES

    def elapsed_ns(self, now_ns: int) -> int:
        """Nanoseconds since docking started."""
        if self._docking_start_ns == 0:
            return 0
        return now_ns - self._docking_start_ns

    def state_elapsed_ns(self, now_ns: int) -> int:
        """Nanoseconds since current state was entered."""
        if self._state_start_ns == 0:
            return 0
        return now_ns - self._state_start_ns

    # ── Actions ─────────────────────────────────────────────────────

    def start(self, enable_navigation: bool):
        """Initiate docking. Transitions to NAVIGATING or SEARCH_TAG."""
        if self._state not in (DockingState.IDLE, DockingState.DOCKED,
                               *[s for s in _ERROR_STATES]):
            self._node.get_logger().warn(f'无法启动：当前状态={self._state.name}')
            return False

        self._transition_to(
            DockingState.NAVIGATING if enable_navigation else DockingState.SEARCH_TAG)
        if self._docking_start_ns == 0:
            self._docking_start_ns = self._state_start_ns
        return True

    def cancel(self):
        """User cancel — transition to CANCELLED."""
        self._transition_to(DockingState.CANCELLED)

    def reset(self):
        """Full reset to IDLE."""
        self._state = DockingState.IDLE
        self._state_start_ns = 0
        self._docking_start_ns = 0
        self._search_phase = 'rotate'
        self._search_phase_start_ns = 0
        self._search_tag_hold_start_ns = 0
        self._align_hold_count = 0
        self._tag_lost_count = 0
        self._stable_since_ns = 0

    # ── State machine evaluation ────────────────────────────────────

    def evaluate(self,
                 tag_pose,          # TagPose or None
                 tag_visible: bool,
                 odom_x: float,
                 odom_y: float,
                 odom_yaw: float,
                 cmd_vx: float,
                 cmd_vy: float,
                 cmd_wz: float,
                 motion_stalled: bool,
                 nav_done: bool,
                 nav_success: bool,
                 now_ns: int,
                 params: dict,
                 maneuver_active: bool = False):
        """Evaluate state transitions. Call at control loop rate.

        Args:
            tag_pose: Latest valid TagPose from pose buffer, or None.
            tag_visible: True if tag is fresh (within tag_fresh_timeout_sec).
            odom_*: Current odometry.
            cmd_*: Current velocity command being sent.
            motion_stalled: True if motion stalled (handled by action_executor now).
            nav_done: True if Nav2 navigation completed.
            nav_success: True if Nav2 succeeded.
            now_ns: Current ROS time in nanoseconds.
            params: Dict of all relevant parameters (see _get_params_keys).
            maneuver_active: True while a blind turn-drive-turn maneuver is
                executing. The tag legitimately leaves view during the blind
                motion, so tag-loss must NOT be counted while this is True.

        Returns:
            DockingState — the current (possibly new) state.
        """
        state = self._state

        # ── Global timeout ────────────────────────────────────────
        if state in _ACTIVE_STATES:
            overall_sec = self.elapsed_ns(now_ns) * 1e-9
            if overall_sec > params.get('timeout_sec', 120.0):
                self._node.get_logger().error(
                    f'整体超时（{overall_sec:.0f}s）')
                self._transition_to(DockingState.TIMEOUT)
                return self._state

            # Tag lost during a vision state → try to re-acquire rather than
            # dying. A single dropped frame at 6 fps (or blur right after a
            # turn) must NOT terminate docking. After ~1 s of continuous loss
            # we fall back to SEARCH_TAG to re-lock; only if SEARCH_TAG itself
            # then times out do we reach the terminal TAG_LOST. Losing the tag
            # while already searching stays terminal.
            if state in (DockingState.ALIGN, DockingState.APPROACH,
                         DockingState.FINAL_SERVO):
                # A blind turn-drive-turn maneuver expects the tag to leave view
                # during the motion — do NOT accumulate loss while it runs, and
                # reset the counter so a maneuver that ends with the tag briefly
                # out of frame gets a full grace window afterward.
                if maneuver_active:
                    self._tag_lost_count = 0
                elif not tag_visible:
                    self._tag_lost_count += 1
                else:
                    self._tag_lost_count = 0
                tag_lost_limit = int(1.0 / 0.05)  # ~1s at 20Hz = 20 cycles
                if self._tag_lost_count > tag_lost_limit:
                    self._node.get_logger().warn(
                        '接近过程中二维码丢失超过 1s → SEARCH_TAG 重新锁定')
                    self._tag_lost_count = 0
                    self._transition_to(DockingState.SEARCH_TAG)
                    return self._state

            # Motion failure check
            if motion_stalled:
                self._node.get_logger().error('运动卡死 — MOTION_FAILED')
                self._transition_to(DockingState.MOTION_FAILED)
                return self._state

        # ── Per-state evaluation ─────────────────────────────────
        if state == DockingState.IDLE:
            pass  # wait for start command

        elif state == DockingState.NAVIGATING:
            nav_timeout = params.get('navigation', {}).get('nav2_timeout_sec', 60.0)
            elapsed = self.state_elapsed_ns(now_ns) * 1e-9
            if nav_done:
                if nav_success:
                    self._node.get_logger().info('Nav2 完成 → SEARCH_TAG')
                    self._transition_to(DockingState.SEARCH_TAG)
                else:
                    self._node.get_logger().error('Nav2 失败')
                    self._transition_to(DockingState.TIMEOUT)
            elif elapsed > nav_timeout:
                self._node.get_logger().error(f'Nav2 超时（{nav_timeout:.0f}s）')
                self._transition_to(DockingState.TIMEOUT)

        elif state == DockingState.SEARCH_TAG:
            self._eval_search(tag_visible, tag_pose, now_ns, params)

        elif state == DockingState.ALIGN:
            self._eval_align(tag_visible, tag_pose, now_ns, params)

        elif state == DockingState.APPROACH:
            self._eval_approach(tag_visible, tag_pose, now_ns, params)

        elif state == DockingState.FINAL_SERVO:
            self._eval_final_servo(tag_visible, tag_pose, now_ns,
                                   cmd_vx, cmd_vy, cmd_wz, params)

        elif state in _ERROR_STATES or state == _SUCCESS_STATE:
            pass  # terminal states

        return self._state

    # ── Per-state evaluators ───────────────────────────────────────

    def _eval_search(self, tag_visible, tag_pose, now_ns, params):
        """SEARCH_TAG: rotate-pause scan pattern."""
        search_cfg = params.get('search', {})
        rotate_time = search_cfg.get('rotate_time_sec', 0.8)
        pause_time = search_cfg.get('pause_time_sec', 1.5)
        hold_time = search_cfg.get('hold_time_sec', 0.5)
        angular_speed = search_cfg.get('angular_speed', 0.3)
        search_dir = search_cfg.get('search_direction', 1)
        search_timeout = search_cfg.get('timeout_sec', 60.0)

        # Per-state timeout
        if self.state_elapsed_ns(now_ns) * 1e-9 > search_timeout:
            self._node.get_logger().error('搜索超时 — 未找到二维码')
            self._transition_to(DockingState.TIMEOUT)
            return

        # Phase switching
        elapsed_phase = (now_ns - self._search_phase_start_ns) * 1e-9

        if self._search_phase == 'rotate':
            if elapsed_phase > rotate_time:
                self._search_phase = 'pause'
                self._search_phase_start_ns = now_ns
                self._node.get_logger().debug('Search: pause for detection',
                                              throttle_duration_sec=1.0)
                return
        elif self._search_phase == 'pause':
            if elapsed_phase > pause_time:
                self._search_phase = 'rotate'
                self._search_phase_start_ns = now_ns
                self._node.get_logger().debug('Search: rotate',
                                              throttle_duration_sec=1.0)
                return

        # Tag lock during pause
        if tag_visible and self._search_phase == 'pause':
            if self._search_tag_hold_start_ns == 0:
                self._search_tag_hold_start_ns = now_ns
            else:
                hold_elapsed = (now_ns - self._search_tag_hold_start_ns) * 1e-9
                if hold_elapsed >= hold_time:
                    self._node.get_logger().info(
                        f'二维码已锁定：距离={tag_pose.dist:.3f}m → APPROACH')
                    self._transition_to(DockingState.APPROACH)
                    return
        else:
            self._search_tag_hold_start_ns = 0

    def _eval_align(self, tag_visible, tag_pose, now_ns, params):
        """ALIGN: coarse yaw alignment with tag before approach."""
        if not tag_visible or tag_pose is None:
            return

        tolerance_cfg = params.get('tolerance', {})
        yaw_tol = math.radians(tolerance_cfg.get('yaw_deg', 5.0))

        if abs(tag_pose.yaw) < yaw_tol:
            self._align_hold_count += 1
            if self._align_hold_count >= 5:  # ~250ms at 20Hz
                self._node.get_logger().info('航向已对准 → APPROACH')
                self._transition_to(DockingState.APPROACH)
                return
        else:
            self._align_hold_count = 0

        # Timeout
        align_timeout = params.get('align_timeout_sec', 15.0)
        if self.state_elapsed_ns(now_ns) * 1e-9 > align_timeout:
            self._node.get_logger().warn('对准超时，继续进入 APPROACH')
            self._transition_to(DockingState.APPROACH)

    def _eval_approach(self, tag_visible, tag_pose, now_ns, params):
        """APPROACH: drive toward tag until within final servo distance."""
        if not tag_visible or tag_pose is None:
            return

        tolerance_cfg = params.get('tolerance', {})
        pos_tol = tolerance_cfg.get('position_m', 0.03)
        dock_target = params.get('dock_target', {})
        target_distance = dock_target.get('distance', 0.30)
        approach_timeout = params.get('approach_timeout_sec', 60.0)

        # Check if close enough for final servo.
        # 用 2x 容差(而非 3x)收紧进入阈值: 进入 FINAL_SERVO 时距目标 <=6cm,
        # 避免 FINAL_SERVO 期间还要大幅移动导致近距离 tag 检测抖动丢失。
        distance = math.hypot(tag_pose.dist - target_distance, tag_pose.lat)
        if distance < pos_tol * 2:  # 2x tolerance → transition to FINAL_SERVO
            self._node.get_logger().info(
                f'距离足够近（位置误差={distance:.3f}m）→ FINAL_SERVO')
            self._transition_to(DockingState.FINAL_SERVO)
            return

        # Timeout
        if self.state_elapsed_ns(now_ns) * 1e-9 > approach_timeout:
            self._node.get_logger().error('接近超时')
            self._transition_to(DockingState.TIMEOUT)

    def _eval_final_servo(self, tag_visible, tag_pose, now_ns,
                          cmd_vx, cmd_vy, cmd_wz, params):
        """FINAL_SERVO: slow precise approach + multi-frame confirmation."""
        tolerance_cfg = params.get('tolerance', {})
        pos_tol = tolerance_cfg.get('position_m', 0.03)
        yaw_tol_deg = tolerance_cfg.get('yaw_deg', 3.0)
        yaw_tol = math.radians(yaw_tol_deg)
        stable_time = tolerance_cfg.get('stable_time_sec', 1.0)
        stable_ns = int(stable_time * 1e9)

        dock_target = params.get('dock_target', {})
        target_distance = dock_target.get('distance', 0.30)
        lateral_offset = dock_target.get('lateral_offset', 0.0)
        yaw_offset_rad = math.radians(dock_target.get('yaw_offset_deg', 0.0))

        if tag_visible and tag_pose is not None:
            error_dist = abs(tag_pose.dist - target_distance)
            error_lat = abs(tag_pose.lat - lateral_offset)
            error_yaw = abs(normalize_angle(tag_pose.yaw - yaw_offset_rad))

            # Check dock success conditions
            at_position = error_dist < pos_tol and error_lat < pos_tol
            at_yaw = error_yaw < yaw_tol

            # Check robot stability (velocity near zero)
            vel_mag = abs(cmd_vx) + abs(cmd_vy) + abs(cmd_wz)
            is_stable = vel_mag < 0.01

            if at_position and at_yaw and is_stable:
                if self._stable_since_ns == 0:
                    self._stable_since_ns = now_ns
                elif (now_ns - self._stable_since_ns) >= stable_ns:
                    self._node.get_logger().info(
                        f'停靠成功：距离误差={error_dist:.3f}m '
                        f'横向误差={error_lat:.3f}m 航向误差={error_yaw:.3f}rad')
                    self._transition_to(DockingState.DOCKED)
                    return
            else:
                self._stable_since_ns = 0

        # Safety: minimum distance
        safety_cfg = params.get('safety', {})
        min_distance = safety_cfg.get('minimum_distance_m', 0.15)
        if tag_pose is not None and tag_pose.dist < min_distance:
            self._node.get_logger().warn(
                f'已达最小安全距离（{tag_pose.dist:.3f}m < {min_distance}m）→ DOCKED')
            self._transition_to(DockingState.DOCKED)
            return

        # Timeout
        final_timeout = params.get('final_servo_timeout_sec', 30.0)
        if self.state_elapsed_ns(now_ns) * 1e-9 > final_timeout:
            self._node.get_logger().error('FINAL_SERVO 超时')
            # Best-effort: accept current pose if tag is visible and close
            if tag_visible and tag_pose is not None:
                error_dist = abs(tag_pose.dist - target_distance)
                if error_dist < pos_tol * 3:
                    self._node.get_logger().warn(
                        f'超时但接受当前位姿（误差={error_dist:.3f}m）')
                    self._transition_to(DockingState.DOCKED)
                    return
            self._transition_to(DockingState.TIMEOUT)

    # ── Internal ────────────────────────────────────────────────────

    def _transition_to(self, new_state: DockingState):
        if self._state == new_state:
            return
        old = self._state
        self._state = new_state
        self._state_start_ns = self._node.get_clock().now().nanoseconds

        # Reset per-state tracking
        self._search_phase = 'rotate'
        self._search_phase_start_ns = self._state_start_ns
        self._search_tag_hold_start_ns = 0
        self._align_hold_count = 0
        self._tag_lost_count = 0
        self._stable_since_ns = 0

        self._node.get_logger().info(f'状态：{old.name} → {new_state.name}')
