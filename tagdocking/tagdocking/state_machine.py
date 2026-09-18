"""Docking state machine — full lifecycle management.

States:
    IDLE         — waiting for start command
    SEARCH_TAG   — rotating/pausing to find the AprilTag
    ALIGN        — (legacy, skipped — absorbed into APPROACH stop-and-go)
    APPROACH     — stop-and-go: lateral→yaw→forward via geometry planner
    FINAL_SERVO  — stability confirmation (tag pose stable + velocity zero)
    DOCKED       — success: position + yaw within tolerance, robot stable
    UNDOCKING    — pulling out: blind reverse a distance + 180° turn
    UNDOCKED     — success: undock maneuver complete

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
    SEARCH_TAG = 2
    ALIGN = 3
    APPROACH = 4
    FINAL_SERVO = 5
    DOCKED = 6
    RETRYING = 8          # 失败后倒车重试(活动态, 节点驱动盲退后转 SEARCH_TAG)
    UNDOCKING = 9         # 泊出(活动态): 盲退一段距离 + 原地转180°
    # Error states
    TAG_LOST = 10
    TIMEOUT = 11
    MOTION_FAILED = 12
    CANCELLED = 13
    UNDOCKED = 14         # 泊出成功(终态)


# States that are considered "active" (not terminal)
_ACTIVE_STATES = {
    DockingState.SEARCH_TAG,
    DockingState.ALIGN,
    DockingState.APPROACH,
    DockingState.FINAL_SERVO,
    DockingState.RETRYING,
    DockingState.UNDOCKING,
}

# Terminal success states
_SUCCESS_STATES = {DockingState.DOCKED, DockingState.UNDOCKED}

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
        sm.start()
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
        self._search_tag_hold_start_ns = 0
        self._align_hold_count = 0
        self._tag_lost_count = 0

        # Final servo stability
        self._stable_since_ns = 0
        self._last_velocity = (0.0, 0.0, 0.0)

        # 失败重试: 倒车后重新停靠。max_retries 由节点从参数写入。
        self._retry_count = 0
        self._max_retries = 2

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
        return self._state in _ERROR_STATES or self._state in _SUCCESS_STATES

    @property
    def is_success(self) -> bool:
        return self._state in _SUCCESS_STATES

    @property
    def is_error(self) -> bool:
        return self._state in _ERROR_STATES

    @property
    def retry_count(self) -> int:
        return self._retry_count

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

    def start(self):
        """Initiate docking. Transitions straight to SEARCH_TAG.

        Navigation (Nav2 pre-dock) has been removed — an external service is
        expected to bring the robot into tag range before calling start.

        Allowed from any non-active state: IDLE, a success terminal (DOCKED /
        UNDOCKED), or an error terminal. In particular re-docking after an
        undock (UNDOCKED) must be allowed — otherwise a dock→undock→dock cycle
        gets stuck at "无法启动：当前状态=UNDOCKED".
        """
        if self._state not in (DockingState.IDLE, *_SUCCESS_STATES, *_ERROR_STATES):
            self._node.get_logger().warn(f'无法启动：当前状态={self._state.name}')
            return False

        self._retry_count = 0   # 用户主动启动: 重试计数清零
        self._transition_to(DockingState.SEARCH_TAG)
        # 每次用户触发都是一次全新的停泊: 重置整体超时起点。
        # (重试走 retry_search(), 不动 _docking_start_ns, 故同一次停泊内的
        #  多次倒车重试仍累计在同一超时窗口内。)
        self._docking_start_ns = self._state_start_ns
        return True

    def start_undock(self):
        """Initiate undocking (pull out): blind reverse + 180° turn.

        Allowed from any non-active state (IDLE / DOCKED / error terminals) —
        typically called after DOCKED. Rejected if a docking sequence is in
        progress. The node drives the blind two-step maneuver; on completion
        it calls finish_undock() → UNDOCKED.
        """
        if self._state in _ACTIVE_STATES:
            self._node.get_logger().warn(
                f'无法泊出：停泊进行中({self._state.name})')
            return False
        self._transition_to(DockingState.UNDOCKING)
        # 泊出是一次独立操作: 重置整体超时起点, 否则 UNDOCKING 也受全局超时
        # (_ACTIVE_STATES) 管辖, 而 _docking_start_ns 还停留在上次停泊的 t0,
        # 隔一阵再触发泊出会立刻 elapsed>timeout_sec 落 TIMEOUT。
        self._docking_start_ns = self._state_start_ns
        return True

    def finish_undock(self):
        """Node calls this when the undock maneuver completed → UNDOCKED."""
        self._transition_to(DockingState.UNDOCKED)

    def cancel(self):
        """User cancel — transition to CANCELLED."""
        self._transition_to(DockingState.CANCELLED)

    def fail(self, reason: str = ''):
        """节点主动报失败 — 可重试的失败(如直行阶段对准过差)。

        若还有重试次数, 转 RETRYING: 节点盲退一段距离后由 retry_search() 转
        SEARCH_TAG 重新锁定靠近。退满 max_retries 次仍失败才落 MOTION_FAILED 终态。
        AprilTag 近场位姿(尤其降采样后的小码)单帧噪声大, 直行入口一次方位超限
        往往是抖动而非真没对准 —— 后退重锁给一次重新靠近的机会。
        与 cancel()(用户主动取消)区分: cancel() 不重试, 直接终态。
        """
        if reason:
            self._node.get_logger().error(f'导航失败：{reason}')
        if self._retry_count < self._max_retries:
            self._retry_count += 1
            self._node.get_logger().warn(
                f'停靠失败，自动重试({self._retry_count}/{self._max_retries}) '
                f'→ 倒车后重新锁定')
            self._transition_to(DockingState.RETRYING)
        else:
            self._node.get_logger().error(
                f'停靠失败且已达最大重试次数({self._max_retries})')
            self._transition_to(DockingState.MOTION_FAILED)

    def retry_search(self):
        """RETRYING 倒车到位后调用: 转 SEARCH_TAG 重新锁定。

        保留 _docking_start_ns(整体超时累计), 只重置搜索子相位。
        """
        self._search_tag_hold_start_ns = 0
        self._tag_lost_count = 0
        self._transition_to(DockingState.SEARCH_TAG)

    def reset(self):
        """Full reset to IDLE."""
        self._state = DockingState.IDLE
        self._state_start_ns = 0
        self._docking_start_ns = 0
        self._search_tag_hold_start_ns = 0
        self._align_hold_count = 0
        self._tag_lost_count = 0
        self._stable_since_ns = 0
        self._retry_count = 0

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

        elif state == DockingState.SEARCH_TAG:
            self._eval_search(tag_visible, tag_pose, now_ns, params)

        elif state == DockingState.ALIGN:
            self._eval_align(tag_visible, tag_pose, now_ns, params)

        elif state == DockingState.APPROACH:
            self._eval_approach(tag_visible, tag_pose, now_ns, params)

        elif state == DockingState.FINAL_SERVO:
            self._eval_final_servo(tag_visible, tag_pose, now_ns,
                                   cmd_vx, cmd_vy, cmd_wz, params)

        elif state == DockingState.RETRYING:
            # 倒车超时保护: 里程计不走(底盘卡死/失联)会卡在 RETRYING,
            # 直接落 MOTION_FAILED 终态(不再重试, 防止无限倒车)。
            retry_timeout = params.get('retry', {}).get('timeout_sec', 15.0)
            if self.state_elapsed_ns(now_ns) * 1e-9 > retry_timeout:
                self._node.get_logger().error('重试倒车超时 — MOTION_FAILED')
                self._transition_to(DockingState.MOTION_FAILED)

        elif state == DockingState.UNDOCKING:
            # 泊出超时保护: 里程计不走(底盘卡死/失联)会卡在 UNDOCKING,
            # 直接落 MOTION_FAILED 终态。
            undock_timeout = params.get('undock', {}).get('timeout_sec', 30.0)
            if self.state_elapsed_ns(now_ns) * 1e-9 > undock_timeout:
                self._node.get_logger().error('泊出超时 — MOTION_FAILED')
                self._transition_to(DockingState.MOTION_FAILED)

        elif state in _ERROR_STATES or state in _SUCCESS_STATES:
            pass  # terminal states

        return self._state

    # ── Per-state evaluators ───────────────────────────────────────

    def _eval_search(self, tag_visible, tag_pose, now_ns, params):
        """SEARCH_TAG: 角度步进扫描；节点负责转/检循环，这里只判超时和锁定。

        节点的 _run_search 负责转固定角度(里程计闭环)→停稳→检测的循环。
        转动期节点冻结检测(_frozen=True)，_on_detections 直接返回，故
        tag_visible 在转动期恒为 False，这里的 tag-lock 不会在转动中误触发。
        """
        search_cfg = params.get('search', {})
        hold_time = search_cfg.get('hold_time_sec', 0.5)
        search_timeout = search_cfg.get('timeout_sec', 60.0)

        # Per-state timeout
        if self.state_elapsed_ns(now_ns) * 1e-9 > search_timeout:
            self._node.get_logger().error('搜索超时 — 未找到二维码')
            self._transition_to(DockingState.TIMEOUT)
            return

        # Tag lock: 检测期持续可见 hold_time → APPROACH
        if tag_visible and tag_pose is not None:
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
        self._search_tag_hold_start_ns = 0
        self._align_hold_count = 0
        self._tag_lost_count = 0
        self._stable_since_ns = 0

        self._node.get_logger().info(f'状态：{old.name} → {new_state.name}')
