"""Odom-closed-loop action executor — jog and turn with odometry dead reckoning.

Each motion is executed at a constant rate, and completion is determined
by odometry displacement (not timed). This eliminates overshoot and
undershoot from acceleration profiles — the robot moves exactly the
commanded distance or angle, verified by wheel odometry.

During motion, the camera is NOT consulted (image blur from movement
degrades detection quality). Visual checks are only used for early-stop
safety (e.g., tag distance already at target).

Usage:
    exec = ActionExecutor()
    exec.start_jog(0.5, rate=0.08)       # jog 0.5m forward at 0.08 m/s
    exec.start_turn(0.3, rate=0.3)       # turn 0.3 rad at 0.3 rad/s
    # In control loop:
    done = exec.update(odom_x, odom_y, odom_yaw, tag_visible, raw_dist, raw_lat,
                       bearing_fn, theta_bounds_fn, now_ns)
    if done:
        plan_next_step()
"""

import math
from .utils import normalize_angle


class ActionExecutor:
    """Odom-closed-loop action: jog (straight) and turn (rotate in place)."""

    def __init__(self,
                 turn_settle_sec: float = 0.5,
                 turn_undershoot: float = 0.75,
                 max_turn_step: float = 0.3,
                 small_turn_rad: float = 0.1,
                 final_approach_distance: float = 1.0,
                 yaw_threshold: float = 0.05):
        self._turn_settle_ns = int(turn_settle_sec * 1e9)
        self._turn_undershoot = turn_undershoot
        self._max_turn_step = max_turn_step
        self._small_turn_rad = small_turn_rad
        self._final_approach_distance = final_approach_distance
        # Fixed bearing tolerance the PLANNER uses to decide a yaw is needed.
        # The turn's visual early-stop must be at least this tight, otherwise
        # the planner keeps demanding a turn (|bearing| > yaw_threshold) while
        # the executor aborts it immediately against the much looser dynamic
        # theta_bounds — a deadlock at mid range (see _update_turn).
        self._yaw_threshold = yaw_threshold

        # Action state: 'idle' | 'jogging' | 'turning' | 'lateral'
        self._action = 'idle'
        self._action_linear = 0.0
        self._action_angular = 0.0
        self._action_lateral = 0.0
        self._action_target = 0.0

        # Jog tracking
        self._jog_start_x = 0.0
        self._jog_start_y = 0.0
        self._jog_start_yaw = 0.0
        self._jog_start_bearing = 0.0
        self._jog_blind = False

        # Turn tracking
        self._turn_start_yaw = 0.0
        self._turn_blind_cap = None

        # Visual settle after turn
        self._last_stop_ns: int | None = None

        # Counters (diagnostic)
        self.jog_count = 0
        self.turn_count = 0

    # ── Properties ──────────────────────────────────────────────────

    @property
    def is_active(self) -> bool:
        return self._action != 'idle'

    @property
    def action_kind(self) -> str:
        return self._action

    @property
    def linear_cmd(self) -> float:
        return self._action_linear

    @property
    def angular_cmd(self) -> float:
        return self._action_angular

    @property
    def lateral_cmd(self) -> float:
        return self._action_lateral

    # ── Start actions ───────────────────────────────────────────────

    def start_jog(self, distance: float, linear_rate: float,
                  blind: bool = False):
        """Start a straight-line jog of `distance` metres.

        Positive = forward, negative = reverse.
        `linear_rate` is the signed constant speed (m/s).
        `blind` = True disables the visual early-stops (distance-at-target and
        bearing-drift) so the jog runs purely on odometry — required for the
        blind turn-drive-turn maneuver, where the pre-move tag pose is stale and
        the whole leg must be driven to completion regardless of what the
        (frozen) camera reading says.
        """
        if self._action != 'idle':
            return
        self.jog_count += 1
        self._action = 'jogging'
        self._action_linear = linear_rate if distance >= 0 else -abs(linear_rate)
        self._action_angular = 0.0
        self._action_target = abs(distance)
        self._jog_blind = blind

    def start_turn(self, angle: float, angular_rate: float,
                   full: bool = False) -> bool:
        """Start an in-place rotation of `angle` radians.

        Positive = CCW (left turn), negative = CW (right turn).

        `full` = False (default, legacy aim-and-go): apply undershoot and
        max-step clamping — a conservative fraction of the angle, re-measured
        each step.

        `full` = True (turn-drive-turn maneuver): execute the ENTIRE computed
        angle, NO undershoot, NO max-step cap. The turn-drive-turn geometry is
        computed as one atomic path; clamping turn1 here would send the robot
        off along the wrong heading for the full drive leg. Odometry closes the
        loop and the planner re-measures after the whole sequence, so the full
        angle is both wanted and safe.

        Returns True if the action actually started, False if the angle was too
        small to bother (caller should advance to the next queued step).
        """
        if self._action != 'idle':
            return False

        if full:
            damped = angle  # execute the full computed angle
            min_turn = 0.005  # rad (~0.3°) — below this, not worth a move
        else:
            # Undershoot: only turn a fraction of the requested angle to
            # prevent overshoot from chassis inertia
            damped = angle * self._turn_undershoot
            if abs(damped) > self._max_turn_step:
                damped = self._max_turn_step if damped > 0 else -self._max_turn_step
            min_turn = 0.02

        if abs(damped) < min_turn:  # too small to bother
            return False

        self.turn_count += 1
        self._action = 'turning'
        # Slow down for small turns
        rate = abs(angular_rate)
        if abs(damped) < self._small_turn_rad:
            rate *= 0.5
        self._action_angular = rate if damped >= 0 else -rate
        self._action_linear = 0.0
        self._action_target = abs(damped)
        return True

    def start_jog_lateral(self, distance: float, lateral_rate: float):
        """Start a pure lateral move (omni/mecanum only).

        Positive distance = move left, negative = move right.
        Completion is tracked by projecting odometry displacement onto
        the lateral axis at the start of the action.
        """
        if self._action != 'idle':
            return
        self.jog_count += 1
        self._action = 'lateral'
        self._action_lateral = lateral_rate if distance >= 0 else -abs(lateral_rate)
        self._action_linear = 0.0
        self._action_angular = 0.0
        self._action_target = abs(distance)

    # ── Set odometry reference ──────────────────────────────────────

    def set_odom_ref(self, x: float, y: float, yaw: float, bearing: float = 0.0,
                     blind_cap: float | None = None):
        """Record the current odometry as the reference for the active action.

        Must be called ONCE after start_jog/start_turn, when the robot is
        considered to have begun moving from this odometry position.
        """
        if self._action == 'jogging':
            self._jog_start_x = x
            self._jog_start_y = y
            self._jog_start_yaw = yaw
            self._jog_start_bearing = bearing
        elif self._action == 'lateral':
            self._jog_start_x = x
            self._jog_start_y = y
            self._jog_start_yaw = yaw
        elif self._action == 'turning':
            self._turn_start_yaw = yaw
            self._turn_blind_cap = blind_cap

    # ── Update (call at control-loop rate) ──────────────────────────

    def update(self, odom_x: float, odom_y: float, odom_yaw: float,
               tag_visible: bool, raw_dist: float | None,
               bearing_fn, theta_bounds_fn,
               target_distance: float, drift_tol: float,
               now_ns: int) -> bool:
        """Check if the current action has completed via odometry.

        Returns True when the action is done (robot has moved the
        commanded distance/angle, or a visual safety check triggers
        early stop).

        After returning True, the caller should read fresh tag data
        and plan the next step.
        """
        if self._action == 'idle':
            return True

        if self._action == 'jogging':
            return self._update_jog(odom_x, odom_y, odom_yaw,
                                    tag_visible, raw_dist, bearing_fn,
                                    theta_bounds_fn, target_distance, drift_tol)

        if self._action == 'lateral':
            return self._update_lateral(odom_x, odom_y,
                                         tag_visible, raw_dist, target_distance)

        if self._action == 'turning':
            return self._update_turn(odom_yaw, tag_visible, raw_dist,
                                     bearing_fn, theta_bounds_fn)

        return False

    def _update_jog(self, ox, oy, oyaw, tag_visible, raw_dist,
                    bearing_fn, theta_bounds_fn, target_dist, drift_tol) -> bool:
        dx = ox - self._jog_start_x
        dy = oy - self._jog_start_y
        traveled = math.hypot(dx, dy)

        # Blind jog (turn-drive-turn leg): odometry-only, NO visual early-stop.
        # The pre-move tag pose is stale for the whole maneuver and the leg was
        # computed as part of one atomic path — a visual short-circuit here
        # would truncate the drive and strand the robot off the normal line.
        if self._jog_blind:
            if traveled >= self._action_target:
                self._stop()
                return True
            return False

        # Safety: visual distance already at target → early stop
        if tag_visible and raw_dist is not None and raw_dist <= target_dist:
            self._stop()
            return True

        # Safety: bearing drift during jog (chassis may run an arc)
        # If bearing has drifted too far, stop early and re-plan
        if tag_visible and raw_dist is not None:
            current_bearing = bearing_fn()
            drift = abs(normalize_angle(current_bearing - self._jog_start_bearing))
            if drift > 2.0 * theta_bounds_fn():
                self._stop()
                return True

        # Odometry target reached
        if traveled >= self._action_target:
            self._stop()
            return True

        return False

    def _update_lateral(self, ox, oy, tag_visible, raw_dist, target_dist) -> bool:
        """Check lateral odometry displacement against target."""
        dx = ox - self._jog_start_x
        dy = oy - self._jog_start_y
        # Project (dx, dy) onto the lateral axis at start of action.
        # Forward = (cos_yaw, sin_yaw), lateral = (-sin_yaw, cos_yaw).
        cos_yaw = math.cos(self._jog_start_yaw)
        sin_yaw = math.sin(self._jog_start_yaw)
        lateral = -sin_yaw * dx + cos_yaw * dy

        # Safety: visual distance already at target → early stop
        if tag_visible and raw_dist is not None and raw_dist <= target_dist:
            self._stop()
            return True

        if abs(lateral) >= self._action_target:
            self._stop()
            return True

        return False

    def _update_turn(self, odom_yaw, tag_visible, raw_dist,
                     bearing_fn, theta_bounds_fn) -> bool:
        turned = abs(normalize_angle(odom_yaw - self._turn_start_yaw))

        # Determine completion target: use blind cap if tag not visible
        target_now = self._action_target
        if not (tag_visible and raw_dist is not None):
            if self._turn_blind_cap is not None:
                target_now = min(self._turn_blind_cap, self._action_target)

        if turned >= target_now:
            self._stop()
            return True

        # NO visual early-stop for turns.
        #
        # In stop-and-go the camera is not trusted mid-motion: docking_node
        # enforces this with its `_frozen` gate — during a blind maneuver all
        # detections are dropped, so `_raw_*`/bearing_fn() here are ALWAYS the
        # stale pre-turn values, and `tag_visible` may go stale too.
        #
        # A turn is commanded by the planner precisely because that pre-turn
        # bearing/lat is out of tolerance. The lateral trigger fires at a very
        # small bearing (atan2(lateral_threshold, dist) — e.g. 2.3° at 1.25 m),
        # far below yaw_threshold (10°). Gating turn completion on that same
        # stale bearing therefore aborted the turn at ZERO rotation on the first
        # tick, the robot never moved, the re-measured pose was identical, and
        # the planner re-demanded the same turn forever — an infinite no-progress
        # loop.
        #
        # Turn completion is governed by ODOMETRY alone (turned >= target_now).
        # Undershoot (turn_undershoot) and max_turn_step already prevent
        # overshoot, and the planner re-measures + re-plans after every step, so
        # there is no over-rotation risk without the visual gate.
        return False

    # ── Visual settle after turn ────────────────────────────────────

    def wait_visual_settle(self, tag_visible: bool, now_ns: int) -> bool:
        """Return True if we should wait for visual to refresh after a turn.

        After a turn completes, the camera image may be blurry. We force
        a short wait to let fresh, sharp frames arrive before reading the
        tag pose.
        """
        if self._last_stop_ns is None:
            return False
        elapsed_ns = now_ns - self._last_stop_ns
        return elapsed_ns < self._turn_settle_ns

    # ── Stop ────────────────────────────────────────────────────────

    def _stop(self):
        was_turning = (self._action == 'turning')
        self._action = 'idle'
        self._action_linear = 0.0
        self._action_angular = 0.0
        self._jog_blind = False
        if was_turning:
            self._last_stop_ns = None  # caller sets this via mark_stop_time

    def mark_stop_time(self, now_ns: int):
        """Record when the action stopped, for visual settle timing."""
        self._last_stop_ns = now_ns

    def cancel(self):
        """Abort current action immediately."""
        self._action = 'idle'
        self._action_linear = 0.0
        self._action_angular = 0.0
        self._action_lateral = 0.0
        self._jog_blind = False
        self._last_stop_ns = None
