"""Tag alignment geometry planner — aim-and-go, all chassis unified.

The planner computes the next discrete action from the tag pose relative
to the robot's base_link frame.

Diff-drive (and omni once centred) — aim-and-go / pure pursuit:
  1. If |lat| exceeds the lateral tolerance OR the bearing exceeds the yaw
     tolerance → turn to aim directly at the tag (null the bearing).
  2. Otherwise → drive straight toward the tag.
  Aiming at the tag and driving toward it shrinks lat monotonically to ~0
  at contact.  Driving straight preserves lat, so "will a straight approach
  stay within the lateral tolerance?" is exactly "is |lat| within tolerance?"
  — no diverging turn-away/turn-back oblique manoeuvre.

Omni / quadruped additionally use a direct lateral slide to centre.

Every motion is executed odometry-closed (stop-and-go): the robot stops,
grabs a sharp frame, plans one bounded step, dead-reckons it via odometry,
stops, and re-measures.  The camera is never trusted mid-motion.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .utils import normalize_angle


@dataclass
class ActionPlan:
    """A single discrete action for the ActionExecutor.

    kind: 'yaw' | 'forward' | 'done'
    """
    kind: str = 'done'
    turn_angle: float = 0.0      # rad, for 'yaw' actions
    jog_distance: float = 0.0    # m,   for 'forward' actions
    lateral_distance: float = 0.0  # m, for omni lateral correction (packed in 'forward')


class GeometryPlanner:
    """Tag normal-line alignment planner.

    Stateless aside from the diff-drive multi-phase lateral correction
    sequence (turn→jog→turn).  After each action completes and a fresh
    tag observation is available, call :meth:`plan`.
    """

    def __init__(self,
                 target_distance: float = 0.30,
                 lateral_threshold: float = 0.04,
                 yaw_threshold: float = 0.06,
                 tune_angle: float = 0.0,
                 jog_min: float = 0.05,
                 jog_max: float = 0.50,
                 position_tol: float = 0.02,
                 base_type: str = 'diff_drive'):
        self._target_dist = target_distance
        self._lateral_threshold = lateral_threshold
        self._yaw_threshold = yaw_threshold
        self._tune_angle = tune_angle
        self._jog_min = jog_min
        self._jog_max = jog_max
        self._pos_tol = position_tol
        self._is_omni = base_type in ('omni', 'quadruped')

    # ── Public API ──────────────────────────────────────────────────

    def plan(self, dist: float, lat: float, yaw: float) -> ActionPlan:
        """Return the next action based on the current tag pose.

        Args:
            dist: distance from robot COR to tag along robot's x-axis (m).
            lat:  lateral offset of tag in robot frame (m) — positive = left.
            yaw:  bearing to the tag (rad, CCW+) == atan2(lat, dist).  The
                  system measures only the *direction* to the tag, not the
                  tag's own facing, so yaw and lat are coupled.

        Returns:
            ActionPlan with the next discrete action.

        Strategy — aim-and-go (pure pursuit):
            Driving straight forward preserves ``lat`` (the robot moves along
            its own x-axis).  So "will a forward jog leave us outside the
            ±lateral tolerance at the target?" reduces to "is |lat| already
            outside tolerance?" — a distance-independent, exact test.

            * |lat| within tolerance AND bearing within tolerance → drive
              straight.  A small residual lat/yaw is left as-is; it is inside
              the docking tolerance and needs no correction.
            * otherwise → turn to aim directly at the tag (null the bearing).
              Aiming at the tag and then driving toward it monotonically
              shrinks lat to ~0 at contact — it converges, unlike the old
              turn-away/turn-back oblique scheme which diverged.

            Every turn is executed odometry-closed with undershoot in the
            ActionExecutor, so a turn never overshoots and flings the tag out
            of frame.
        """
        # ── Omni / quadruped: real lateral DOF → direct slide ──
        if self._is_omni and abs(lat) > self._lateral_threshold:
            return self._start_lateral(dist, lat, yaw)

        # ── Diff-drive (and omni once centred): aim-and-go ──
        # Turn only when a straight approach would miss the lateral tolerance
        # (|lat| too big) or we are pointed too far off the tag (|yaw| too big).
        if abs(lat) > self._lateral_threshold or abs(yaw) > self._yaw_threshold:
            return ActionPlan(kind='yaw', turn_angle=yaw)

        # ── Straight approach along the line of sight ──
        remaining = dist - self._target_dist
        if abs(remaining) <= self._pos_tol:
            return ActionPlan(kind='done')

        # Clamp jog distance
        jog = min(abs(remaining), self._jog_max)
        jog = max(jog, self._jog_min) if jog > 0 else 0.0

        if jog < 1e-3:
            return ActionPlan(kind='done')

        jog_signed = jog if remaining > 0 else -jog
        return ActionPlan(kind='forward', jog_distance=jog_signed)

    def reset(self):
        """No-op retained for API compatibility (aim-and-go is stateless).

        The old diff-drive multi-phase lateral sequence held state here; the
        current planner has none, so callers (abort/start/cancel) need do
        nothing, but the method is kept so those call sites stay valid.
        """
        pass

    # ── Turn-drive-turn sequence (normal-line docking) ───────────────

    def plan_sequence(self, dist: float, lat: float, normal: float,
                      yaw_tol: float | None = None) -> list[ActionPlan]:
        """Plan a full blind maneuver onto the tag's normal line.

        Instead of the fragile incremental "turn a little, look, turn again"
        loop — which swings the tag toward the FOV edge and loses it — this
        computes the ENTIRE path from a single good measurement and returns it
        as a list of actions the executor runs back-to-back by odometry, with
        the camera not consulted until the maneuver finishes. The tag leaving
        view mid-maneuver is expected and fine.

        Geometry (all in base_link, REP-103: x fwd, y left, +yaw CCW):

            T = (dist, lat)                     tag position
            n = (cos normal, sin normal)        tag outward normal (already
                                                corrected to point toward the
                                                robot side)
            A = T + d_target · n                docking standoff point on the
                                                normal line, d_target from tag

            turn1 = atan2(A_y, A_x)             rotate to face A
            drive = |A|                         straight to A
            turn2 = (normal + pi) - turn1       at A, face the tag (-n dir)

        After turn1+drive+turn2 the robot sits on the tag's normal line at
        d_target, facing the tag squarely. A final straight-in jog is then just
        the residual (handled by re-measuring — see the node's iterate loop).

        Returns [] when already within tolerance (docked).

        Note: relies on `normal`, which is AprilTag's least-reliable DOF far
        away. The node re-measures and re-plans at each stop (iterative
        refine): as the robot nears and squares up, `normal` stabilizes and the
        sequence converges. Early iterations need only be approximately right.
        """
        yt = yaw_tol if yaw_tol is not None else self._yaw_threshold

        # Standoff point A on the normal line.
        n_x, n_y = math.cos(normal), math.sin(normal)
        a_x = dist + self._target_dist * n_x
        a_y = lat + self._target_dist * n_y

        # Are we already docked? On the normal line at d_target, facing tag.
        # Residual position error = distance from robot origin to A.
        pos_err = math.hypot(a_x, a_y)
        # Residual heading error: direction robot must face to look at tag.
        bearing = math.atan2(lat, dist)
        # "Squareness": how far the current line-of-sight is off the tag normal.
        # When square-on, bearing == normal + pi (robot faces along -n).
        square_err = abs(normalize_angle((normal + math.pi) - bearing))

        if pos_err <= self._pos_tol and square_err <= yt:
            return [ActionPlan(kind='done')]

        turn1 = math.atan2(a_y, a_x)
        drive = pos_err

        # Cap the blind straight leg. A very long dead-reckoned drive accumulates
        # odometry error and (at low frame rate) keeps the tag out of view too
        # long, so we advance at most jog_max per iteration and re-measure.
        clamped = drive > self._jog_max
        if clamped:
            drive = self._jog_max

        # Final turn — CRUCIAL for a narrow FOV. Every blind maneuver must END
        # with the robot facing the tag, or it can never re-acquire it and is
        # declared lost. Two cases:
        #   • reached A (not clamped): face the tag SQUARE-ON along -normal, so
        #     the next straight-in jog stays on the normal line.
        #   • clamped (stopped short of A): we are NOT on the normal line yet, so
        #     square-on would point away from the tag. Instead re-aim straight AT
        #     the tag from the position we actually reach, keeping it centred in
        #     frame for the next measurement. Iterative refine converges as the
        #     robot nears and the clamp stops biting.
        if clamped:
            # Position reached after turn1 + drive (robot heading == turn1).
            p_x = drive * math.cos(turn1)
            p_y = drive * math.sin(turn1)
            # Direction to the tag from there, in the ORIGINAL frame …
            phi = math.atan2(lat - p_y, dist - p_x)
            # … expressed relative to the robot's post-drive heading (turn1).
            turn2 = normalize_angle(phi - turn1)
        else:
            turn2 = normalize_angle((normal + math.pi) - turn1)

        seq: list[ActionPlan] = []
        if abs(turn1) > 1e-3:
            seq.append(ActionPlan(kind='yaw', turn_angle=turn1))
        if drive > self._jog_min * 0.5:
            seq.append(ActionPlan(kind='forward', jog_distance=drive))
        if abs(turn2) > 1e-3:
            seq.append(ActionPlan(kind='yaw', turn_angle=turn2))

        # If everything rounded away but we weren't "done", nudge straight so we
        # never return an empty non-done list (which would stall the loop).
        if not seq:
            return [ActionPlan(kind='done')]
        return seq

    # ── Straight-line final approach (two-phase docking phase 2) ────

    def plan_straight(self, dist: float) -> list[ActionPlan]:
        """Plan a pure straight-line forward jog — no angle adjustment.

        Two-phase docking phase 2: the caller has already confirmed the robot
        is within ``start_distance`` of the tag and the heading is square-on
        within the dedicated yaw threshold. Drive straight forward along the
        current heading until within ``position_tol`` of ``target_distance``.

        Lateral and yaw errors are intentionally NOT corrected — the robot
        drives straight regardless, like parking into a garage. Any lateral
        offset present at the phase-2 handoff is preserved to the final pose.

        Returns ``[done]`` when within position tolerance of target_distance.
        """
        residual = dist - self._target_dist
        if abs(residual) <= self._pos_tol:
            return [ActionPlan(kind='done')]
        # Clamp to jog_max — iterative stop-and-go, re-measure after each step
        # (preserves re-measure safety and narrow-FOV tag retention).
        drive = min(residual, self._jog_max)
        if drive <= self._jog_min * 0.5:
            return [ActionPlan(kind='done')]
        return [ActionPlan(kind='forward', jog_distance=drive)]

    # ── Lateral correction helpers ──────────────────────────────────

    def _start_lateral(self, dist: float, lat: float, yaw: float) -> ActionPlan:
        """Omni / quadruped only: direct lateral slide perpendicular to the
        line of sight.  Positive lat → tag is left → robot moves left
        (positive lateral) to centre under the tag.  Diff-drive never calls
        this — it corrects lateral error by aiming at the tag (see plan()).
        """
        return ActionPlan(kind='forward',
                          jog_distance=abs(lat),
                          lateral_distance=lat)
