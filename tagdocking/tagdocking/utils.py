"""Core utility functions: angle math, quaternion conversions, pose types.

Conventions follow ROS REP-103: +x=forward, +y=left, +yaw=CCW (left turn).
"""

import math
from collections import namedtuple


def normalize_angle(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quat(q) -> float:
    """Extract yaw (rotation around Z) from a quaternion.

    Accepts any object with .x .y .z .w attributes
    (ROS quaternion message or geometry_msgs Quaternion).
    """
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_from_yaw(yaw: float):
    """Return (x, y, z, w) tuple for a rotation of `yaw` around Z."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def tag_normal_angle(q, dist: float, lat: float) -> float:
    """Direction of the tag's outward normal in the base_link ground plane.

    apriltag_ros runs with ``z_up:=false``, so the tag frame's z-axis is the
    outward normal of the tag face. ``q`` is the rotation of the base_link→tag
    transform (object with .x .y .z .w). Rotating the tag-frame z unit vector
    (0,0,1) by ``q`` gives the tag normal expressed in base_link; its third
    column of the rotation matrix is::

        n_x = 2 (x z + w y)
        n_y = 2 (y z - w x)

    We only need the ground-plane projection (n_x, n_y). The angle atan2(n_y,
    n_x) is the direction the normal points in base_link.

    Planar-tag solvePnP can FLIP the normal (point it into the tag instead of
    out toward the camera), especially at small apparent size / oblique angle.
    We self-correct: the true outward normal must point from the tag back
    toward the robot, i.e. have a positive component along (tag→robot) =
    (-dist, -lat). If it doesn't, flip it. This makes the result robust to the
    flip ambiguity without trusting the raw sign.

    Returns the normal angle (rad). With the robot square-on to a facing tag
    (lat≈0) the normal points along base_link -x, so the angle ≈ ±pi.
    """
    n_x = 2.0 * (q.x * q.z + q.w * q.y)
    n_y = 2.0 * (q.y * q.z - q.w * q.x)
    # Force the normal to point toward the robot (tag→robot = (-dist, -lat)).
    if n_x * (-dist) + n_y * (-lat) < 0.0:
        n_x, n_y = -n_x, -n_y
    return math.atan2(n_y, n_x)


# ── Pose types ───────────────────────────────────────────────────

TagPose = namedtuple('TagPose', ['dist', 'lat', 'yaw', 'normal', 'stamp_ns'])
"""Tag pose in robot base_link frame (REP-103).

dist     : float — forward distance (m), positive = tag ahead
lat      : float — lateral offset (m), positive = tag to the left
yaw      : float — bearing to the tag (rad) = atan2(lat, dist). ≈0 when the
                   robot is pointed straight at the tag.
normal   : float — direction of the tag's OUTWARD normal expressed as an angle
                   in the base_link ground plane (rad). This is the direction
                   the robot must eventually face (reversed) to dock squarely.
                   0 ⇒ tag faces straight back at the robot along -x.
stamp_ns : int   — detection timestamp (nanoseconds, from sensor_msgs/Header)
"""


TagPoseRaw = namedtuple('TagPoseRaw', ['dist', 'lat', 'yaw', 'stamp_ns'])
"""Raw (unfiltered) tag pose — same fields as TagPose."""
