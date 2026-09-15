#!/usr/bin/env python3
"""
CSC477 Tutorial 2, Exercise 2: coordinate frames and 2D rigid transforms (no ROS).

Every sensor reading in ROS is stamped with a frame_id. A lidar publishes points
in its own "laser" frame; the robot body is "base_link"; odometry tracks
base_link in "odom". Before you can fuse, map, or visualize anything you must
move points between these frames. tf2 does this for you at runtime, but the
math is a 3x3 matrix, and you should be able to write it yourself.

We use homogeneous coordinates in 2D:

    T = [ cos(yaw)  -sin(yaw)  x ]        p_parent = T_parent_child @ p_child
        [ sin(yaw)   cos(yaw)  y ]
        [    0          0      1 ]

Notation: T_a_b is "the pose of frame b expressed in frame a", and it maps points
from b-coordinates to a-coordinates. Chaining: T_odom_laser = T_odom_base @ T_base_laser.

Fill in the TODOs, then run

    python3 ex2_frames_and_transforms.py

Only numpy is required; matplotlib is optional for the plot at the end.
"""

from __future__ import annotations

import math
from typing import Tuple

import numpy as np


# --------------------------------------------------------------------------- #
# TODO 1: build a transform from (x, y, yaw)                                   #
# --------------------------------------------------------------------------- #
def make_transform(x: float, y: float, yaw: float) -> np.ndarray:
    """Return the 3x3 homogeneous transform for a translation (x, y) and rotation yaw [rad]."""
    # TODO: replace the line below.
    raise NotImplementedError("make_transform")


# --------------------------------------------------------------------------- #
# TODO 2: recover (x, y, yaw) from a transform                                 #
# --------------------------------------------------------------------------- #
def pose_from_transform(T: np.ndarray) -> Tuple[float, float, float]:
    """
    Inverse of make_transform. Return (x, y, yaw) as Python floats with
    yaw in (-pi, pi]. Hint: math.atan2 on the rotation block; do NOT use acos.
    """
    # TODO: replace the line below.
    raise NotImplementedError("pose_from_transform")


# --------------------------------------------------------------------------- #
# TODO 3: apply a transform to many points at once                             #
# --------------------------------------------------------------------------- #
def transform_points(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """
    pts is an (M, 2) array of points in the child frame. Return an (M, 2) array
    of the same points in the parent frame. No Python loops: append a column of
    ones, multiply, drop the last coordinate.
    """
    # TODO: replace the line below.
    raise NotImplementedError("transform_points")


# --------------------------------------------------------------------------- #
# TODO 4: invert a rigid transform without np.linalg.inv                       #
# --------------------------------------------------------------------------- #
def invert_transform(T: np.ndarray) -> np.ndarray:
    """
    For a rigid transform T = [R t; 0 1] the inverse is [R^T  -R^T t; 0 1].
    Return it as a 3x3 array. (It will be checked against np.linalg.inv.)
    """
    # TODO: replace the line below.
    raise NotImplementedError("invert_transform")


# --------------------------------------------------------------------------- #
# TODO 5: chain transforms to move lidar points into the odom frame            #
# --------------------------------------------------------------------------- #
def laser_points_in_odom(robot_pose: Tuple[float, float, float],
                         laser_mount: Tuple[float, float, float],
                         pts_laser: np.ndarray) -> np.ndarray:
    """
    robot_pose  = (x, y, yaw) of base_link in odom          -> T_odom_base
    laser_mount = (x, y, yaw) of the laser in base_link      -> T_base_laser
    pts_laser   = (M, 2) points in the laser frame

    Return the (M, 2) points expressed in odom. Think about the order of the
    matrix product: which transform is applied to the points first?
    """
    # TODO: replace the line below.
    raise NotImplementedError("laser_points_in_odom")


# --------------------------------------------------------------------------- #
# Self-checks: do not edit below this line                                     #
# --------------------------------------------------------------------------- #
def _check(name, fn):
    try:
        fn()
        print(f"[PASS] {name}")
        return True
    except NotImplementedError as e:
        print(f"[TODO] {name}: {e} not implemented yet")
    except AssertionError as e:
        print(f"[FAIL] {name}: {e}")
    except Exception as e:  # noqa: BLE001
        print(f"[ERROR] {name}: {type(e).__name__}: {e}")
    return False


def _test_make_transform():
    T = make_transform(1.0, 2.0, math.pi / 2)
    assert isinstance(T, np.ndarray) and T.shape == (3, 3), "must be a 3x3 numpy array"
    expected = np.array([[0, -1, 1], [1, 0, 2], [0, 0, 1]], dtype=float)
    assert np.allclose(T, expected, atol=1e-9), f"expected\n{expected}\ngot\n{T}"
    assert np.allclose(make_transform(0, 0, 0), np.eye(3)), "identity pose must give the identity matrix"


def _test_pose_from_transform():
    for pose in [(0.0, 0.0, 0.0), (1.5, -2.0, 0.3), (-4.0, 1.0, -2.5), (2.0, 2.0, 3.0)]:
        got = pose_from_transform(make_transform(*pose))
        assert len(got) == 3 and all(isinstance(v, float) for v in got), "return three Python floats"
        assert np.allclose(got, pose, atol=1e-9), f"pose {pose} round-tripped to {got}"


def _test_transform_points():
    T = make_transform(1.0, 0.0, math.pi / 2)  # child frame is rotated 90 deg CCW and shifted +1 in x
    pts = np.array([[1.0, 0.0], [0.0, 1.0], [2.0, 3.0]])
    got = transform_points(T, pts)
    expected = np.array([[1.0, 1.0], [0.0, 0.0], [-2.0, 2.0]])
    assert isinstance(got, np.ndarray) and got.shape == (3, 2), f"expected shape (3, 2), got {getattr(got, 'shape', None)}"
    assert np.allclose(got, expected, atol=1e-9), f"expected\n{expected}\ngot\n{got}"
    # rigid transforms preserve distances between points
    d0 = np.linalg.norm(pts[0] - pts[2]); d1 = np.linalg.norm(got[0] - got[2])
    assert abs(d0 - d1) < 1e-9, "a rigid transform must preserve distances"


def _test_invert_transform():
    for pose in [(1.0, 2.0, 0.7), (-3.0, 0.5, -1.9), (0.0, 0.0, math.pi)]:
        T = make_transform(*pose)
        Ti = invert_transform(T)
        assert np.allclose(Ti, np.linalg.inv(T), atol=1e-9), f"inverse of {pose} is wrong"
        assert np.allclose(T @ Ti, np.eye(3), atol=1e-9), "T @ inv(T) must be the identity"


def _test_laser_points_in_odom():
    # Robot at (10, 5) facing +y (yaw 90 deg). Lidar mounted 0.5 m ahead of base_link,
    # facing backwards (yaw 180 deg). A point 2 m "ahead" of the lidar is therefore
    # 2 m behind the lidar in body terms -> 1.5 m behind base_link -> in odom: (10, 5 - 1.5).
    pts_laser = np.array([[2.0, 0.0], [0.0, 1.0]])
    got = laser_points_in_odom((10.0, 5.0, math.pi / 2), (0.5, 0.0, math.pi), pts_laser)
    expected = np.array([[10.0, 3.5], [11.0, 5.5]])
    assert got.shape == (2, 2), f"expected shape (2, 2), got {got.shape}"
    assert np.allclose(got, expected, atol=1e-9), (
        f"expected\n{expected}\ngot\n{got}\n(check the order: T_odom_base @ T_base_laser, laser first)")


def main():
    print("Exercise 2: frames and transforms\n")
    results = [
        _check("make_transform", _test_make_transform),
        _check("pose_from_transform", _test_pose_from_transform),
        _check("transform_points", _test_transform_points),
        _check("invert_transform", _test_invert_transform),
        _check("laser_points_in_odom", _test_laser_points_in_odom),
    ]
    print(f"\n{sum(results)}/{len(results)} checks passing")

    # Demo: a robot drives an arc past a wall; its rear-mounted lidar sees the wall.
    # Points that are all over the place in the laser frame line up in odom.
    try:
        laser_mount = (-0.3, 0.0, math.pi)          # 30 cm behind base_link, facing backwards
        wall_odom = np.stack([np.linspace(0, 6, 25), np.full(25, 2.0)], axis=1)  # the true wall y = 2
        poses, scans_laser, scans_odom = [], [], []
        for k in range(4):
            pose = (1.0 + 1.2 * k, 0.2 * k, 0.25 * k)  # (x, y, yaw) of base_link in odom
            T_odom_laser = make_transform(*pose) @ make_transform(*laser_mount)
            pts_laser = transform_points(invert_transform(T_odom_laser), wall_odom)  # what the lidar "sees"
            poses.append(pose); scans_laser.append(pts_laser)
            scans_odom.append(laser_points_in_odom(pose, laser_mount, pts_laser))
        err = max(np.abs(s - wall_odom).max() for s in scans_odom)
        print(f"\nDemo: 4 scans of the same wall re-projected into odom, max error {err:.2e} m")

        import matplotlib.pyplot as plt
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.5))
        for k, s in enumerate(scans_laser):
            ax1.plot(s[:, 0], s[:, 1], ".", label=f"scan {k}")
        ax1.plot(0, 0, "k^", ms=10); ax1.set_title("wall points in the laser frame (4 scans)")
        ax1.set_aspect("equal"); ax1.grid(True); ax1.legend()
        for k, (s, pose) in enumerate(zip(scans_odom, poses)):
            ax2.plot(s[:, 0], s[:, 1], ".", ms=8)
            ax2.plot(pose[0], pose[1], "k^", ms=8)
            ax2.annotate("", xy=(pose[0] + 0.6 * math.cos(pose[2]), pose[1] + 0.6 * math.sin(pose[2])),
                         xytext=(pose[0], pose[1]), arrowprops=dict(arrowstyle="->"))
        ax2.plot(wall_odom[:, 0], wall_odom[:, 1], "k-", lw=1, alpha=0.4, label="true wall")
        ax2.set_title("same points in odom: they line up"); ax2.set_aspect("equal"); ax2.grid(True); ax2.legend()
        for ax in (ax1, ax2):
            ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
        fig.tight_layout(); plt.show()
    except NotImplementedError:
        pass
    except ImportError:
        pass


if __name__ == "__main__":
    main()
