#!/usr/bin/env python3
"""
CSC477 Tutorial 2, Exercise 1: laser scan geometry (no ROS required).

A 2D lidar reports one range measurement per beam. In ROS the message type is
sensor_msgs/msg/LaserScan. Here we use a plain Python dataclass with exactly the
same field names, so everything you write in this file can be pasted into a
ROS 2 node unchanged (the node receives a real LaserScan instead).

Coordinate conventions (REP 103): x forward, y left, z up. Beam angles are
measured counter-clockwise from the x axis: 0 is straight ahead, +pi/2 is the
robot's left, -pi/2 is its right.

Your job: fill in the functions marked TODO, then run

    python3 ex1_laser_scan_geometry.py

The self-checks at the bottom tell you which functions are correct.
Only numpy is required.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np


# --------------------------------------------------------------------------- #
# A stand-in for sensor_msgs/msg/LaserScan                                     #
# --------------------------------------------------------------------------- #
@dataclass
class FakeLaserScan:
    """Mirrors the fields of sensor_msgs/msg/LaserScan (header omitted)."""

    angle_min: float  # [rad] angle of beam 0
    angle_max: float  # [rad] angle of the last beam
    angle_increment: float  # [rad] angular spacing between beams
    range_min: float  # [m]  returns below this are invalid
    range_max: float  # [m]  returns above this are invalid (inf = no return)
    ranges: List[float] = field(default_factory=list)


def make_scan(
    obstacles: List[Tuple[float, float, float]],
    n_beams: int = 271,
    fov_deg: float = 270.0,
    range_min: float = 0.1,
    range_max: float = 10.0,
    noise_std: float = 0.0,
    seed: int = 0,
) -> FakeLaserScan:
    """
    Simulate a scan of a scene made of circular obstacles.

    obstacles: list of (x, y, radius) in the laser frame (x forward, y left).
    Beams that hit nothing return +inf, as real lidars report.
    """
    half_fov = math.radians(fov_deg) / 2.0
    angle_min, angle_max = -half_fov, half_fov
    angle_increment = (angle_max - angle_min) / (n_beams - 1)
    angles = angle_min + np.arange(n_beams) * angle_increment
    d = np.stack([np.cos(angles), np.sin(angles)], axis=1)  # unit ray directions (N, 2)

    ranges = np.full(n_beams, np.inf)
    for cx, cy, r in obstacles:
        # ray o + t d (o = origin) meets circle |p - c| = r  ->  t^2 - 2 t (d.c) + |c|^2 - r^2 = 0
        b = d @ np.array([cx, cy])
        c = cx * cx + cy * cy - r * r
        disc = b * b - c
        hit = disc >= 0
        t = b[hit] - np.sqrt(disc[hit])
        t = np.where(t > 0, t, np.inf)
        ranges[hit] = np.minimum(ranges[hit], t)

    if noise_std > 0:
        rng = np.random.default_rng(seed)
        finite = np.isfinite(ranges)
        ranges[finite] += rng.normal(0.0, noise_std, finite.sum())
    ranges[(ranges > range_max) | (ranges < range_min)] = np.inf

    return FakeLaserScan(float(angle_min), float(angle_max), float(angle_increment),
                         range_min, range_max, ranges.tolist())


# --------------------------------------------------------------------------- #
# TODO 1: beam index -> angle                                                  #
# --------------------------------------------------------------------------- #
def beam_angles(scan: FakeLaserScan) -> np.ndarray:
    """
    Return a numpy array with the angle [rad] of every beam in `scan.ranges`.

    Beam i is at angle  scan.angle_min + i * scan.angle_increment.
    The returned array must have the same length as scan.ranges.
    """
    # TODO: replace the line below.
    raise NotImplementedError("beam_angles")


# --------------------------------------------------------------------------- #
# TODO 2: which beams are valid?                                               #
# --------------------------------------------------------------------------- #
def valid_mask(scan: FakeLaserScan) -> np.ndarray:
    """
    Return a boolean numpy array, True for beams whose range is usable:
    finite (not inf / nan) and within [scan.range_min, scan.range_max].
    """
    # TODO: replace the line below. Hint: np.isfinite.
    raise NotImplementedError("valid_mask")


# --------------------------------------------------------------------------- #
# TODO 3: polar -> Cartesian                                                   #
# --------------------------------------------------------------------------- #
def scan_to_points(scan: FakeLaserScan) -> np.ndarray:
    """
    Convert the VALID beams into an (M, 2) array of (x, y) points in the laser
    frame, where x = r cos(theta) and y = r sin(theta).

    M is the number of valid beams. This is the first step of anything that
    treats a scan as geometry: line fitting, mapping, drawing markers in rviz2.
    """
    # TODO: replace the line below. Use beam_angles() and valid_mask().
    raise NotImplementedError("scan_to_points")


# --------------------------------------------------------------------------- #
# TODO 4: nearest obstacle                                                     #
# --------------------------------------------------------------------------- #
def nearest_obstacle(scan: FakeLaserScan) -> Optional[Tuple[float, float]]:
    """
    Return (range, bearing) of the closest valid return as Python floats,
    where bearing is the beam angle in radians. Return None if no beam is valid.

    Careful: np.argmin over an array that contains inf or nan gives garbage.
    Mask first, or replace invalid ranges with +inf before taking argmin.
    """
    # TODO: replace the line below.
    raise NotImplementedError("nearest_obstacle")


# --------------------------------------------------------------------------- #
# TODO 5: front clearance                                                      #
# --------------------------------------------------------------------------- #
def front_clearance(scan: FakeLaserScan, half_width: float = math.radians(15)) -> Optional[float]:
    """
    Minimum valid range among beams within +/- half_width of straight ahead
    (angle 0), as a Python float. Return None if there is no valid beam there.

    This is the quantity behind every "emergency stop" behaviour. Using a
    sector instead of the single 0-degree beam makes it robust to one noisy
    return and to thin obstacles that fall between beams.
    """
    # TODO: replace the line below.
    raise NotImplementedError("front_clearance")


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


def _test_beam_angles():
    scan = make_scan([], n_beams=5, fov_deg=180.0)
    ang = beam_angles(scan)
    assert isinstance(ang, np.ndarray), "must return a numpy array"
    assert ang.shape == (5,), f"expected shape (5,), got {ang.shape}"
    expected = np.deg2rad([-90, -45, 0, 45, 90])
    assert np.allclose(ang, expected, atol=1e-9), f"expected {expected}, got {ang}"


def _test_valid_mask():
    scan = FakeLaserScan(
        angle_min=0.0, angle_max=1.0, angle_increment=0.2, range_min=0.5, range_max=5.0,
        ranges=[0.1, 0.5, 2.0, 5.0, 7.5, float("inf"), float("nan")],
    )
    m = valid_mask(scan)
    assert m.dtype == bool, "must return a boolean array"
    expected = np.array([False, True, True, True, False, False, False])
    assert np.array_equal(m, expected), f"expected {expected}, got {m}"


def _test_scan_to_points():
    # One small pole straight ahead at (3, 0) and one to the left at (0, 2).
    scan = make_scan([(3.0, 0.0, 0.2), (0.0, 2.0, 0.2)], n_beams=361, fov_deg=360.0)
    pts = scan_to_points(scan)
    assert isinstance(pts, np.ndarray) and pts.ndim == 2 and pts.shape[1] == 2, \
        f"expected an (M, 2) array, got shape {getattr(pts, 'shape', None)}"
    assert pts.shape[0] == int(valid_mask(scan).sum()), "one point per VALID beam"
    assert np.all(np.isfinite(pts)), "points must be finite (filter invalid beams first)"
    # every point lies on the surface of one of the two poles
    d1 = np.hypot(pts[:, 0] - 3.0, pts[:, 1] - 0.0)
    d2 = np.hypot(pts[:, 0] - 0.0, pts[:, 1] - 2.0)
    on_surface = (np.abs(d1 - 0.2) < 1e-6) | (np.abs(d2 - 0.2) < 1e-6)
    assert on_surface.all(), "some points are not where the obstacles are (check cos/sin and x/y order)"
    # the beam pointing straight ahead must give (2.8, 0)
    ahead = pts[np.abs(pts[:, 1]) < 1e-9]
    assert len(ahead) and abs(ahead[0, 0] - 2.8) < 1e-6, f"beam at angle 0 should hit (2.8, 0), got {ahead}"


def _test_nearest_obstacle():
    scan = make_scan([(3.0, 0.0, 0.2), (0.0, 1.5, 0.2), (-1.0, -1.0, 0.1)], n_beams=361, fov_deg=360.0)
    got = nearest_obstacle(scan)
    assert got is not None, "obstacles are visible"
    r, bearing = got
    assert isinstance(r, float) and isinstance(bearing, float), "return Python floats"
    assert abs(r - 1.3) < 1e-6, f"nearest pole is 1.5 - 0.2 = 1.3 m away, got {r}"
    assert abs(bearing - math.pi / 2) < 1e-6, f"it is on the left (+pi/2), got {bearing}"
    assert nearest_obstacle(make_scan([])) is None, "empty scene -> None"


def _test_front_clearance():
    scan = make_scan([(2.0, 0.0, 0.3), (0.0, 1.0, 0.3)], n_beams=271, fov_deg=270.0)
    got = front_clearance(scan)
    assert got is not None and isinstance(got, float), "return a Python float"
    assert abs(got - 1.7) < 1e-6, f"pole ahead at 2.0 m with radius 0.3 -> clearance 1.7, got {got}"
    # obstacle slightly off-centre but inside the +/-15 deg sector still counts
    scan = make_scan([(2.0, 0.3, 0.1)], n_beams=271, fov_deg=270.0)
    got = front_clearance(scan)
    assert got is not None and got < 2.1, f"off-centre pole should be inside the sector, got {got}"
    # obstacle at 40 deg is outside the sector
    scan = make_scan([(2.0 * math.cos(0.7), 2.0 * math.sin(0.7), 0.1)], n_beams=271, fov_deg=270.0)
    assert front_clearance(scan) is None, "nothing in the front sector -> None"


def main():
    print("Exercise 1: laser scan geometry\n")
    results = [
        _check("beam_angles", _test_beam_angles),
        _check("valid_mask", _test_valid_mask),
        _check("scan_to_points", _test_scan_to_points),
        _check("nearest_obstacle", _test_nearest_obstacle),
        _check("front_clearance", _test_front_clearance),
    ]
    print(f"\n{sum(results)}/{len(results)} checks passing")

    # A small demo once everything works: a cluttered, noisy scene.
    try:
        scene = [(2.5, 0.2, 0.3), (1.0, 1.5, 0.2), (3.0, -2.0, 0.5), (-1.5, 0.5, 0.2)]
        scan = make_scan(scene, noise_std=0.01)
        pts = scan_to_points(scan)
        r, b = nearest_obstacle(scan)
        fc = front_clearance(scan)
        print(f"\nDemo scan: {len(scan.ranges)} beams, {len(pts)} valid points, "
              f"nearest obstacle {r:.2f} m at {math.degrees(b):+.0f} deg, front clearance {fc:.2f} m")
        try:
            import matplotlib.pyplot as plt
            fig, ax = plt.subplots(figsize=(5, 5))
            for cx, cy, rad in scene:
                ax.add_patch(plt.Circle((cx, cy), rad, color="0.7"))
            ax.plot(pts[:, 0], pts[:, 1], "b.", ms=3, label="lidar returns")
            ax.plot(0, 0, "k^", ms=10, label="robot (x forward)")
            ax.set_aspect("equal"); ax.grid(True); ax.legend(loc="lower left")
            ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_title("scan in the laser frame")
            plt.show()
        except ImportError:
            pass
    except NotImplementedError:
        pass


if __name__ == "__main__":
    main()
