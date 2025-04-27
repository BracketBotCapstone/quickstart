"""localization.py
Simple differential-drive odometry helper built on top of
`lib.odrive_uart.ODriveUART`.  It continuously queries wheel encoder
counts from the ODrive and integrates them to estimate the robot pose
(x, z, yaw) in a planar world frame whose origin coincides with the
robot's start position (0, 0, 0).

When executed directly (`python -m lib.localization` or
`python lib/localization.py`) it also visualises the live pose and the
traversed path in Rerun as a 3‑D line strip under the `world/odom/path`
entity, alongside a transform representing the current robot pose at
`world/robot`.

This module can likewise be imported by other scripts (e.g.
`examples/navigate.py`) to obtain a continuously updated pose estimate
via the `DifferentialDriveOdometry` class or the convenience
`OdometryThread` wrapper.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Tuple

import numpy as np
import rerun as rr
from scipy.spatial.transform import Rotation as R_scipy
import json

# Add the project root so `lib` is importable when run as a script
import os, sys
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

from lib.odrive_uart import ODriveUART  # noqa: E402

# ───────────────────────── CONFIG ──────────────────────────
WHEEL_DIAMETER_M: float = 0.165  # 165 mm wheels
WHEEL_BASE_M: float = 0.425       # distance between wheels (centre‑to‑centre)
LOG_HZ: float = 5.0              # log frequency for standalone visualiser
RERUN_TCP_ADDR: str = "192.168.2.24:9876"  # change if needed


def _circumference(diam_m: float) -> float:
    """Return wheel circumference in metres given its diameter."""
    return math.pi * diam_m


@dataclass
class DifferentialDriveOdometry:
    """Incremental planar odometry for a differential‑drive robot."""

    wheel_base: float = WHEEL_BASE_M
    wheel_diameter: float = WHEEL_DIAMETER_M

    # Pose estimate in world frame (x points forward, z to the left to
    # match the RHS Y‑down convention used elsewhere in the codebase).
    x: float = 0.0
    z: float = 0.0
    yaw: float = 0.0  # radians, 0 = facing +x

    # Last encoder readings (in turns) to compute deltas.
    _prev_left_turns: float | None = field(default=None, init=False)
    _prev_right_turns: float | None = field(default=None, init=False)

    def reset(self) -> None:
        """Reset pose to the origin and forget previous encoder values."""
        self.x = self.z = self.yaw = 0.0
        self._prev_left_turns = self._prev_right_turns = None

    # ───────────── Main update ──────────────

    def update(self, left_turns: float, right_turns: float) -> Tuple[float, float, float]:
        """Advance the pose estimate given absolute wheel encoder turns.

        Parameters
        ----------
        left_turns, right_turns : float
            Cumulative encoder readings in *turns* since the ODrive was
            powered on.  Sign conventions depend on the `dir_left` /
            `dir_right` parameters passed to ``ODriveUART``.

        Returns
        -------
        (x, z, yaw) : tuple of float
            The updated pose estimate in metres / radians.
        """
        if self._prev_left_turns is None:
            # First call → just cache values without integrating.
            self._prev_left_turns = left_turns
            self._prev_right_turns = right_turns
            return self.x, self.z, self.yaw

        # ∆ wheel travel in metres
        dl = (left_turns - self._prev_left_turns) * _circumference(self.wheel_diameter)
        dr = (right_turns - self._prev_right_turns) * _circumference(self.wheel_diameter)

        self._prev_left_turns = left_turns
        self._prev_right_turns = right_turns

        # Differential‑drive kinematics
        dc = 0.5 * (dl + dr)                   # forward distance
        dtheta = (dr - dl) / self.wheel_base   # change in heading

        # Integrate in SE(2) — here we use the exact integration for the
        # constant‑twist motion over the dt interval.
        if abs(dtheta) > 1e-6:
            # follow an arc
            R_icc = dc / dtheta  # radius to the ICC (Instantaneous Centre of Curvature)
            half_yaw = self.yaw + 0.5 * dtheta
            # Use cos for forward (x) and sin for left (z) to match
            # the world‑frame convention (x forward, z left). This
            # was previously inverted which caused forward motion to
            # appear as positive left in the pose estimate.
            self.x += dc * math.cos(half_yaw)
            self.z += dc * math.sin(half_yaw)
        else:
            # straight‑line approximation
            self.x += dc * math.cos(self.yaw)
            self.z += dc * math.sin(self.yaw)

        self.yaw = (self.yaw + dtheta) % (2 * math.pi)
        return self.x, self.z, self.yaw


class OdometryThread(threading.Thread):
    """Background thread that keeps a `DifferentialDriveOdometry` updated."""

    def __init__(self, odrv: ODriveUART, log_to_rerun: bool = False):
        super().__init__(daemon=True)
        self.odrv = odrv
        self.odo = DifferentialDriveOdometry()
        self.log_to_rerun = log_to_rerun
        self._running = threading.Event()
        self._running.set()
        self._path_world: List[Tuple[float, float, float]] = []  # stored as (x, y, z)
        self._boxes_initialized: bool = False

        if log_to_rerun:
            rr.init("robot_localization")
            rr.connect_grpc(f"rerun+http://{RERUN_TCP_ADDR}/proxy")
            rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    # Expose latest pose
    @property
    def pose(self) -> Tuple[float, float, float]:  # (x, z, yaw)
        return self.odo.x, self.odo.z, self.odo.yaw

    # Full path for visualisation or analysis
    @property
    def path(self) -> np.ndarray:
        return np.asarray(self._path_world, dtype=np.float32)

    def run(self) -> None:
        period_s = 1.0 / LOG_HZ
        last_time = time.time()

        while self._running.is_set():
            try:
                l_turns = self.odrv.get_position_turns_left()
                r_turns = self.odrv.get_position_turns_right()
                x, z, yaw = self.odo.update(l_turns, r_turns)
                # Map body left (+z) onto viewer left (−X) so that driving
                # left in body frame shows as left in the viewer.
                x_view = -z  # X right positive → body right (−z)
                y_view = x   # Y forward = body forward
                yaw_view = yaw - math.pi / 2  # viewer yaw lags body 90°, matches axis mapping (Δyaw ≈ -90°)

                # Extra debug: show delta between body yaw and viewer yaw and explicit axis meanings
                diff_deg = ((math.degrees(yaw_view - yaw) + 180) % 360) - 180  # should be ~ -90° constant if correct

                print(
                    (
                        f"Body  ->  fwd(x)={x:+.3f}  left(z)={z:+.3f}  yaw={math.degrees(yaw):+.1f}°  | "
                        f"Viewer  ->  X(right)={x_view:+.3f}  Y(fwd)={y_view:+.3f}  yaw_view={math.degrees(yaw_view):+.1f}°  "
                        f"(Δyaw={diff_deg:+.1f}°)"
                    )
                )

                self._path_world.append((x_view, y_view, 0.0))
                # Quaternion about Z for viewer frame, with 90 deg rotation
                quat_rr = R_scipy.from_euler("z", yaw_view - math.pi/2).as_quat()

                if self.log_to_rerun and (time.time() - last_time) >= period_s:
                    last_time = time.time()
                    rr.set_time_seconds("t", last_time)
                    rr.log(
                        "world/robot",
                        rr.Transform3D(
                            translation=[x_view, y_view, 0.0],
                            rotation=rr.Quaternion(xyzw=quat_rr),
                        ),
                    )
                    # Log the robot's 3D model once per session
                    if not self._boxes_initialized:
                        model_path = Path(__file__).parent.parent / "lib" / "Bracketbot.stl"
                        rr.log(
                            "world/robot",
                            rr.Asset3D(path=str(model_path)),
                            static=True
                        )
                        self._boxes_initialized = True
                    if len(self._path_world) >= 2:
                        pts = np.asarray(self._path_world[-1000:], dtype=np.float32)
                        rr.log("world/odom/path", rr.LineStrips3D(pts, radii=0.02))
            except Exception as exc:
                print(f"[Odometry] error: {exc}")
            time.sleep(period_s)

    def stop(self) -> None:
        self._running.clear()


# ──────────────────── Stand‑alone entry‑point ────────────────────


def _standalone() -> None:  # pragma: no cover (utility script)
    odrv = ODriveUART()
    odo_thread = OdometryThread(odrv, log_to_rerun=True)
    odo_thread.start()
    print("[Odometry] running – press Ctrl‑C to quit…")

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Exiting…")
    finally:
        odo_thread.stop()
        odo_thread.join()


if __name__ == "__main__":
    _standalone()
