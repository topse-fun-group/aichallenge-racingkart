"""Per-vehicle finite-difference velocity tracker for V2X positions.

This module is intentionally pure Python with no rclpy dependency: it
operates on duck-typed messages whose attributes match
``v2x_msgs/V2XVehiclePositionArray``. That keeps it cheap to unit-test
and reusable from non-ROS contexts (e.g. offline replay of rosbag CSVs).
"""

import math
from collections import deque
from typing import Deque, Dict, List, Tuple


def _stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


# 速度を求める差分の時間窓。V2X は 20Hz で届くため、隣接 2 点 (dt≈0.05s) の差分は
# 位置ノイズがそのまま速度に乗る。本番 rosbag (v1.3.5) の実測:
#
#     窓[s]   ジッタ中央   ジッタ95%   最大推定速度
#     0.00      4.57       19.70        63.6 km/h   <- 隣接 2 点 (従来)
#     0.25      0.62        4.70        43.9 km/h
#     0.50      0.30        2.27        41.4 km/h
#
# 実車は 35km/h 以下なので、従来の推定は最大 63.6km/h と使い物にならなかった。
# 0.25s なら遅れ 0.125s (制御 40Hz・コミット期間 1.5s に対して十分小さい) で
# ジッタを 95%tile で 4 分の 1 に落とせる。
V2X_VELOCITY_WINDOW_SEC = 0.25

# 窓に必要なサンプル数の余裕。実測のサンプル間隔は中央 0.07s。
_SAMPLE_MAXLEN = 16


class V2XVehicleTracker:
    """Tracks recent samples per ``vehicle_id`` and exposes
    constant-velocity predictions over a caller-provided time grid.

    Velocity is differenced over ``velocity_window_sec`` rather than between
    consecutive samples, so position noise is not amplified by the 20 Hz rate.
    """

    def __init__(self, v_max_safety: float, position_jump_threshold: float, warn_callback=None,
                 velocity_window_sec: float = V2X_VELOCITY_WINDOW_SEC):
        self._v_max_safety = float(v_max_safety)
        self._jump_thresh = float(position_jump_threshold)
        self._window = float(velocity_window_sec)
        self._warn = warn_callback if warn_callback is not None else (lambda _msg: None)
        self._samples: Dict[str, Deque[Tuple[float, float, float]]] = {}
        self._velocities: Dict[str, Tuple[float, float]] = {}
        self._active: List[str] = []

    def update(self, msg) -> None:
        active: List[str] = []
        for v in msg.vehicles:
            vid = v.vehicle_id
            t = _stamp_to_seconds(v.header.stamp)
            x = float(v.position.x)
            y = float(v.position.y)
            buf = self._samples.setdefault(vid, deque(maxlen=_SAMPLE_MAXLEN))

            # Detect a position jump against the previous sample (if any).
            jumped = False
            if buf:
                _t_prev, x_prev, y_prev = buf[-1]
                if math.hypot(x - x_prev, y - y_prev) > self._jump_thresh:
                    buf.clear()
                    jumped = True
                    self._warn(
                        f"V2X: position jump for vehicle '{vid}' "
                        f"(>{self._jump_thresh} m) — velocity reset")

            buf.append((t, x, y))

            if jumped or len(buf) < 2:
                self._velocities[vid] = (0.0, 0.0)
            else:
                t1, x1, y1 = buf[-1]
                # 窓に収まる最も古いサンプルとの差分を取る。最新サンプル自身を
                # 選ぶと dt=0 になるので、候補は buf[:-1] に限る。窓より粗く
                # しか届いていないときは直前サンプル (従来と同じ挙動) に落ちる。
                t0, x0, y0 = buf[-2]
                for sample in list(buf)[:-1]:
                    if t1 - sample[0] <= self._window:
                        t0, x0, y0 = sample
                        break
                dt = t1 - t0
                if dt > 0.0:
                    vx = (x1 - x0) / dt
                    vy = (y1 - y0) / dt
                    if math.hypot(vx, vy) > self._v_max_safety:
                        self._velocities[vid] = (0.0, 0.0)
                        self._warn(
                            f"V2X: velocity for vehicle '{vid}' exceeds "
                            f"{self._v_max_safety} m/s — clamped to zero")
                    else:
                        self._velocities[vid] = (vx, vy)
                else:
                    self._velocities[vid] = (0.0, 0.0)
            active.append(vid)
        self._active = active

    def velocity(self, vehicle_id: str) -> Tuple[float, float]:
        return self._velocities.get(vehicle_id, (0.0, 0.0))

    def predict_positions(
        self, vehicle_id: str, t_samples
    ) -> List[Tuple[float, float]]:
        buf = self._samples.get(vehicle_id)
        if not buf:
            return []
        _t_last, x_last, y_last = buf[-1]
        vx, vy = self._velocities.get(vehicle_id, (0.0, 0.0))
        return [(x_last + vx * t, y_last + vy * t) for t in t_samples]

    def active_vehicle_ids(self) -> List[str]:
        return list(self._active)

    def predict_all(self, t_samples) -> Dict[str, List[Tuple[float, float]]]:
        return {vid: self.predict_positions(vid, t_samples) for vid in self._active}


def predictions_to_obstacles(predictions, vehicle_radius: float, obstacle_cls=None):
    """Flatten a ``{vehicle_id: [(x, y), ...]}`` mapping into a list of
    circular obstacles consumable by ``multi_purpose_mpc_ros.core.map``.

    ``obstacle_cls`` is injectable for testability; production callers
    leave it as ``None`` to use ``core.map.Obstacle``. The deferred
    import keeps this module's load time fast and lets the unit tests
    on hosts without ``scikit-image`` exercise the helper with a stub
    dataclass.
    """
    if obstacle_cls is None:
        from multi_purpose_mpc_ros_with_dynamic_param.core.map import Obstacle as obstacle_cls
    out = []
    for _vid, points in predictions.items():
        for x, y in points:
            out.append(obstacle_cls(cx=x, cy=y, radius=vehicle_radius))
    return out
