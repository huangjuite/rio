"""ROS-free Python interface to ekf_rio (Doer & Kohl) and reve.

The upstream implementation is a ROS1/catkin workspace. This package builds
`rio::EkfRioFilter`, `rio::Strapdown` and `reve::RadarEgoVelocityEstimator`
against a set of compatibility shim headers instead, so the estimator can be
driven directly from Python.

All quantities crossing this boundary use **FLU** conventions (x forward,
y left, z up) for the world and body frames, and radar FLU for point clouds.
The NED/FRD and radar-RFU conversions ekf_rio and reve expect internally are
applied in C++; see `src/pyekf_rio/frames.h`.
"""

from dataclasses import dataclass, field

import numpy as np

from ._core import (
    EgoVelocityEstimator,
    EkfRioRunner,
    LogLevel,
    RadarResult,
    RioConfig,
    RioState,
    frame_flu_to_frd,
    frame_radar_flu_to_rfu,
    load_config,
    set_log_level,
)

__all__ = [
    "EgoVelocityEstimator",
    "EkfRio",
    "EkfRioRunner",
    "Keyframe",
    "LogLevel",
    "RadarResult",
    "RioConfig",
    "RioState",
    "frame_flu_to_frd",
    "frame_radar_flu_to_rfu",
    "load_config",
    "set_log_level",
]


@dataclass
class Keyframe:
    """One estimated pose, field-compatible with `rpl_rio.KeyframeData`.

    Only the members downstream consumers actually read are provided, which is
    enough for `unrio.odom.eval.TrajectoryEvaluator.add_est`.

    Attributes:
        stamp: timestamp in seconds.
        position: `(3,)` world FLU position.
        rotation: `(4,)` quaternion in `wxyz` order.
        velocity: `(3,)` world FLU velocity.
        velocity_body: `(3,)` body FLU velocity.
        radar_velocity: `(3,)` radar ego velocity in the radar RFU frame.
        radar_velocity_cov: `(3, 3)` covariance of `radar_velocity`.
        imu_bias_acc: `(3,)` accelerometer bias, body FLU.
        imu_bias_gyro: `(3,)` gyroscope bias, body FLU.
        pose_cov: `(6, 6)` position/attitude block of the filter covariance.
    """

    stamp: float
    position: np.ndarray
    rotation: np.ndarray
    velocity: np.ndarray
    velocity_body: np.ndarray = field(
        default_factory=lambda: np.zeros(3)
    )
    radar_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    radar_velocity_cov: np.ndarray = field(default_factory=lambda: np.zeros((3, 3)))
    imu_bias_acc: np.ndarray = field(default_factory=lambda: np.zeros(3))
    imu_bias_gyro: np.ndarray = field(default_factory=lambda: np.zeros(3))
    pose_cov: np.ndarray = field(default_factory=lambda: np.zeros((6, 6)))

    @classmethod
    def from_result(cls, result: RadarResult) -> "Keyframe":
        """Build a keyframe from a `RadarResult`."""
        s = result.state
        cov = np.asarray(s.covariance)
        pose_cov = np.zeros((6, 6))
        if cov.shape[0] >= 9:
            # (position, attitude) blocks, at error-state indices 0 and 6.
            idx = np.r_[0:3, 6:9]
            pose_cov = cov[np.ix_(idx, idx)]
        return cls(
            stamp=s.t,
            position=np.asarray(s.position),
            rotation=np.asarray(s.quaternion),
            velocity=np.asarray(s.velocity),
            velocity_body=np.asarray(s.velocity_body),
            radar_velocity=np.asarray(result.v_radar),
            radar_velocity_cov=np.asarray(result.cov_v_radar),
            imu_bias_acc=np.asarray(s.bias_acc),
            imu_bias_gyro=np.asarray(s.bias_gyro),
            pose_cov=pose_cov,
        )


class EkfRio:
    """Convenience wrapper around `EkfRioRunner`.

    Args:
        config: filter configuration.
    """

    def __init__(self, config: RioConfig) -> None:
        self._runner = EkfRioRunner(config)

    @classmethod
    def from_config(cls, path: str) -> "EkfRio":
        """Build from a YAML config file."""
        return cls(load_config(path))

    @property
    def initialized(self) -> bool:
        """True once the IMU levelling window has completed."""
        return self._runner.initialized

    @property
    def state(self) -> RioState:
        """Current filter state, in FLU conventions."""
        return self._runner.state

    @property
    def config(self) -> RioConfig:
        """The active configuration, with extrinsics already composed."""
        return self._runner.config

    def reset(self) -> None:
        """Drop all state and start over."""
        self._runner.reset()

    def feed_imu(
        self, acc: np.ndarray, gyro: np.ndarray, t: float
    ) -> bool:
        """Feed one body-FLU IMU sample.

        Args:
            acc: `(3,)` specific force in m/s^2.
            gyro: `(3,)` angular rate in rad/s.
            t: timestamp in seconds.

        Returns:
            True on the sample that completes initialization.
        """
        return self._runner.feed_imu(
            np.asarray(acc, dtype=np.float64),
            np.asarray(gyro, dtype=np.float64),
            float(t),
        )

    def feed_radar(self, pc: np.ndarray, t: float) -> RadarResult:
        """Feed one radar scan.

        Args:
            pc: `(N, 4)` as `(x, y, z, doppler)` or `(N, 6)` with
                `(snr_db, noise_db)` appended, in radar FLU coordinates.
            t: timestamp in seconds.

        Returns:
            The estimator/update outcome, including the resulting state.
        """
        return self._runner.feed_radar(np.asarray(pc, dtype=np.float64), float(t))
