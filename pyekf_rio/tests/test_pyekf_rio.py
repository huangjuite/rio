"""Sanity tests for the shim layer, the strapdown and reve's estimator.

These are deliberately cheap and self-contained (no dataset needed). Together
they pin down the pieces most likely to be silently wrong: the tf2 quaternion
shim, the NED/FRD sign conventions, and the radar-frame rotation.
"""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

import pyekf_rio as P

G = 9.81


def base_config(**overrides) -> P.RioConfig:
    """A minimal config: 1 s levelling, radar update on, no altimeter."""
    c = P.RioConfig()
    c.T_init = 1.0
    c.g_n = G
    c.calib_gyro = False
    c.altimeter_update = False
    c.radar_update = True
    c.use_odr = False
    c.min_db = 0.0
    c.max_dist = 1000.0
    c.filter_min_z = -1000.0
    c.filter_max_z = 1000.0
    c.azimuth_thresh_deg = 90.0
    c.elevation_thresh_deg = 90.0
    for k, v in overrides.items():
        setattr(c, k, v)
    c.finalize()
    return c


def feed_static(ekf: P.EkfRio, acc, duration=2.0, rate=100.0, gyro=(0, 0, 0), t0=0.0):
    """Feed constant IMU samples for `duration` seconds."""
    n = int(duration * rate)
    for i in range(n):
        ekf.feed_imu(np.asarray(acc, float), np.asarray(gyro, float), t0 + i / rate)
    return t0 + (n - 1) / rate


# --- shim math -------------------------------------------------------------


def test_frame_matrices_are_rotations():
    M = P.frame_flu_to_frd()
    K = P.frame_radar_flu_to_rfu()
    for R in (M, K):
        assert np.allclose(R @ R.T, np.eye(3))
        assert np.isclose(np.linalg.det(R), 1.0)
    # M is an involution.
    assert np.allclose(M @ M, np.eye(3))


def test_extrinsics_compose_to_expected_quaternion():
    """Identity FLU mounting must give the 180 deg rotation about (1,1,0)."""
    c = base_config()
    assert np.allclose(
        [c.q_b_r_w, c.q_b_r_x, c.q_b_r_y, c.q_b_r_z],
        [0.0, np.sqrt(0.5), np.sqrt(0.5), 0.0],
    )
    C = Rotation.from_quat([c.q_b_r_x, c.q_b_r_y, c.q_b_r_z, c.q_b_r_w]).as_matrix()
    # radar RFU axes -> body FRD axes
    assert np.allclose(C @ [0, 1, 0], [1, 0, 0], atol=1e-12)  # forward
    assert np.allclose(C @ [1, 0, 0], [0, 1, 0], atol=1e-12)  # right
    assert np.allclose(C @ [0, 0, 1], [0, 0, -1], atol=1e-12)  # up


# --- strapdown / initialization -------------------------------------------


def test_static_level_imu_does_not_drift():
    """A level, stationary FLU IMU reads +g on z and must stay put."""
    ekf = P.EkfRio(base_config())
    feed_static(ekf, (0.0, 0.0, G), duration=11.0)

    assert ekf.initialized
    s = ekf.state
    assert np.linalg.norm(s.position) < 1e-3
    assert np.linalg.norm(s.velocity) < 1e-4
    assert abs(s.euler[0]) < 1e-6 and abs(s.euler[1]) < 1e-6


@pytest.mark.parametrize("roll_deg", [-20.0, -5.0, 5.0, 20.0])
def test_attitude_initialises_from_gravity(roll_deg):
    """Tilting about the FLU x-axis must be recovered as filter roll."""
    phi = np.deg2rad(roll_deg)
    # Level specific force is (0, 0, g) in FLU; rotate the sensor by -phi.
    acc = Rotation.from_euler("x", -phi).as_matrix() @ np.array([0.0, 0.0, G])

    ekf = P.EkfRio(base_config())
    feed_static(ekf, acc, duration=2.0)

    assert ekf.initialized
    # M = Rx(pi) commutes with a rotation about the shared x axis, so roll is
    # identical in FLU and NED (only pitch and yaw change sign).
    assert np.isclose(ekf.state.euler[0], phi, atol=1e-6)
    assert abs(ekf.state.euler[1]) < 1e-6


def test_gyro_integrates_into_yaw():
    """A constant FLU yaw rate must integrate into the reported heading."""
    rate_dps = 10.0
    duration = 3.0
    ekf = P.EkfRio(base_config(T_init=0.5))
    feed_static(
        ekf,
        (0.0, 0.0, G),
        duration=duration,
        gyro=(0.0, 0.0, np.deg2rad(rate_dps)),
    )

    assert ekf.initialized
    q = ekf.state.quaternion  # wxyz, world FLU <- body FLU
    yaw = Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_euler("ZYX")[0]
    # Integration starts once initialization finishes, so only the remaining
    # window contributes.
    expected = np.deg2rad(rate_dps) * (duration - 0.5)
    assert np.isclose(yaw, expected, rtol=0.05)


# --- reve ego-velocity -----------------------------------------------------


def synthetic_cloud(v_flu, n=200, rng=None, doppler_sign=-1.0, max_range=10.0):
    """Build a radar-FLU cloud consistent with a known sensor velocity.

    A static target at unit direction `u` (radar FLU) has range rate
    `-u . v`. `doppler_sign` flips that to probe the opposite convention.
    """
    rng = rng or np.random.default_rng(0)
    # Directions within a forward-looking +-60 deg azimuth / +-30 deg elevation.
    az = rng.uniform(-np.pi / 3, np.pi / 3, n)
    el = rng.uniform(-np.pi / 6, np.pi / 6, n)
    u = np.stack(
        [np.cos(az) * np.cos(el), np.sin(az) * np.cos(el), np.sin(el)], axis=1
    )
    r = rng.uniform(1.0, max_range, n)
    xyz = u * r[:, None]
    doppler = doppler_sign * (u @ np.asarray(v_flu, float))
    return np.column_stack([xyz, doppler])


def test_ego_velocity_recovers_known_velocity():
    """With RANSAC off and no noise the LSQ solution must be exact."""
    v_true = np.array([2.0, 0.3, -0.1])
    est = P.EgoVelocityEstimator(base_config(use_ransac=False, max_sigma_x=10.0,
                                             max_sigma_y=10.0, max_sigma_z=10.0))
    res = est.estimate(synthetic_cloud(v_true))

    assert res.velocity_valid
    assert np.allclose(res.v_body, v_true, atol=1e-6)


def test_ego_velocity_sign_convention_is_detectable():
    """The wrong doppler sign must give the negated velocity, not garbage."""
    v_true = np.array([2.0, 0.3, -0.1])
    est = P.EgoVelocityEstimator(base_config(use_ransac=False, max_sigma_x=10.0,
                                             max_sigma_y=10.0, max_sigma_z=10.0))
    res = est.estimate(synthetic_cloud(v_true, doppler_sign=+1.0))

    assert res.velocity_valid
    assert np.allclose(res.v_body, -v_true, atol=1e-6)


def test_ego_velocity_survives_outliers_with_ransac():
    v_true = np.array([1.2, 0.1, 0.0])
    rng = np.random.default_rng(1)
    pc = synthetic_cloud(v_true, n=300, rng=rng)
    # 25% gross outliers in the doppler column.
    idx = rng.choice(len(pc), size=len(pc) // 4, replace=False)
    pc[idx, 3] += rng.uniform(-5.0, 5.0, len(idx))

    est = P.EgoVelocityEstimator(base_config(max_sigma_x=1.0, max_sigma_y=1.0,
                                             max_sigma_z=1.0))
    res = est.estimate(pc)

    assert res.velocity_valid
    assert res.n_inliers > 150
    assert np.allclose(res.v_body, v_true, atol=0.05)


def test_zero_velocity_is_detected():
    est = P.EgoVelocityEstimator(base_config())
    res = est.estimate(synthetic_cloud(np.zeros(3)))

    assert res.velocity_valid
    assert np.allclose(res.v_body, np.zeros(3), atol=1e-9)


# --- end to end ------------------------------------------------------------


def test_radar_update_pulls_velocity_toward_measurement():
    """After initialization, radar updates must drive the body velocity.

    Scoped to the convergence window on purpose: under perfectly constant
    velocity with no rotation, accelerometer bias and pitch are unobservable
    (a pitch error is indistinguishable from an x-axis bias), so the filter
    eventually wanders along that null direction. Real trajectories rotate and
    accelerate, which makes the pair observable.
    """
    v_true = np.array([1.0, 0.0, 0.0])
    # Noiseless detections make reve report sigma ~ 0, which drives the Kalman
    # gain to infinity. Real data always carries some spread; the sigma offsets
    # are reve's own floor for exactly this reason.
    cfg = base_config(
        T_init=1.0,
        outlier_percentil_radar=0.0,
        sigma_offset_radar_x=0.05,
        sigma_offset_radar_y=0.05,
        sigma_offset_radar_z=0.05,
    )
    ekf = P.EkfRio(cfg)
    rng = np.random.default_rng(7)

    t = 0.0
    dt = 0.01
    applied = 0
    for step in range(400):
        t = step * dt
        ekf.feed_imu(np.array([0.0, 0.0, G]), np.zeros(3), t)
        if ekf.initialized and step % 10 == 0:
            pc = synthetic_cloud(v_true, rng=rng)
            pc[:, 3] += rng.normal(0.0, 0.05, len(pc))
            res = ekf.feed_radar(pc, t)
            applied += int(res.update_applied)

    assert applied > 25
    # The strapdown sees zero acceleration, so the filter should settle on the
    # measured ego velocity.
    assert np.allclose(ekf.state.velocity_body, v_true, atol=0.05)
