# pyekf_rio

A ROS-free Python interface to the estimation core of this repository:
`rio::EkfRioFilter` (ekf_rio), `rio::Strapdown` (rio_utils) and
`reve::RadarEgoVelocityEstimator` (reve).

The upstream packages are ROS1/catkin and depend on roscpp, tf2, PCL and
dynamic_reconfigure. Rather than modify them, `src/shim/` supplies
compatibility headers for each of those dependencies, so **every upstream
`.cpp` and `.h` compiles unmodified** and the fork stays rebaseable on
`christopherdoer/rio`.

## Layout

| path | purpose |
|---|---|
| `src/shim/` | stand-ins for `ros/`, `angles/`, `sensor_msgs/`, `tf2/`, `pcl/` and the two dynamic_reconfigure `*Config.h` headers |
| `src/shim_impl/` | replacements for reve's PCL-heavy `radar_point_cloud.cpp`, plus an ODR stub |
| `src/pyekf_rio/` | `RioConfig` + YAML loader, frame conventions, and `EkfRioRunner` (replaces `EkfRioRos`) |
| `python/` | nanobind module and the `pyekf_rio` package |
| `config/` | ready-made configs |

Only four upstream translation units are compiled: `rio_utils/src/strapdown.cpp`,
`ekf_rio/src/ekf_rio_filter.cpp`, `reve/.../radar_ego_velocity_estimator.cpp`
and `reve/.../odr.cpp` (plus the Fortran odrpack). Everything ROS-facing --
`*_ros.cpp`, `nodes/*`, `ekf_yrio`, `x_rio`, `gnss_x_rio` -- is excluded.

## Frames

Callers work exclusively in **FLU** (x forward, y left, z up); the NED world,
FRD body and reve RFU radar conversions happen in `src/pyekf_rio/frames.h`.

## Build

```bash
pip install -e .                       # from the fork root
cmake -S pyekf_rio/src -B build        # or C++ only
cmake --build build
```

`-DPYEKF_RIO_WITH_ODR=OFF` drops reve's ODR refinement and the Fortran
dependency.

## Use

```python
from pyekf_rio import EkfRio

ekf = EkfRio.from_config("pyekf_rio/config/iq1m.yaml")
ekf.feed_imu(acc_flu, gyro_flu, t)          # until `ekf.initialized`
result = ekf.feed_radar(pc_flu, t)          # (N, 4) or (N, 6)
print(result.state.position, result.v_body)
```

`EgoVelocityEstimator` exposes reve on its own, without the EKF, which is the
easiest way to check doppler-sign and frame conventions against ground truth.
