# Bridge contract

The algorithm (`slam_nav_core`, `universal_slam_bringup`) never depends on a robot package.
Porting to a new robot = one new bridge package. Nothing in the algorithm changes.

## What a bridge package provides

1. `config/<robot>.yaml` — the robot profile (schema below).
2. `launch/<name>.launch.py` accepting exactly these arguments:
   - `robot_config`: absolute profile path
   - `use_sim_time`: `true|false`
   - `enable_control`: `true|false` (must refuse unless the profile is verified)
3. Nodes that publish the normalized sensor inputs and consume `cmd_vel` (below).

Launch: `ros2 launch universal_slam_bringup universal_slam.launch.py robot_config:=$(ros2 pkg prefix <bridge>)/share/<bridge>/config/<robot>.yaml use_bridge:=true`.
The bringup resolves the bridge from `bridge.package` / `bridge.launch` in the profile.

## Profile keys read by the algorithm

| key | meaning |
| --- | --- |
| `bridge.package`, `bridge.launch` | bridge to include when `use_bridge:=true` |
| `calibration_verified` | `true` only after on-robot calibration/footprint acceptance; gates Nav2 and control |
| `tf.{map,odom,base,lidar,imu}_frame` | frame names (unique, no leading slash) |
| `topics.{points,imu,odom,cmd_vel,obstacles}_topic` | canonical topic names |
| `extrinsics.imu`, `extrinsics.lidar` | base->sensor pose, metres and degrees |
| `lidar.{lidar_type,scan_line,timestamp_unit,scan_rate,blind}` | FAST-LIO preprocessing |
| `velocity.{vx_max,vy_max,wz_max}` | SI navigation limits (MPPI, behaviours, divergence bounds) |
| `navigation.robot_radius`, `navigation.base_height` | footprint; base_link height above the floor |

Every other key (sensor sources, network, control calibration) is private to the bridge.

## Inputs to the algorithm

- Normalized PointCloud2 on `topics.points_topic`, frame `tf.lidar_frame`: x/y/z/intensity float32, time float32 relative seconds, ring uint16 (mechanical-LiDAR branch). Sensor pose is explicitly configured; renaming a frame must not imply a geometric transform.
- Imu on `topics.imu_topic`, frame `tf.imu_frame`: angular velocity rad/s, acceleration m/s^2 including gravity, sensor acquisition stamp.
- Both reliable QoS. Original time base must be common; do not replace sensor timestamps with arrival time. Reject invalid/non-monotonic scans and do not invent ring/time fields.

## Outputs of the algorithm

- `topics.odom_topic` (nav_msgs/Odometry, `odom_frame` -> `base_frame`) and the only `odom->base_link` TF, produced by the odometry adapter from IMU-centered LIO state. If LIO odometry becomes physically implausible (speed or position jump), the adapter latches and stops publishing until `/odometry_adapter/reset` is called; bridges must treat missing odometry as a stop condition.
- `topics.cmd_vel_topic` (geometry_msgs/Twist, SI units, base frame) from Nav2.

## Control safety (bridge responsibility)

Sensor enable is independent of control enable. Defaults never send control requests. An enabled backend must be explicitly armed, verify fresh feedback/odometry and reject NaN/Inf. Limits use SI units; normalized backends additionally require measured full-scale values. Never infer hardware full scale from a conservative safety limit. Faults latch; fresh data does not automatically resume motion or reclaim authority. Retain the vendor balance controller, hardware emergency stop and a tested loss-of-command watchdog.
