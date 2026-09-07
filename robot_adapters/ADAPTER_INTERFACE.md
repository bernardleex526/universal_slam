# Adapter contract

## Inputs to the algorithm

- Normalized PointCloud2: x/y/z/intensity float32, time float32 relative seconds, ring uint16 for the supported mechanical-LiDAR branch. Sensor pose is explicitly configured; renaming a frame must not imply a geometric transform.
- Imu: angular velocity rad/s, acceleration m/s^2 including gravity, sensor acquisition stamp. Empty source frame requires explicit profile permission.
- Canonical outputs /uslam/points_raw and /uslam/imu_raw are reliable; source subscription QoS is separately configurable. LIO subscriptions match canonical outputs.
- Original time base must be common. Do not replace sensor timestamps with arrival time. Reject invalid/non-monotonic scans and do not invent ring/time fields.
- Official M20 bag uses absolute float64 timestamp and ring0..191. Its adapter converts and sorts by real per-point time.

## Odometry and TF

LIO publishes IMU-centered state on /uslam/lio/odometry. The odometry adapter applies base/IMU extrinsics including velocity lever arm and covariance, then publishes /uslam/odom and the only global odom->base_link TF. LIO native TF is private.

## Control

Sensor enable is independent of control enable. Defaults never send control requests. An enabled backend must be explicitly armed, verify fresh feedback/odometry and reject NaN/Inf. Limits use SI units; normalized backends additionally require measured full-scale values for the selected gait. Never infer hardware full scale from a conservative safety limit.

Faults latch; fresh data does not automatically resume motion or reclaim authority. Retain the vendor balance controller, hardware emergency stop and tested loss-of-command watchdog. SW01 control is deliberately disabled pending SDK verification.

## Deployment

Profiles are unverified by default and cannot enable navigation/control until real calibration and footprint/ground validation. /NAV_CMD integration and reusable-map relocalization are not implemented in this revision. Refer to the root README for executable commands, not historical PLAN.md assumptions.
