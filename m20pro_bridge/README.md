# m20pro_bridge

The only robot-specific package. See the root README and docs/BRIDGE_INTERFACE.md for the contract.

- `config/m20pro.yaml`: live M20 Pro profile (calibration_verified=false until on-robot acceptance). `config/m20_official_bag.yaml`: offline reference for the official M20 dataset.
- Sensor bridge checks source frame, schema, finite samples, ring bounds and monotonic acquisition times. It converts RoboSense absolute seconds to sorted relative float seconds.
- No automatic standing, mode changes, gait changes, authority requests or recovery.
- Control requires enable_control, calibration_confirmed, measured full-scale velocities and expected_gait; the launch also requires calibration_verified and rejects simulation.
- ~/arm is std_srvs/srv/SetBool. Fresh odometry and Regular/RL/HES=0/Direction=0/Charge=0/Gait feedback are required. Commands received before arming are discarded.
- While armed, a stale cmd_vel stream (older than cmd_timeout_s: goal reached, Wait behaviour, arming before Nav2 streams) sends zero and stays armed; the next command resumes. idle_disarm_s (default 30 s) of silence disarms. Feedback, odometry, NaN or error faults still disarm immediately.
- A fault enters STOPPING and sends repeated zero commands. Fresh non-Regular mode is an explicit axis-control handoff. Restoring data never automatically arms.
- Startup with defaults sends no UDP packets. Exit sends best-effort zero only; physical watchdog/emergency-stop validation remains mandatory.
- The UDP axis backend is not /NAV_CMD. SI safety limits and measured hardware full-scale values are separate.
- The vendor rslidar driver publishes the merged cloud in the body-center frame, so the profile's lidar pose is zero and the IMU pose is the MJCF imu_site.
