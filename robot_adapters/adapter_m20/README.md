# M20 adapter

See ../../.. / README.md (repository root) and docs/REPAIRS_2026-09-07.md for the current deployment contract.

- Sensor bridge checks source frame, schema, finite samples, ring bounds and monotonic acquisition times. It converts RoboSense absolute seconds to sorted relative float seconds.
- No automatic standing, mode changes, gait changes, authority requests or recovery.
- Control requires enable_control, calibration_confirmed, measured full-scale velocities and expected_gait; the launch also requires calibration_verified and rejects simulation.
- ~/arm is std_srvs/srv/SetBool. Fresh odometry and Regular/RL/HES=0/Direction=0/Charge=0/Gait feedback are required. Commands received before arming are discarded.
- A fault enters STOPPING and sends repeated zero commands. Fresh non-Regular mode is an explicit axis-control handoff. Restoring data never automatically arms.
- Startup with defaults sends no UDP packets. Exit sends best-effort zero only; physical watchdog/emergency-stop validation remains mandatory.
- The UDP axis backend is not /NAV_CMD. SI safety limits and measured hardware full-scale values are separate.
- Official sample metadata describes M20, not a verified per-device M20 Pro calibration.
