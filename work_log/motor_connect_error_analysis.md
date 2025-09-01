# motor_connect.cpp Position Error Analysis (Joints 2–3)

This note explains why the system fails to reach the commanded pulses on joints 2 and 3 (steady error ≈ ±150 counts), and which parts of the code contribute to it. It is written to be self-contained for another engineer/AI to understand the situation without prior context.

## System Overview

- Low-level control: `src/dongsoo_cpp_pkg/src/motor_connect.cpp`
  - Dynamixel X-series motors (IDs 1–5) in current control mode
  - BulkRead of present current/velocity/position; SyncWrite of `Goal Current (102)`
  - Position PID (in raw counts) + gravity feedforward (FF) in current raw counts
  - Publishers:
    - `motor/position` (Int32MultiArray, raw counts for 5 motors)
    - `motor/current`, `motor/velocity` (Float32MultiArray)
  - Subscribers:
    - `motor/command_position` (Int32MultiArray, targets for motors 1–4)
    - `motor/command_dxl5_position` (Int32MultiArray, target for motor 5)

- Mid-level apps (Python): consume `/motor/position` and compute FK/poses, plan trajectories, and call IK services.
  - `src/dongsoo_py_pkg/dongsoo_py_pkg/data_hub.py`: publishes camera/gripper 4x4 matrices
  - `src/dongsoo_py_pkg/dongsoo_py_pkg/monitoring_hub.py`: monitors state and compares poses
  - `src/dongsoo_py_pkg/dongsoo_py_pkg/dongsoo_server.py`: service → IK → trajectory → publishes `/motor/command_position`

- Robot parameters (JSON): `src/dongsoo_description/config/`
  - `gravity_dh_param.json`: 4-DOF (J1–J4) DH for gravity compensation
  - `link_inertial.json`: link mass and COM in each link’s DH frame

## Symptom

- Using `ros2 topic pub --once` to trigger the client → service → IK → `/motor/command_position`, the motors move but J2 and J3 settle with a steady position error about ±150 counts.
- Logging of per-joint error and present current confirms the problem is most severe for joints 2 and 3.

## Control Path Summary

1) Service computes a joint trajectory (rad) → converts to pulses → publishes to `/motor/command_position`.
2) `motor_connect.cpp` stores the desired pulses for J1–J4 and runs a 200 Hz loop:
   - BulkRead presents → compute `q(rad)` for J1–J4 → compute gravity torque τg
   - Convert τg to current raw (LSB) and add to PID output → clamp → SyncWrite
   - Publish present currents/velocities/positions

## Key Observations

- Position PID gains (raw counts → current raw):
  - `KP_POS_GAINS = {0.5, 1.25, 1.0, 1.0}`
  - `KI_POS_GAINS = {0.0, 0.01, 0.01, 0.0}`  (no I on J1/J4; small I on J2/J3)
  - `KD_POS_GAINS = {0.012, 0.015, 0.012, 0.01}`
- Gravity FF uses DH + link COM to compute τg and adds it in current raw units.
- SyncWrite output is clamped: ±900 for XH540 (J1–J3), ±500 for XH430 (J4).

## Likely Root Causes (specifically for J2–J3)

1) Gravity vector parsing bug (FF may be effectively OFF)

- Code attempts to read `gravity_vector` by checking if a single line contains both the key ("x"/"y"/"z") and the word "gravity".
- In the actual JSON, the axis lines (x/y/z) are on their own lines and do not include the word "gravity".
- Result: `gvec_` remains default-initialized (zeros), making τg ≈ 0 → no FF to counter gravity.

File: `src/dongsoo_cpp_pkg/src/motor_connect.cpp:101`–`106`

```cpp
} else if (line.find("\"x\"") != std::string::npos && line.find("gravity") != std::string::npos) {
  params.gravity_vector.x() = extractNumber(line, "x");
} else if (line.find("\"y\"") != std::string::npos && line.find("gravity") != std::string::npos) {
  params.gravity_vector.y() = extractNumber(line, "y");
} else if (line.find("\"z\"") != std::string::npos && line.find("gravity") != std::string::npos) {
  params.gravity_vector.z() = extractNumber(line, "z");
}
```

2) `d1` parsing can be overwritten to 0.0

- The code sets `d1` whenever a line contains `"d"` with no guard. Later `d: 0.0` lines (from other joints) overwrite the base joint’s `d1`.
- An incorrect `d1` corrupts transformations and the Jacobian, leading to wrong τg, most impacting J2–J3.

File: `src/dongsoo_cpp_pkg/src/motor_connect.cpp:85`–`100`

```cpp
if (line.find("\"d\"") != std::string::npos) {
  params.d1 = extractNumber(line, "d");
}
// ... there is no guard to only set this once for J1
```

3) `zero_count_` affects gravity FF (even if other nodes use 2048 as zero)

- PID error uses `desired_pos - pos_count` (zero-unaware), but gravity FF uses `q = sign*(pos_count - zero_count_)*COUNT2RAD_`.
- If `zero_count_` is not set to the actual joint zero (often ≈ 2048 or a calibrated home), the FF angles are wrong (potentially π-shifted). This creates a constant torque bias that the PID must counter, causing steady error.

Files:
- Definition: `src/dongsoo_cpp_pkg/src/motor_connect.cpp:376`–`377`
- Usage: `src/dongsoo_cpp_pkg/src/motor_connect.cpp:506`–`509`

```cpp
std::array<int32_t,4>  zero_count_{ {0, 0, 0, 0} };
// ...
q_rad[i] = sign_[i] * ((pos_count[i] - zero_count_[i]) * COUNT2RAD_);
```

4) Limited integral action on J2–J3

- J2/J3 have small but nonzero `KI` (0.01). With FF broken or misaligned, this may be too small to drive steady bias to zero in a reasonable time, especially under current limits.

5) Current saturation

- Combined FF error + PID can hit ±900 (J1–J3) or ±500 (J4). If saturated often, steady error is expected.

## Why J2 and J3 specifically?

- J2 (shoulder) and J3 (elbow) experience the largest gravity moments. If FF is missing or wrong, these axes show the biggest steady-state errors. J1 and J4 see less gravity-induced load, so they appear fine.

## Quick Diagnostics

- Inspect gravity debug logs (printed every 5 s):
  - `q[rad]`, `tau_g[Nm]`, `pos_err[cnt]`, `tau_g_raw[LSB]`
  - If `tau_g` is near zero across poses, the gravity vector parsing is broken.
  - At a neutral pose (encoders ~2048), expect `q ≈ [0, π/2, 0, 0]` (J2 has θ offset). If not, check `zero_count_`/`sign_`.

File: `src/dongsoo_cpp_pkg/src/motor_connect.cpp:514`–`523`

- Add a temporary log when `cmd_raw` hits its clamp to see saturation.

## Recommended Fixes (priority order)

1) Fix gravity vector parsing

- Remove the `line.find("gravity")` requirement and parse `x`, `y`, `z` once when inside the `gravity_vector` block, or switch to a proper JSON parser.

2) Guard `d1` parsing

- Only set `d1` for the first joint’s `d`. Do not overwrite on subsequent `d: 0.0` lines.

3) Set proper `zero_count_` and verify `sign_`

- Set `zero_count_` to calibrated home or, as a minimum, `{2048, 2048, 2048, 2048}` to align FF with other nodes’ notion of zero.
- Ensure `sign_` matches the convention “positive current → positive joint angle”.

4) Consider slightly higher KI on J2/J3 after FF is corrected

- If a small steady error persists, increase KI moderately (e.g., 0.015–0.03) keeping anti-windup.

5) Optional anti-windup improvement

- Stop integrating when the output is saturated and the error would push further into saturation.

## Minimal Patch Plan (not yet applied)

- Expose `zero_count_` and `sign_` as ROS parameters (e.g., `~zero_counts`, `~signs`) with safe defaults.
- Fix parsing for `gravity_dh_param.json`:
  - Read `gravity_vector` correctly.
  - Protect `d1` from being overwritten.
- Add a transient log when `cmd_raw` hits the clamp to detect saturation behavior.

## Code Pointers

- `src/dongsoo_cpp_pkg/src/motor_connect.cpp:172`–`191` gains and constants
- `src/dongsoo_cpp_pkg/src/motor_connect.cpp:235`–`243` count→rad constants
- `src/dongsoo_cpp_pkg/src/motor_connect.cpp:430`–`469` gravity torque `computeGravityTorqueNm`
- `src/dongsoo_cpp_pkg/src/motor_connect.cpp:538`–`545` PID + FF sum
- `src/dongsoo_cpp_pkg/src/motor_connect.cpp:548`–`557` output clamp
- `src/dongsoo_cpp_pkg/src/motor_connect.cpp:514`–`523` gravity debug logs

## Final Notes

- Even if all other nodes (DataHub/Monitoring) use 2048 as 0 rad, `motor_connect.cpp` must compute `q(rad)` for FF with the same zero reference. Otherwise, gravity FF will be phase/offset incorrect and produce a constant bias, most visible on J2/J3.
- The combination of broken gravity parsing + wrong `zero_count_` explains the ±150 count residual error on J2/J3. Fixing those should substantially reduce the steady error; KI tuning can then polish the result.

