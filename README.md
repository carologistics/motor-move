# motor_move

ROS2 Action Server for omnidirectional robot movement using a MIMO-PID controller with optional feedforward motion profile.

## Packages

- **motor_move** – Action server (C++), controls x, y and yaw simultaneously
- **motor_move_msgs** – Action message definition (`MotorMove.action`)

## Build

```bash
sudo apt-get install libeigen3-dev
cd ~/ros2/your_workspace
colcon build --packages-select motor_move_msgs motor_move
source install/setup.bash
```

## Launch

Via launch file (loads parameters from `config/motor_move.yaml`):

```bash
ros2 launch motor_move motor_move_launch.py namespace:=/robotinobase1
```

Launch arguments:

| Argument | Default | Description |
|---|---|---|
| `namespace` | `/` | Robot namespace |
| `use_sim_time` | `false` | Use simulation clock |
| `enable_tuning_log` | `false` | Enable CSV logging |
| `enable_live_tuning` | `false` | Allow runtime parameter changes |
| `tuning_remote_target` | `""` | SCP target for log transfer |

Direct run with inline parameters:

```bash
ros2 run motor_move motor_move --ros-args \
  -r __ns:=/robotinobase1 \
  -p enable_feedforward:=true \
  -p enable_tuning_log:=true \
  -p tuning_remote_target:="user@{youre_ip}:/home/user/pid_tuning"
```

## Sending Goals

**Action**: `/{namespace}/motor_move_action`
**Type**: `motor_move_msgs/action/MotorMove`

The goal is a `geometry_msgs/PoseStamped`. The `frame_id` determines whether the target is relative (base_link) or absolute (odom/map).

Move 1m forward:
```bash
ros2 action send_goal /robotinobase1/motor_move_action motor_move_msgs/action/MotorMove \
  "{motor_goal: {header: {frame_id: 'robotinobase1/base_link'}, \
  pose: {position: {x: 1.0, y: 0.0, z: 0.0}, \
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}" --feedback
```

Rotate 90 degrees:
```bash
ros2 action send_goal /robotinobase1/motor_move_action motor_move_msgs/action/MotorMove \
  "{motor_goal: {header: {frame_id: 'robotinobase1/base_link'}, \
  pose: {position: {x: 0.0, y: 0.0, z: 0.0}, \
  orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}}}}" --feedback
```

Move + rotate:
```bash
ros2 action send_goal /robotinobase1/motor_move_action motor_move_msgs/action/MotorMove \
  "{motor_goal: {header: {frame_id: 'robotinobase1/base_link'}, \
  pose: {position: {x: 1.0, y: 0.5, z: 0.0}, \
  orientation: {x: 0.0, y: 0.0, z: 0.383, w: 0.924}}}}" --feedback
```

**Feedback**: `distance_to_target` (float, meters)
**Result**: `success` (bool)

## How It Works

The controller runs a loop at 15 Hz. Each cycle it:

1. Transforms the target pose into the robot's base frame to get the current error (dx, dy, dyaw)
2. Computes a **feedforward** velocity from a trapezoidal motion profile (braking curve based on remaining distance)
3. Runs the error through a **MIMO-PID controller** (3x3 gain matrices for x, y, yaw)
4. Publishes the combined velocity as `cmd_vel`
5. Terminates when the position and yaw error are within tolerance, or on timeout

The PID uses derivative-on-measurement (not on error) to avoid spikes on setpoint changes, and has anti-windup clamping on the integral term.

## Parameters

All parameters are configured in `motor_move/config/motor_move.yaml`.

### PID Gains

3x3 diagonal matrices. The diagonal values control x, y and yaw independently.

| Parameter | Default | Description |
|---|---|---|
| `Kp` | `diag(1.8, 1.8, 1.8)` | Proportional gain – reaction to current error |
| `Ki` | `diag(0.38, 0.38, 0.38)` | Integral gain – eliminates steady-state error |
| `Kd` | `diag(0.2, 0.2, 0.2)` | Derivative gain – dampens overshoot |
| `p_max_linear` | `0.4` | P-term output limit for linear velocity [m/s] |
| `p_max_angular` | `1.0` | P-term output limit for angular velocity [rad/s] |

### Control

| Parameter | Default | Description |
|---|---|---|
| `loop_rate` | `15.0` | Control loop frequency [Hz] |
| `timeout_seconds` | `10.0` | Max time per goal before abort [s] |
| `distance_tolerance` | `0.05` | Position error threshold for goal reached [m] |
| `yaw_tolerance_degrees` | `5.0` | Yaw error threshold for goal reached [deg] |

### Feedforward

Computes a target velocity based on distance to goal using a trapezoidal profile. The robot accelerates up to max velocity and brakes smoothly before the target.

| Parameter | Default | Description |
|---|---|---|
| `enable_feedforward` | `true` | Enable motion profile feedforward |
| `max_linear_velocity` | `0.5` | Max linear speed [m/s] |
| `max_linear_acceleration` | `0.5` | Max linear acceleration [m/s²] |
| `max_angular_velocity` | `1.0` | Max angular speed [rad/s] |
| `max_angular_acceleration` | `1.0` | Max angular acceleration [rad/s²] |

### Decoupling

Compensates cross-axis coupling (e.g. x-movement affecting y). Uses a 3x3 matrix applied to the error before PID. Identity matrix = no effect.

| Parameter | Default | Description |
|---|---|---|
| `enable_decoupling` | `false` | Enable decoupling matrix |
| `decoupling_matrix` | Identity | 3x3 compensation matrix (row-major) |

### Tuning and Logging

| Parameter | Default | Description |
|---|---|---|
| `enable_live_tuning` | `false` | Allow `ros2 param set` at runtime |
| `enable_tuning_log` | `false` | Log PID data to CSV |
| `tuning_log_path` | `/home/robotino/.../tuning_logs` | Directory for CSV files |
| `tuning_remote_target` | `""` | SCP target for auto-transfer after run |

## Topics and Frames

**Publishes**: `/{namespace}/cmd_vel` (geometry_msgs/Twist)
**Requires TF2**: `{namespace}/odom` -> `{namespace}/base_link`

```

## License

Apache 2.0 – Copyright Carologistics
