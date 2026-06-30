# motor_move

Simple ROS 2 action server for moving an omnidirectional robot to a target pose.

The action goal is a `geometry_msgs/PoseStamped`. Its `header.frame_id` tells
the node which frame the target is in. The node stores that target in odom, then
looks it up against `base_link` every control cycle and publishes `cmd_vel`.

## Build

```bash
cd ~/ros2/your_workspace
colcon build --packages-select motor_move_msgs motor_move
source install/setup.bash
```

## Launch

```bash
ros2 launch motor_move motor_move_launch.py namespace:=/robotinobase1
```

Launch arguments:

| Argument | Default | Description |
|---|---|---|
| `namespace` | `/` | Robot namespace |
| `use_sim_time` | `false` | Use simulation clock |

## Sending Goals

Action: `/{namespace}/motor_move_action`
Type: `motor_move_msgs/action/MotorMove`

Move 1 m forward in the robot frame:

```bash
ros2 action send_goal /robotinobase1/motor_move_action motor_move_msgs/action/MotorMove \
  "{motor_goal: {header: {frame_id: 'robotinobase1/base_link'}, \
  pose: {position: {x: 1.0, y: 0.0, z: 0.0}, \
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}" --feedback
```

Move to an odom pose:

```bash
ros2 action send_goal /robotinobase1/motor_move_action motor_move_msgs/action/MotorMove \
  "{motor_goal: {header: {frame_id: 'robotinobase1/odom'}, \
  pose: {position: {x: 2.0, y: 1.0, z: 0.0}, \
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}" --feedback
```

Feedback: `distance_to_target` in meters.
Result: `success`.

## Control

Each cycle the node transforms the stored target into `base_link`, computes the
remaining linear and yaw error, and commands velocity toward the target.

Speed follows a simple acceleration/braking rule:

```text
speed = min(max_speed, sqrt(2 * acceleration * remaining_error))
```

Command changes are also limited by `acceleration * dt`, so the robot ramps up
to `max_speed` and ramps down as it approaches the target. The same
`max_speed` and `acceleration` parameters are used for linear and yaw motion.

## Parameters

Configured in `motor_move/config/motor_move.yaml`:

| Parameter | Default | Dynamic | Description |
|---|---:|---|---|
| `max_speed` | `0.5` | yes | Maximum linear speed [m/s] and yaw speed [rad/s] |
| `acceleration` | `0.5` | yes | Linear acceleration [m/s^2] and yaw acceleration [rad/s^2] |
Update dynamic parameters while the node runs:

```bash
ros2 param set /robotinobase1/motor_move max_speed 0.4
ros2 param set /robotinobase1/motor_move acceleration 0.3
```

## Topics and Frames

Publishes: `/{namespace}/cmd_vel` (`geometry_msgs/Twist`)

Requires TF2:

```text
{namespace}/odom -> {namespace}/base_link
```
