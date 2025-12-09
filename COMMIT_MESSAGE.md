# Commit Message

```
Add motor_move packages: Standalone ROS2 Action Server with MIMO-PID controller

This commit adds two ROS2 packages for precise robot movement control:

## Packages Added

### motor_move_msgs
- Action message definitions for motor_move communication
- Defines MotorMove.action with Goal, Result, and Feedback messages
- Required dependency for motor_move package

### motor_move
- ROS2 Action Server implementation for precise position control
- MIMO-PID controller (Multi-Input Multi-Output) for x, y, and yaw axes
- Namespace support for multi-robot setups
- Configurable PID parameters via YAML configuration

## Features

### Core Functionality
- Action-based robot movement control with feedback
- MIMO-PID controller using 3x3 gain matrices
- TF2-based coordinate transformations (odom <-> base_link)
- Velocity command publishing (cmd_vel topic)
- Distance and yaw error calculation

### Improvements and Fixes
- Fix: YAML parameters are now actually used (previously hardcoded values were used)
- Fix: MIMO_PID dimension issues corrected
  - Parametrized constructor now initializes as column vectors (3x1) instead of row vectors (1x3)
  - set_Kp/Ki/Kd functions now automatically update dimensions when matrices are set
- Enhanced debugging for x/y PID problem
  - Separate logging for error matrix (x, y, yaw)
  - Separate logging for PID output (x, y, yaw)
- Removed dead code after return statement

### Configuration
- PID parameters configurable via motor_move/config/motor_move.yaml
- Default values: Kp=1.8, Ki=0.38, Kd=0.2 (for all axes)
- Support for different parameters per axis (x, y, yaw)

## Dependencies

### ROS2 Packages (Standard)
- rclcpp / rclcpp_action
- geometry_msgs
- tf2 / tf2_ros / tf2_geometry_msgs
- nav2_common (for launch file)

### System
- Eigen3 (for matrix operations)

## Usage

### Build Order
motor_move_msgs must be built before motor_move:
```bash
colcon build --packages-select motor_move_msgs motor_move
```

### Launch
```bash
ros2 launch motor_move motor_move_launch.py namespace:=robotinobase3
```

### Send Goal
```bash
ros2 action send_goal /robotinobase3/motor_move_action motor_move_msgs/action/MotorMove \
  "{motor_goal: {header: {frame_id: 'robotinobase3/base_link'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
```

## Inputs

1. Action Goals via `/motor_move_action` topic
   - Type: motor_move_msgs/action/MotorMove
   - Contains target pose (position + orientation)

2. TF2 Transforms
   - Requires: {namespace}/odom and {namespace}/base_link frames
   - Provided by: Odometry/SLAM nodes or simulation

3. Configuration Parameters (YAML)
   - PID gains: Kp, Ki, Kd (3x3 matrices)

## Outputs

1. cmd_vel (Twist messages)
   - Topic: /{namespace}/cmd_vel
   - Type: geometry_msgs/msg/Twist
   - Rate: ~15 Hz during movement
   - Consumed by: Robot controller (e.g., robotino_driver)

2. Action Feedback
   - Topic: /{namespace}/motor_move_action/_action/feedback
   - Contains: distance_to_target (float32)
   - Rate: ~15 Hz during movement

3. Action Result
   - Topic: /{namespace}/motor_move_action/_action/result
   - Contains: success (bool)
   - Sent once when goal is reached, cancelled, or failed

## Testing

- Both packages build successfully
- Action server starts correctly
- PID controller computes correct outputs
- YAML parameters are loaded and used
- Enhanced debugging logs work correctly

## Repository Structure

```
motor_move/
├── motor_move_msgs/          # Action Messages Package
│   ├── action/
│   │   └── MotorMove.action
│   ├── CMakeLists.txt
│   └── package.xml
├── motor_move/                # Action Server Package
│   ├── src/
│   │   ├── motor_move.cpp     # Main implementation
│   │   ├── motor_move_Tim.cpp # Alternative implementation
│   │   ├── simple_move.cpp    # Incomplete implementation
│   │   └── mimo.cpp           # MIMO-PID controller
│   ├── include/
│   │   └── motor_move/
│   │       ├── motor_move.hpp
│   │       └── mimo.hpp
│   ├── launch/
│   │   └── motor_move_launch.py
│   ├── config/
│   │   └── motor_move.yaml
│   ├── CMakeLists.txt
│   └── package.xml
├── README.md                   # Comprehensive documentation
├── SETUP.md                    # Setup instructions
└── LICENSE                     # MIT License
```

## Breaking Changes

None - This is a new standalone package.

## Migration Notes

If migrating from robotino_navigation workspace:
1. Copy both packages (motor_move_msgs and motor_move)
2. Build in correct order (motor_move_msgs first)
3. Update launch files to use new package location
4. Update any hardcoded paths if necessary

## Related Issues

- Fixed issue where YAML parameters were read but not used
- Fixed MIMO_PID dimension mismatch causing potential runtime errors
- Added debugging to diagnose x/y axis PID performance differences

## Documentation

- README.md: Comprehensive usage guide
- SETUP.md: Setup and installation instructions
- Inline code comments: Detailed explanations in source files

Signed-off-by: Carologistics
```

