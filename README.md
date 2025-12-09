# motor_move

ROS2 Action Server für präzise Roboter-Bewegungen mit MIMO-PID Controller.

## 📦 Packages

- **motor_move_msgs**: Action Message Definitions (muss zuerst gebaut werden)
- **motor_move**: Action Server mit MIMO-PID Controller für x, y und yaw

## 🔌 Installation

```bash
# Dependencies
sudo apt-get install libeigen3-dev

# Build
cd ~/ros2/your_workspace
colcon build --packages-select motor_move_msgs motor_move
source install/setup.bash

# Start
ros2 launch motor_move motor_move_launch.py namespace:=robotino1
```

## 📥 Action Goals

**Topic**: `/{namespace}/motor_move_action`  
**Type**: `motor_move_msgs/action/MotorMove`

**Goal Fields**:
- `motor_goal`: PoseStamped mit Position (x, y) und Quaternion-Orientierung

**Beispiele**:

Vorwärtsbewegung (1m):
```bash
ros2 action send_goal /robotino1/motor_move_action motor_move_msgs/action/MotorMove "{motor_goal: {header: {frame_id: 'robotino1/base_link'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}" --feedback
```

Rotation um 90° (Quaternion):
```bash
ros2 action send_goal /robotino1/motor_move_action motor_move_msgs/action/MotorMove "{motor_goal: {header: {frame_id: 'robotino1/base_link'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}}}" --feedback
```

Bewegung + Rotation:
```bash
ros2 action send_goal /robotino1/motor_move_action motor_move_msgs/action/MotorMove "{motor_goal: {header: {frame_id: 'robotino1/base_link'}, pose: {position: {x: 1.0, y: 0.5, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.382683, w: 0.92388}}}" --feedback
```

## ⚙️ Konfiguration

**Datei**: `motor_move/config/motor_move.yaml`

```yaml
motor_move:
  ros__parameters:
    Kp: [1.8, 0.0, 0.0, 0.0, 1.8, 0.0, 0.0, 0.0, 1.8]
    Ki: [0.38, 0.0, 0.0, 0.0, 0.38, 0.0, 0.0, 0.0, 0.38]
    Kd: [0.2, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.2]
    loop_rate: 15.0
    timeout_seconds: 10.0
    yaw_tolerance_degrees: 5.0
    distance_tolerance: 0.05
```

**Benötigte TF2 Frames**: `{namespace}/odom` und `{namespace}/base_link`

## 📤 Outputs

- **cmd_vel** (`/{namespace}/cmd_vel`): Twist Messages mit Geschwindigkeitskommandos (~15 Hz)
- **Action Feedback**: `distance_to_target` während der Bewegung
- **Action Result**: `success: true/false` bei Zielerreichung, Timeout oder Fehler

## 🔧 Multi-Robot Setup

```bash
ros2 launch motor_move motor_move_launch.py namespace:=robotino1
ros2 launch motor_move motor_move_launch.py namespace:=robotino2
```

## 🐛 Debugging

```bash
ros2 action list
ros2 action info /robotino1/motor_move_action
ros2 run tf2_tools view_frames
ros2 topic echo /robotino1/cmd_vel
```

## 📄 License

Apache 2.0 License - Copyright Carologistics
