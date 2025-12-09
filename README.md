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
ros2 launch motor_move motor_move_launch.py namespace:=robotinobase3
```

## 📥 Action Goals

**Topic**: `/{namespace}/motor_move_action`  
**Type**: `motor_move_msgs/action/MotorMove`

**Goal Fields**:
- `motor_goal`: PoseStamped mit Position (x, y) und Quaternion-Orientierung
- `yaw_angle_deg`: Optionaler Yaw-Winkel in Grad (-360 bis 360). Wenn gesetzt, überschreibt die Quaternion-Orientierung

**Beide Methoden funktionieren**:
- **Mit Quaternion**: Setze `yaw_angle_deg` auf einen Wert außerhalb -360..360 (z.B. 9999.0) oder lasse es weg
- **Mit Grad**: Setze `yaw_angle_deg` auf einen Wert zwischen -360 und 360

**Beispiele**:

Vorwärtsbewegung mit Quaternion:
```bash
ros2 action send_goal /robotinobase3/motor_move_action motor_move_msgs/action/MotorMove "{motor_goal: {header: {frame_id: 'robotinobase3/base_link'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, yaw_angle_deg: 9999.0}" --feedback
```

Rotation um 90° mit Grad (einfacher):
```bash
ros2 action send_goal /robotinobase3/motor_move_action motor_move_msgs/action/MotorMove "{motor_goal: {header: {frame_id: 'robotinobase3/base_link'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, yaw_angle_deg: 90.0}" --feedback
```

Bewegung + Rotation mit Grad:
```bash
ros2 action send_goal /robotinobase3/motor_move_action motor_move_msgs/action/MotorMove "{motor_goal: {header: {frame_id: 'robotinobase3/base_link'}, pose: {position: {x: 1.0, y: 0.5, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, yaw_angle_deg: 45.0}" --feedback
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
ros2 launch motor_move motor_move_launch.py namespace:=robotinobase1
ros2 launch motor_move motor_move_launch.py namespace:=robotinobase2
```

## 🐛 Debugging

```bash
ros2 action list
ros2 action info /robotinobase3/motor_move_action
ros2 run tf2_tools view_frames
ros2 topic echo /robotinobase3/cmd_vel
```

## 📄 License

Apache 2.0 License - Copyright Carologistics
