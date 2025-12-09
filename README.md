# motor_move

Standalone ROS2 Action Server für präzise Roboter-Bewegungen mit MIMO-PID Controller.

## 📦 Was beinhaltet dieses Repo?

Dieses Repository enthält zwei ROS2 Packages für die präzise Steuerung von Robotern:

### 1. `motor_move_msgs` - Action Message Definitions
- Definiert die Action Messages für die Kommunikation mit dem motor_move Action Server
- Enthält: `MotorMove.action` mit Goal, Result und Feedback Definitionen
- **Muss zuerst gebaut werden** (motor_move hängt davon ab)

### 2. `motor_move` - Action Server Implementation
- ROS2 Action Server für präzise Positionssteuerung
- MIMO-PID Controller (Multi-Input Multi-Output) für x, y und yaw
- Unterstützt Namespace-basierte Multi-Robot-Setups
- Konfigurierbare PID-Parameter über YAML

## 🔌 Einbindung in einen ROS2 Workspace

### Schritt 1: Repository klonen

```bash
cd ~/ros2/your_workspace/src
git clone <repo-url> motor_move
```

### Schritt 2: Dependencies installieren

```bash
# ROS2 Packages (sollten bereits installiert sein)
# - rclcpp, rclcpp_action
# - geometry_msgs
# - tf2, tf2_ros, tf2_geometry_msgs
# - nav2_common (für Launch-Datei)

# System Dependencies
sudo apt-get install libeigen3-dev
```

### Schritt 3: Workspace bauen

```bash
cd ~/ros2/your_workspace

# Wichtig: motor_move_msgs muss vor motor_move gebaut werden
colcon build --packages-select motor_move_msgs motor_move

# Oder alles auf einmal (colcon erkennt Abhängigkeiten):
colcon build

# Workspace aktivieren
source install/setup.bash
```

### Schritt 4: Verwendung

```bash
# Action Server starten
ros2 launch motor_move motor_move_launch.py namespace:=robotinobase3
```

## 📥 Inputs (Was motor_move benötigt)

### 1. Action Goals (über ROS2 Actions)

**Topic**: `/{namespace}/motor_move_action`  
**Type**: `motor_move_msgs/action/MotorMove`

**Goal Structure**:
```yaml
motor_goal:
  header:
    frame_id: "{namespace}/base_link"  # Frame des Ziel-Pose
  stamp: {sec: 0, nanosec: 0}          # Optional, wird ignoriert
  pose:
    position:
      x: 1.0    # Ziel-Position in x-Richtung (Meter)
      y: 0.0    # Ziel-Position in y-Richtung (Meter)
      z: 0.0    # Wird ignoriert (2D-Bewegung)
    orientation:
      x: 0.0    # Quaternion x
      y: 0.0    # Quaternion y
      z: 0.0    # Quaternion z (für Yaw-Rotation)
      w: 1.0    # Quaternion w
```

**Beispiel - Vorwärtsbewegung (1m)**:
```bash
ros2 action send_goal /robotinobase3/motor_move_action motor_move_msgs/action/MotorMove \
  "{motor_goal: {header: {frame_id: 'robotinobase3/base_link'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
```

**Beispiel - Reine Yaw-Rotation (45°)**:
```bash
ros2 action send_goal /robotinobase3/motor_move_action motor_move_msgs/action/MotorMove \
  "{motor_goal: {header: {frame_id: 'robotinobase3/base_link'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.382683, w: 0.92388}}}"
```

### 2. TF2 Transforms (über TF2 Tree)

**Benötigte Frames**:
- `{namespace}/odom` - Odometry Frame (global, fix)
- `{namespace}/base_link` - Robot Base Frame (bewegt sich relativ zu odom)

**Quelle**: Diese Frames werden normalerweise von anderen Nodes bereitgestellt:
- Odometry Node (z.B. `robotino_driver`)
- SLAM Node (z.B. `slam_toolbox`)
- Simulation (z.B. Gazebo/Webots)

**Verwendung**: motor_move transformiert die Ziel-Pose vom `base_link` Frame ins `odom` Frame, um die relative Bewegung zu berechnen.

### 3. Konfigurationsparameter (YAML)

**Datei**: `motor_move/config/motor_move.yaml`

**Parameter**:
```yaml
motor_move:
  ros__parameters:
    Kp: [1.8, 0.0, 0.0, 0.0, 1.8, 0.0, 0.0, 0.0, 1.8]  # Proportional gains [x, y, yaw]
    Ki: [0.38, 0.0, 0.0, 0.0, 0.38, 0.0, 0.0, 0.0, 0.38]  # Integral gains
    Kd: [0.2, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.2]    # Derivative gains
```

**Matrix-Format**: Die Parameter werden als 3x3 Matrizen interpretiert (Row-Major):
```
[Kp_x, 0, 0]
[0, Kp_y, 0]
[0, 0, Kp_yaw]
```

## 📤 Outputs (Was motor_move publiziert)

### 1. cmd_vel (Twist Messages)

**Topic**: `/{namespace}/cmd_vel`  
**Type**: `geometry_msgs/msg/Twist`  
**Rate**: ~15 Hz (während Bewegung)

**Struktur**:
```yaml
linear:
  x: 0.5    # Vorwärts/Rückwärts Geschwindigkeit (m/s)
  y: 0.0    # Links/Rechts Geschwindigkeit (m/s)
  z: 0.0    # Wird ignoriert
angular:
  x: 0.0    # Wird ignoriert
  y: 0.0    # Wird ignoriert
  z: 0.3    # Rotationsgeschwindigkeit (rad/s)
```

**Ziel**: Diese Nachrichten werden von einem Robot Controller abonniert (z.B. `robotino_driver`), der die Geschwindigkeitskommandos in Motorbefehle umwandelt.

### 2. Action Feedback

**Topic**: `/{namespace}/motor_move_action/_action/feedback`  
**Type**: `motor_move_msgs/action/MotorMove_Feedback`

**Struktur**:
```yaml
distance_to_target: 0.15  # Aktuelle Distanz zum Ziel (Meter)
```

**Rate**: ~15 Hz (während Bewegung)

### 3. Action Result

**Topic**: `/{namespace}/motor_move_action/_action/result`  
**Type**: `motor_move_msgs/action/MotorMove_Result`

**Struktur**:
```yaml
success: true  # true wenn Ziel erreicht, false bei Fehler
```

**Wann**: Wird einmalig gesendet, wenn:
- Ziel erreicht wurde (distance < 0.05m und yaw < 0.087rad)
- Action abgebrochen wurde
- Fehler aufgetreten ist

## 🔄 Interaktions-Fluss

```
┌─────────────┐
│ Action      │
│ Client      │───[Goal]───►┌──────────────┐
│ (z.B. CLI   │             │ motor_move   │
│  oder Node) │◄──[Feedback]│ Action       │
│             │             │ Server       │
└─────────────┘             └──────┬───────┘
                                    │
                                    │ cmd_vel
                                    ▼
                            ┌──────────────┐
                            │ Robot        │
                            │ Controller   │
                            │ (z.B.        │
                            │ robotino_    │
                            │ driver)      │
                            └──────┬───────┘
                                   │
                                   │ Motorbefehle
                                   ▼
                            ┌──────────────┐
                            │ Robot        │
                            │ Hardware     │
                            └──────┬───────┘
                                   │
                                   │ Odometry
                                   ▼
                            ┌──────────────┐
                            │ TF2 Tree     │
                            │ (odom,       │
                            │  base_link)  │
                            └──────┬───────┘
                                   │
                                   │ TF Transforms
                                   ▼
                            ┌──────────────┐
                            │ motor_move   │
                            │ (berechnet   │
                            │  Error)     │
                            └──────────────┘
```

## 🎯 Verwendungsbeispiele

### Beispiel 1: Einfache Vorwärtsbewegung

```bash
# Terminal 1: Action Server starten
ros2 launch motor_move motor_move_launch.py namespace:=robotinobase3

# Terminal 2: Goal senden
ros2 action send_goal /robotinobase3/motor_move_action motor_move_msgs/action/MotorMove \
  "{motor_goal: {header: {frame_id: 'robotinobase3/base_link'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}" \
  --feedback
```

### Beispiel 2: Python Action Client

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from motor_move_msgs.action import MotorMove
from geometry_msgs.msg import PoseStamped

class MotorMoveClient(Node):
    def __init__(self):
        super().__init__('motor_move_client')
        self.action_client = ActionClient(self, MotorMove, '/robotinobase3/motor_move_action')
    
    def send_goal(self, x, y, yaw_quat_z=0.0, yaw_quat_w=1.0):
        goal_msg = MotorMove.Goal()
        goal_msg.motor_goal.header.frame_id = 'robotinobase3/base_link'
        goal_msg.motor_goal.pose.position.x = float(x)
        goal_msg.motor_goal.pose.position.y = float(y)
        goal_msg.motor_goal.pose.orientation.z = float(yaw_quat_z)
        goal_msg.motor_goal.pose.orientation.w = float(yaw_quat_w)
        
        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
    
    def feedback_callback(self, feedback_msg):
        distance = feedback_msg.feedback.distance_to_target
        self.get_logger().info(f'Distance to target: {distance:.3f}m')
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
    
    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Goal finished: success={result.success}')

def main():
    rclpy.init()
    client = MotorMoveClient()
    client.send_goal(x=1.0, y=0.0)
    rclpy.spin(client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Beispiel 3: C++ Action Client

Siehe Beispiel-Code in der Dokumentation oder erstelle einen eigenen Client basierend auf `rclcpp_action::Client`.

## ⚙️ Konfiguration

### PID-Parameter anpassen

1. **YAML-Datei bearbeiten**:
```bash
nano ~/ros2/your_workspace/src/motor_move/motor_move/config/motor_move.yaml
```

2. **Parameter anpassen** (z.B. höherer Kp für y-Achse):
```yaml
motor_move:
  ros__parameters:
    Kp: [1.8, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 1.8]  # Höherer Kp für y
    Ki: [0.38, 0.0, 0.0, 0.0, 0.4, 0.0, 0.0, 0.0, 0.38]
    Kd: [0.2, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.2]
```

3. **Neu bauen und starten**:
```bash
colcon build --packages-select motor_move
source install/setup.bash
ros2 launch motor_move motor_move_launch.py namespace:=robotinobase3
```

### Namespace-Konfiguration

Für Multi-Robot-Setups:

```bash
# Robot 1
ros2 launch motor_move motor_move_launch.py namespace:=robotinobase1

# Robot 2
ros2 launch motor_move motor_move_launch.py namespace:=robotinobase2

# Robot 3
ros2 launch motor_move motor_move_launch.py namespace:=robotinobase3
```

Jeder Robot hat dann seinen eigenen Action Server:
- `/robotinobase1/motor_move_action`
- `/robotinobase2/motor_move_action`
- `/robotinobase3/motor_move_action`

## 🐛 Debugging

### Action Server Status prüfen

```bash
# Alle Actions auflisten
ros2 action list

# Spezifische Action Info
ros2 action info /robotinobase3/motor_move_action

# Action Type prüfen
ros2 interface show motor_move_msgs/action/MotorMove
```

### Logs anzeigen

```bash
# Während motor_move läuft, werden Debug-Logs ausgegeben:
# - Error matrix (x, y, yaw)
# - PID Output (x, y, yaw)
# - Distance to target
# - Yaw error
```

### TF2 Frames prüfen

```bash
# TF Tree anzeigen
ros2 run tf2_tools view_frames

# Transform prüfen
ros2 run tf2_ros tf2_echo robotinobase3/odom robotinobase3/base_link
```

## 📋 Voraussetzungen

### System Requirements

- **ROS2**: Jazzy (empfohlen) oder neuer
- **Eigen3**: `sudo apt-get install libeigen3-dev`
- **CMake**: Version 3.8 oder höher

### ROS2 Dependencies

Diese sollten bereits mit ROS2 installiert sein:
- `rclcpp` / `rclcpp_action`
- `geometry_msgs`
- `tf2` / `tf2_ros` / `tf2_geometry_msgs`
- `nav2_common` (nur für Launch-Datei)

### Laufzeit-Voraussetzungen

1. **TF2 Tree**: Muss laufen mit:
   - `{namespace}/odom` Frame
   - `{namespace}/base_link` Frame
   
   Normalerweise bereitgestellt durch:
   - Odometry Node (z.B. `robotino_driver`)
   - SLAM Node
   - Simulation

2. **cmd_vel Subscriber**: Ein Robot Controller muss `/{namespace}/cmd_vel` abonnieren:
   - `robotino_driver`
   - `turtlebot3_teleop`
   - Oder ähnliche Controller

## 🔧 Troubleshooting

### Problem: "Transform error"

**Ursache**: TF2 Frames fehlen oder sind nicht verfügbar.

**Lösung**:
```bash
# TF Tree prüfen
ros2 run tf2_tools view_frames

# Prüfen ob Frames existieren
ros2 topic echo /tf_static
```

### Problem: "Action Server not found"

**Ursache**: motor_move Node läuft nicht oder Namespace falsch.

**Lösung**:
```bash
# Prüfen ob Node läuft
ros2 node list | grep motor_move

# Action Server prüfen
ros2 action list | grep motor_move
```

### Problem: Roboter bewegt sich nicht

**Ursache**: cmd_vel wird nicht abonniert oder PID-Parameter zu niedrig.

**Lösung**:
```bash
# Prüfen ob cmd_vel publiziert wird
ros2 topic echo /robotinobase3/cmd_vel

# PID-Parameter erhöhen (siehe Konfiguration)
```

## 📚 Weitere Informationen

- **MIMO-PID Controller**: Verwendet 3x3 Matrizen für gekoppelte Regelung
- **Action Server**: Asynchrone Ausführung mit Feedback
- **Namespace Support**: Multi-Robot-Setups möglich

## 📄 License

Apache 2.0 License - Copyright Carologistics

## 👥 Maintainer

- Original: Carologistics
- Maintainer: jacob.weyer@rwth-aachen.de
