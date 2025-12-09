# Anleitung: motor_move in eigenes Repo auslagern

## Übersicht

`motor_move` ist ein **standalone Action Server** - es greift **NICHT** auf andere Nodes zu, um Goals zu bekommen. Stattdessen:
- ✅ **Passiv**: Wartet auf Action Clients, die Goals senden
- ✅ **Unabhängig**: Keine direkten Node-Abhängigkeiten
- ✅ **Kommunikation**: Nur über ROS2 Topics/Actions

## Abhängigkeiten

### Erforderliche Packages (müssen mit ausgelagert werden):

1. **motor_move_msgs** ⚠️ **MUSS mit ausgelagert werden**
   - Definiert die Action Messages
   - Ohne dieses Package funktioniert motor_move nicht

2. **motor_move** (Haupt-Package)
   - Action Server Implementation
   - MIMO PID Controller

### Externe ROS2 Dependencies (Standard-Packages):

- `rclcpp` / `rclcpp_action` (ROS2 Core)
- `geometry_msgs` (Standard ROS2 Messages)
- `tf2` / `tf2_ros` / `tf2_geometry_msgs` (TF2 für Transformationen)
- `nav2_common` (nur für Launch-Datei, optional)

### System-Abhängigkeiten:

- `Eigen3` (für Matrix-Operationen)

## Kommunikation

### Input (was motor_move empfängt):

1. **Action Goals** über `/motor_move_action`
   - Typ: `motor_move_msgs/action/MotorMove`
   - Von: Jeder Action Client (CLI, andere Nodes, etc.)

2. **TF Transforms** (über TF2)
   - Benötigt: `odom_frame` und `base_frame`
   - Von: TF2 Tree (normalerweise von Odometry/SLAM Nodes)

### Output (was motor_move publiziert):

1. **cmd_vel** (Twist Messages)
   - Topic: `/{namespace}/cmd_vel`
   - Typ: `geometry_msgs/msg/Twist`
   - An: Robot Controller (z.B. robotino_driver)

## Schritt-für-Schritt Auslagerung

### 1. Neues Git-Repo erstellen

```bash
# Auf GitHub/GitLab/etc. neues Repo erstellen
# Dann lokal klonen:
cd ~/ros2
git clone <neues-repo-url> motor_move_standalone
cd motor_move_standalone
```

### 2. Packages kopieren

```bash
# motor_move_msgs muss ZUERST kopiert werden (motor_move hängt davon ab)
cp -r ~/ros2/robotino_navigation_ws/src/robotino_navigation/motor_move_msgs .
cp -r ~/ros2/robotino_navigation_ws/src/robotino_navigation/motor_move .
```

### 3. Repo-Struktur anpassen

```
motor_move_standalone/
├── motor_move_msgs/          # Messages Package (MUSS zuerst gebaut werden)
│   ├── action/
│   │   └── MotorMove.action
│   ├── CMakeLists.txt
│   └── package.xml
├── motor_move/                # Action Server Package
│   ├── src/
│   ├── include/
│   ├── launch/
│   ├── config/
│   ├── CMakeLists.txt
│   └── package.xml
├── README.md
└── LICENSE
```

### 4. package.xml anpassen

**motor_move/package.xml** - Maintainer/License anpassen:
```xml
<maintainer email="deine@email.de">Dein Name</maintainer>
<license>MIT</license>  <!-- oder deine Wahl -->
```

**motor_move_msgs/package.xml** - Gleiche Anpassungen

### 5. README.md erstellen

```markdown
# motor_move

Standalone ROS2 Action Server für präzise Roboter-Bewegungen mit MIMO-PID Controller.

## Features

- MIMO-PID Controller für x, y, yaw
- Action-basierte Steuerung
- Namespace-Support für Multi-Robot-Setups
- Konfigurierbare PID-Parameter über YAML

## Installation

```bash
cd ~/ros2/your_workspace/src
git clone <repo-url> motor_move_standalone
cd ..
colcon build --packages-select motor_move_msgs motor_move
source install/setup.bash
```

## Usage

```bash
# Start Action Server
ros2 launch motor_move motor_move_launch.py namespace:=robotinobase3

# Send Goal
ros2 action send_goal /robotinobase3/motor_move_action motor_move_msgs/action/MotorMove \
  "{motor_goal: {header: {frame_id: 'robotinobase3/base_link'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
```

## Dependencies

- ROS2 (Jazzy)
- Eigen3
- Standard ROS2 Packages (rclcpp, geometry_msgs, tf2)
```

### 6. Build-System anpassen (optional)

Falls du ein monorepo mit mehreren Packages hast, kannst du ein Root-`CMakeLists.txt` erstellen:

```cmake
cmake_minimum_required(VERSION 3.8)
project(motor_move_standalone)

# Packages werden einzeln gebaut, kein Root-Build nötig
```

### 7. Git Setup

```bash
cd ~/ros2/motor_move_standalone
git add .
git commit -m "Initial commit: motor_move standalone package"
git push origin main
```

### 8. In neuem Workspace verwenden

```bash
cd ~/ros2/neuer_workspace/src
git clone <repo-url> motor_move_standalone
cd ~/ros2/neuer_workspace
colcon build --packages-select motor_move_msgs motor_move
source install/setup.bash
```

## Wichtige Hinweise

### ⚠️ Build-Reihenfolge

**motor_move_msgs** MUSS vor **motor_move** gebaut werden:
```bash
colcon build --packages-select motor_move_msgs motor_move
# Oder einzeln:
colcon build --packages-select motor_move_msgs
colcon build --packages-select motor_move
```

### ⚠️ TF2 Abhängigkeit

motor_move benötigt einen laufenden TF2 Tree mit:
- `{namespace}/odom` Frame
- `{namespace}/base_link` Frame

Diese werden normalerweise von anderen Nodes bereitgestellt (Odometry, SLAM, etc.).

### ⚠️ cmd_vel Subscriber

motor_move publiziert auf `cmd_vel`, aber es gibt keinen direkten Subscriber. Der Robot Controller (z.B. robotino_driver) muss diesen Topic abonnieren.

## Testing

```bash
# 1. TF2 Tree starten (z.B. mit robotino_driver oder Simulation)
# 2. motor_move starten
ros2 launch motor_move motor_move_launch.py namespace:=test_robot

# 3. Action Server prüfen
ros2 action list | grep motor_move

# 4. Test-Goal senden
ros2 action send_goal /test_robot/motor_move_action motor_move_msgs/action/MotorMove \
  "{motor_goal: {header: {frame_id: 'test_robot/base_link'}, pose: {position: {x: 0.5, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

## Optional: CI/CD Setup

Füge GitHub Actions oder GitLab CI hinzu für automatisches Build-Testing.

