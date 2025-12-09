# Repository Struktur

## Übersicht

Dieses Repository enthält die vollständig ausgelagerten motor_move Komponenten aus dem robotino_navigation Repo.

## Verzeichnisstruktur

```
motor-move/
├── .git/                      # Git Repository
├── .gitignore                 # Git Ignore Rules
├── .colcon.meta              # Colcon Workspace Konfiguration
├── LICENSE                   # MIT License
├── README.md                 # Haupt-Dokumentation
├── SETUP.md                  # Setup-Anleitung
├── COMMIT_MESSAGE.md         # Detaillierte Commit-Message
│
├── motor_move_msgs/          # Action Messages Package
│   ├── action/
│   │   └── MotorMove.action  # Action Definition
│   ├── CMakeLists.txt        # CMake Build-Konfiguration
│   └── package.xml          # ROS2 Package Definition
│
└── motor_move/               # Action Server Package
    ├── src/
    │   ├── motor_move.cpp   # Haupt-Implementation (✅ verwendet)
    │   └── mimo.cpp         # MIMO-PID Controller (✅ verwendet)
    │
    ├── include/
    │   └── motor_move/
    │       ├── motor_move.hpp # Haupt-Header
    │       └── mimo.hpp      # MIMO-PID Header
    │
    ├── launch/
    │   └── motor_move_launch.py # Launch-Datei
    │
    ├── config/
    │   └── motor_move.yaml  # PID-Parameter Konfiguration
    │
    ├── CMakeLists.txt        # CMake Build-Konfiguration
    └── package.xml          # ROS2 Package Definition
```

## Entfernte Dateien (Toter Code)

Folgende Dateien wurden entfernt, da sie nicht verwendet werden:

- ❌ `motor_move/src/motor_move_Tim.cpp` - Alternative Implementation (nicht verwendet)
- ❌ `motor_move/src/simple_move.cpp` - Unvollständige Implementation (nicht verwendet)
- ❌ `motor_move/AUSLAGERUNG_ANLEITUNG.md` - Temporäre Anleitung (ersetzt durch README.md)
- ❌ `motor_move/src/.vscode/` - IDE-spezifische Konfiguration

## Build-Konfiguration

### CMakeLists.txt

Die `CMakeLists.txt` kompiliert nur die benötigten Dateien:

```cmake
add_executable(${PROJECT_NAME} src/motor_move.cpp src/mimo.cpp)
```

### Build-Reihenfolge

1. **motor_move_msgs** muss zuerst gebaut werden
2. **motor_move** kann dann gebaut werden

```bash
colcon build --packages-select motor_move_msgs motor_move
```

## Verwendung

Dieses Repository kann direkt als ROS2 Workspace verwendet werden:

```bash
cd motor-move
colcon build
source install/setup.bash
```

Oder in einen bestehenden Workspace integriert werden:

```bash
cd ~/ros2/your_workspace/src
git clone <repo-url> motor_move
cd ~/ros2/your_workspace
colcon build
```

## Saubere Initialisierung

Dieses Repository stellt eine saubere, ausgelagerte Version der motor_move Komponenten dar:

- ✅ Nur benötigter Code
- ✅ Keine Build-Artefakte
- ✅ Keine temporären Dateien
- ✅ Vollständige Dokumentation
- ✅ Klare Struktur
- ✅ Ready für Production

