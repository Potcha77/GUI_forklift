# Forklift GUI — Setup & Usage

## Files
| File | Purpose |
|------|---------|
| `forklift_gui.py`    | Main GUI (Python 3.9+, Tkinter) — touch-screen optimised |
| `forklift_bridge.py` | ROS2 Humble companion node |

---

## Two Modes

### 🤖 AUTONOMOUS MODE
Enter pallet position (POS-XX) and weight via the on-screen numpad. The robot navigates autonomously based on pallet data sent over ROS2.

### 🕹 MANUAL CONTROL MODE
Drive the forklift directly using the on-screen D-pad:
- **▲ ▼ ◀ ▶** — forward / backward / strafe
- **↺ ↻** — rotate left / right
- **■ STOP** — immediate stop
- **HORN / LIFT UP / LOWER** — accessory controls

Hold any arrow button for continuous movement. Release to stop.

---

## Touch Input
- **Position number**: tap the number display to open a numpad — only the number after `POS-` is editable.
- **Weight**: tap the kg display to open a numpad.
- All dialogs (confirm, toast notifications) are full touch-target sized.

---

## Requirements

### Python only (simulation)
```bash
python3 forklift_gui.py
```

### With ROS2 Humble
```bash
source /opt/ros/humble/setup.bash
python3 forklift_gui.py
```

---

## Running with ROS2

### Terminal 1 — Nav stack
```bash
source /opt/ros/humble/setup.bash
ros2 launch nav2_bringup navigation_launch.py
```

### Terminal 2 — Bridge
```bash
source /opt/ros/humble/setup.bash
python3 forklift_bridge.py
```

### Terminal 3 — GUI
```bash
source /opt/ros/humble/setup.bash
python3 forklift_gui.py
```

---

## ROS2 Topics

| Topic | Type | Direction | Purpose |
|-------|------|-----------|---------|
| `/forklift/pallet_data` | `String` (JSON) | GUI → ROS2 | Pallet entry on ADD |
| `/forklift/command`     | `String`        | GUI → ROS2 | All commands (mode, nav, manual) |
| `/forklift/current_weight` | `Float32`    | GUI → ROS2 | Live weight |
| `/forklift/status`      | `String`        | ROS2 → GUI | Status feedback |
| `/forklift/map_positions` | `String` (JSON) | ROS2 → GUI | Known positions |
| `/amcl_pose`            | `PoseWithCovarianceStamped` | ROS2 → GUI | Robot localisation |
| `/cmd_vel`              | `Twist`         | bridge → Nav | Velocity control |

### Mode commands (GUI → ROS2 on /forklift/command)
```
MODE_AUTONOMOUS
MODE_MANUAL
MANUAL_FORWARD / MANUAL_BACKWARD
MANUAL_STRAFE_LEFT / MANUAL_STRAFE_RIGHT
MANUAL_ROTATE_LEFT / MANUAL_ROTATE_RIGHT
MANUAL_STOP
MANUAL_HORN / MANUAL_LIFT_UP / MANUAL_LIFT_DOWN
```

---

## Weight Zones

| Range | Status | Speed |
|-------|--------|-------|
| 0 – 200 kg   | ✔ ACCEPTED     | Full speed    |
| 200 – 400 kg | ⚠ WARNING      | Reduced speed |
| > 400 kg     | ✖ REJECTED     | Stopped       |
