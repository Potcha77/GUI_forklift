# Forklift ROS2 Integration & ESP32 Test Guide

---

## PART 1 — Adapting `forklift_bridge.py` to Your ROS2 Setup

Every section you need to change is marked with `<<<CHANGE>>>` in the file.
Here is a summary of each one:

---

### 1.1 — Localization Topic

**Where:** `_on_pose()` subscriber + the topic string

```python
self.create_subscription(
    PoseWithCovarianceStamped,
    "/amcl_pose",          # ← CHANGE THIS to your localizer topic
    self._on_pose, 10)
```

| Your localizer | Topic to use | Msg type |
|---|---|---|
| Nav2 AMCL | `/amcl_pose` | `PoseWithCovarianceStamped` |
| Robot_localization EKF | `/odometry/filtered` | `Odometry` |
| Raw odometry only | `/odom` | `Odometry` |
| Custom localizer | whatever you named it | check your node |

If you switch to `Odometry`, change the import and callback:
```python
from nav_msgs.msg import Odometry

def _on_pose(self, msg: Odometry):
    p = msg.pose.pose.position   # same from here
```

---

### 1.2 — Motor / Velocity Topic

```python
self.pub_vel = self.create_publisher(Twist, "/cmd_vel", 10)
```

Change `/cmd_vel` to whatever topic your motor controller or ESP32 serial bridge listens on.
Common alternatives: `/robot_base/cmd_vel`, `/esp32/cmd_vel`, `/diff_drive_controller/cmd_vel`

---

### 1.3 — Lift Actuator

Find this comment block:
```python
# <<<CHANGE>>> — if you have separate lift / fork actuator topics
# self.pub_lift = self.create_publisher(String, "/forklift/lift_cmd", 10)
```
Uncomment it and set your topic. Then in `_on_command`, replace:
```python
self._publish_status(f"ACK:{cmd}")
```
with:
```python
self.pub_lift.publish(String(data="UP"))   # or "DOWN"
```

---

### 1.4 — Lights (ESP32 GPIO)

Same pattern. Find:
```python
# self.pub_lights = self.create_publisher(String, "/forklift/lights", 10)
```
Uncomment it. Then in the lights handler replace the `ACK` line with:
```python
self.pub_lights.publish(String(data=raw[len("MANUAL_"):]))
```
Your ESP32 node should subscribe to that topic and set GPIO pins accordingly.

---

### 1.5 — Navigation (Autonomous GOTO)

Find `_navigate_to()`. Two options:

**Option A — nav2 action (recommended if you have Nav2 running):**
```python
from nav2_msgs.action import NavigateToPose
# in __init__:
self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

# in _navigate_to:
goal = NavigateToPose.Goal()
goal.pose.header.frame_id = "map"
goal.pose.header.stamp = self.get_clock().now().to_msg()
goal.pose.pose.position.x = x
goal.pose.pose.position.y = y
goal.pose.pose.orientation.w = 1.0
self._nav_client.send_goal_async(goal)
```

**Option B — publish to /move_base_simple/goal (simpler, no feedback):**
```python
from geometry_msgs.msg import PoseStamped
# in __init__:
self.pub_goal = self.create_publisher(PoseStamped, "/move_base_simple/goal", 10)
# in _navigate_to:
ps = PoseStamped()
ps.header.frame_id = "map"
ps.pose.position.x = x
ps.pose.position.y = y
ps.pose.orientation.w = 1.0
self.pub_goal.publish(ps)
```

---

## PART 2 — Test Running with the ESP32 Host

### 2.1 — Prerequisites

On your PC (ROS2 Humble must be sourced):
```bash
source /opt/ros/humble/setup.bash
```

Make sure your ESP32 is connected and its ROS2 serial bridge is running.
If you're using `micro-ROS` on the ESP32:
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```
Replace `/dev/ttyUSB0` with your actual port (`dmesg | tail` after plugging in to find it).

---

### 2.2 — Verify ESP32 is Talking to ROS2

In a terminal, check what topics the ESP32 is publishing/subscribing to:
```bash
ros2 topic list
ros2 topic echo /cmd_vel          # should print Twist when you move
ros2 topic echo /forklift/status  # should print status strings
```

If `/cmd_vel` doesn't appear, the ESP32 bridge isn't connecting. Check baud rate and port.

---

### 2.3 — Launch Order (3 terminals)

**Terminal 1 — Bridge node:**
```bash
source /opt/ros/humble/setup.bash
python3 forklift_bridge.py
```

**Terminal 2 — GUI:**
```bash
source /opt/ros/humble/setup.bash
python3 forklift_gui.py
```

**Terminal 3 — Monitor (optional but useful):**
```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /forklift/command   # watch what GUI sends
ros2 topic echo /cmd_vel            # watch what bridge sends to motors
```

---

### 2.4 — Testing Manual Mode Step by Step

1. Switch GUI to **MANUAL CONTROL**
2. Press **▲ (Forward)** — check Terminal 3 for:
   ```
   linear.x: 0.3   angular.z: 0.0
   ```
3. Press **◀ / ▶** — should see angular.z change
4. Press diagonal buttons — should see both linear.x and angular.z non-zero
5. Press **■ STOP** — both should go to 0.0
6. Toggle a **LIGHT** button — check `/forklift/status` echoes `ACK:MANUAL_LIGHT_FRONT_ON`
7. Press **⬆ LIFT / ⬇ LOWER** — check `/forklift/status` echoes `ACK:MANUAL_LIFT_UP`

If the ESP32 motors respond to `/cmd_vel`, you should see physical movement at step 2.

---

### 2.5 — Testing Autonomous Mode Step by Step

1. First add at least one position in **POSITION DATA ENTRY**:
   - Type name `SHELF-A`, set X=1.0, Y=2.0, click **ADD/UPDATE**
   - Or drive the forklift manually to a spot, type the name, click **USE CURRENT POS**

2. Switch to **AUTONOMOUS** mode

3. Enter a pallet entry:
   - Set position number to match (or type `SHELF-A` as prefix)
   - Enter weight e.g. 150 kg
   - Click **▶ EVALUATE & ADD** — should appear green in registry

4. Click **🚀 START OPERATION**
   - GUI log should show: `Navigating to SHELF-A...` → `Lifting pallet...` → `Completed`
   - Terminal 3 should show `GOTO:SHELF-A` on `/forklift/command`
   - Bridge should call `_navigate_to(1.0, 2.0)` — once you implement Option A or B above,
     the robot will physically navigate

5. Add a rejected entry (weight > 400 kg) — confirm it is **skipped** during START

---

### 2.6 — Common Issues

| Symptom | Cause | Fix |
|---|---|---|
| GUI shows ROS2 OFF | rclpy not found or not sourced | `source /opt/ros/humble/setup.bash` before launching |
| Motors don't move | Wrong `/cmd_vel` topic name | `ros2 topic list` and match the bridge topic |
| Position shows 0,0 always | Wrong localization topic | Change `/amcl_pose` to your actual topic |
| ESP32 not seen by ROS2 | micro-ROS agent not running | Start agent first (Step 2.1) |
| GOTO does nothing | `_navigate_to` not implemented | Follow Section 1.5 above |
