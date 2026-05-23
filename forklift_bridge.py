#!/usr/bin/env python3
"""
forklift_bridge.py  —  ROS2 Humble companion node
─────────────────────────────────────────────────
HOW TO ADAPT THIS FILE TO YOUR OWN ROS2 SETUP:
  Search for every block marked  <<<CHANGE>>>  and replace with your
  own topic names, message types, and logic.

Subscribes:
  /forklift/pallet_data    (std_msgs/String — JSON)
  /forklift/command        (std_msgs/String)

Publishes:
  /forklift/status         (std_msgs/String)
  /forklift/map_positions  (std_msgs/String — JSON list)
  /cmd_vel                 (geometry_msgs/Twist)

Manual commands handled:
  MANUAL_FORWARD, MANUAL_BACKWARD,
  MANUAL_STRAFE_LEFT, MANUAL_STRAFE_RIGHT,
  MANUAL_ROTATE_FL, MANUAL_ROTATE_FR,
  MANUAL_ROTATE_BL, MANUAL_ROTATE_BR,
  MANUAL_STOP, MANUAL_LIFT_UP, MANUAL_LIFT_DOWN
  MANUAL_LIGHT_LEFT/RIGHT/FRONT/BACK_ON/OFF
  MODE_AUTONOMOUS, MODE_MANUAL
  SAVE_POSITION:{json}
  START_AUTONOMOUS:{json}
  GOTO:{pos_name}
"""

import json
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg   import String, Bool
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg   import Odometry

# <<<CHANGE>>> — if you use nav2 actions for navigation
# from nav2_msgs.action import NavigateToPose

WEIGHT_FULL_SPEED = 200.0
WEIGHT_WARNING    = 400.0
LINEAR_FULL       = 0.5    # m/s  autonomous
LINEAR_REDUCED    = 0.2    # m/s  autonomous heavy load
MANUAL_LINEAR     = 0.3    # m/s  manual
MANUAL_ANGULAR    = 0.8    # rad/s manual


class ForkliftBridge(Node):
    def __init__(self):
        super().__init__("forklift_bridge")

        # ── Subscriptions from GUI ────────────────────────────────────────────
        self.create_subscription(String, "/forklift/pallet_data",
                                 self._on_pallet_data, 10)
        self.create_subscription(String, "/forklift/command",
                                 self._on_command, 10)

        # <<<CHANGE>>> — subscribe to YOUR localization topic
        # Default is /amcl_pose (nav2 AMCL). Replace with your topic name
        # and message type if you use a different localizer (e.g. /odom, /ekf_pose)
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",                          # ← YOUR localization topic
            self._on_pose, 10)

        # <<<CHANGE>>> — if you publish odometry separately
        # self.create_subscription(Odometry, "/odom", self._on_odom, 10)

        # ── Publishers ───────────────────────────────────────────────────────
        self.pub_status  = self.create_publisher(String, "/forklift/status",        10)
        self.pub_map_pos = self.create_publisher(String, "/forklift/map_positions",  10)

        # <<<CHANGE>>> — replace /cmd_vel with your motor controller topic
        # e.g. /robot_base/cmd_vel, /esp32/cmd_vel, /diff_drive/cmd_vel
        self.pub_vel = self.create_publisher(Twist, "/cmd_vel", 10)   # ← YOUR cmd topic

        # <<<CHANGE>>> — if you have separate lift / fork actuator topics
        # self.pub_lift = self.create_publisher(String, "/forklift/lift_cmd", 10)

        # <<<CHANGE>>> — if you have GPIO / light topics for the ESP32
        # self.pub_lights = self.create_publisher(String, "/forklift/lights", 10)

        # <<<CHANGE>>> — nav2 action client for autonomous navigation
        # Uncomment and replace with your navigation action server name
        # self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.create_timer(10.0, self._publish_map_positions)

        self.current_weight  = 0.0
        self.current_mode    = "AUTONOMOUS"
        self.current_pose    = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.position_db: dict[str, dict] = {}   # name → {x, y}

        self.get_logger().info("ForkliftBridge ready.")

    # ── Localization callback ─────────────────────────────────────────────────
    def _on_pose(self, msg: PoseWithCovarianceStamped):
        """
        <<<CHANGE>>> — adapt if your localizer publishes a different msg type.
        e.g. for Odometry:  msg.pose.pose.position
        for PoseStamped:    msg.pose.position
        """
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # Convert quaternion → yaw
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.current_pose = {"x": round(p.x, 3),
                             "y": round(p.y, 3),
                             "yaw": round(math.degrees(yaw), 1)}
        # Forward to GUI
        self.pub_status.publish(
            String(data=f"POSE:{p.x:.3f},{p.y:.3f}"))

    # ── Pallet data callback ──────────────────────────────────────────────────
    def _on_pallet_data(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("Invalid JSON on /forklift/pallet_data")
            return
        weight = float(data.get("weight_kg", 0))
        pos    = data.get("position", "?")
        self.current_weight = weight
        self.get_logger().info(f"Pallet: {pos} = {weight} kg")
        if weight <= WEIGHT_FULL_SPEED:
            self._set_speed(LINEAR_FULL)
            self._publish_status(f"OK:{pos}:{weight}kg:FULL_SPEED")
        elif weight <= WEIGHT_WARNING:
            self._set_speed(LINEAR_REDUCED)
            self._publish_status(f"WARN:{pos}:{weight}kg:REDUCED_SPEED")
        else:
            self._set_vel(0, 0)
            self._publish_status(f"ERROR:{pos}:{weight}kg:REJECTED")

    # ── Command dispatcher ────────────────────────────────────────────────────
    def _on_command(self, msg: String):
        raw = msg.data
        cmd = raw.upper().split(":")[0]
        self.get_logger().info(f"Command: {raw}")

        # ── Mode ──
        if cmd == "MODE_AUTONOMOUS":
            self.current_mode = "AUTONOMOUS"
            self._publish_status("MODE:AUTONOMOUS")
        elif cmd == "MODE_MANUAL":
            self.current_mode = "MANUAL"
            self._publish_status("MODE:MANUAL")

        # ── Emergency / nav ──
        elif cmd == "E_STOP":
            self._set_vel(0, 0)
            self._publish_status("EMERGENCY_STOP")
        elif cmd == "PAUSE":
            self._set_vel(0, 0)
            self._publish_status("PAUSED")
        elif cmd == "RESUME":
            spd = LINEAR_FULL if self.current_weight <= WEIGHT_FULL_SPEED else LINEAR_REDUCED
            self._set_speed(spd)
            self._publish_status("RESUMED")
        elif cmd == "NAVIGATE_HOME":
            self._navigate_to(0.0, 0.0)
            self._publish_status("NAVIGATING_HOME")
        elif cmd == "REDUCE_SPEED":
            self._set_speed(LINEAR_REDUCED)

        # ── Autonomous operation ──
        elif cmd == "START_AUTONOMOUS":
            try:
                targets = json.loads(raw.split(":", 1)[1])
                self._publish_status(f"AUTO_START:{len(targets)}_targets")
            except Exception as e:
                self.get_logger().warn(f"START_AUTONOMOUS parse error: {e}")

        elif cmd == "GOTO":
            pos_name = raw.split(":", 1)[1]
            if pos_name in self.position_db:
                d = self.position_db[pos_name]
                self._navigate_to(d["x"], d["y"])
                self._publish_status(f"NAVIGATING:{pos_name}")
            else:
                self.get_logger().warn(f"Unknown position: {pos_name}")
                self._publish_status(f"UNKNOWN_POSITION:{pos_name}")

        # ── Position database ──
        elif cmd == "SAVE_POSITION":
            try:
                d = json.loads(raw.split(":", 1)[1])
                self.position_db[d["name"]] = {"x": d["x"], "y": d["y"]}
                self.get_logger().info(f"Saved position: {d}")
                self._publish_map_positions()
            except Exception as e:
                self.get_logger().warn(f"SAVE_POSITION error: {e}")

        # ── Manual drive ──
        elif cmd == "MANUAL_FORWARD":
            self._set_vel(MANUAL_LINEAR, 0)
        elif cmd == "MANUAL_BACKWARD":
            self._set_vel(-MANUAL_LINEAR, 0)
        elif cmd == "MANUAL_STRAFE_LEFT":
            self._set_vel(0, MANUAL_ANGULAR)
        elif cmd == "MANUAL_STRAFE_RIGHT":
            self._set_vel(0, -MANUAL_ANGULAR)
        # Diagonal rotations: combine linear + angular
        elif cmd == "MANUAL_ROTATE_FL":   # forward + turn left
            self._set_vel(MANUAL_LINEAR * 0.6,  MANUAL_ANGULAR)
        elif cmd == "MANUAL_ROTATE_FR":   # forward + turn right
            self._set_vel(MANUAL_LINEAR * 0.6, -MANUAL_ANGULAR)
        elif cmd == "MANUAL_ROTATE_BL":   # backward + turn left
            self._set_vel(-MANUAL_LINEAR * 0.6,  MANUAL_ANGULAR)
        elif cmd == "MANUAL_ROTATE_BR":   # backward + turn right
            self._set_vel(-MANUAL_LINEAR * 0.6, -MANUAL_ANGULAR)
        elif cmd == "MANUAL_STOP":
            self._set_vel(0, 0)

        # ── Lift ──
        elif cmd in ("MANUAL_LIFT_UP", "MANUAL_LIFT_DOWN"):
            # <<<CHANGE>>> — publish to your actual lift actuator topic
            # self.pub_lift.publish(String(data=cmd))
            self._publish_status(f"ACK:{cmd}")

        # ── Lights ──
        elif cmd.startswith("MANUAL_LIGHT_"):
            # <<<CHANGE>>> — forward to your ESP32 GPIO/light topic
            # self.pub_lights.publish(String(data=raw[7:]))  # strip MANUAL_
            self._publish_status(f"ACK:{cmd}")

    # ── Navigation helper ─────────────────────────────────────────────────────
    def _navigate_to(self, x: float, y: float):
        """
        <<<CHANGE>>> — replace with your actual navigation call.

        Option A — nav2 action (recommended):
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = "map"
            goal.pose.pose.position.x = x
            goal.pose.pose.position.y = y
            goal.pose.pose.orientation.w = 1.0
            self._nav_client.send_goal_async(goal)

        Option B — publish a goal topic directly:
            from geometry_msgs.msg import PoseStamped
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            self.pub_goal.publish(ps)    # ← your /move_base_simple/goal or similar
        """
        self.get_logger().info(f"Navigate to ({x:.3f}, {y:.3f})  ← IMPLEMENT ME")

    # ── Velocity helpers ──────────────────────────────────────────────────────
    def _set_speed(self, linear: float):
        self._set_vel(linear, 0)

    def _set_vel(self, linear: float, angular: float):
        t = Twist()
        t.linear.x  = float(linear)
        t.angular.z = float(angular)
        self.pub_vel.publish(t)

    def _publish_status(self, status: str):
        self.pub_status.publish(String(data=status))

    def _publish_map_positions(self):
        names = list(self.position_db.keys())
        self.pub_map_pos.publish(String(data=json.dumps(names)))


def main(args=None):
    rclpy.init(args=args)
    node = ForkliftBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
