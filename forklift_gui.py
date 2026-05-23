#!/usr/bin/env python3
"""
Forklift Pallet Weight Management GUI
Touch-Screen Optimised — ESP32 / ROS2 Hybrid
Two Modes: AUTONOMOUS | MANUAL CONTROL
"""

import tkinter as tk
from tkinter import ttk, font
import threading
import json
import time
import os
from datetime import datetime
from typing import Optional

# ── Persistence ───────────────────────────────────────────────────────────────
POSITIONS_FILE = os.path.join(os.path.expanduser("~"), "forklift_positions.json")

# ── ROS2 optional import ──────────────────────────────────────────────────────
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String, Float32
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
    from nav_msgs.msg import OccupancyGrid
    from std_srvs.srv import Trigger
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

# ── Constants ─────────────────────────────────────────────────────────────────
WEIGHT_FULL_SPEED = 50
WEIGHT_WARNING    = 100
MAX_POSITIONS     = 20

# ── Manual control speeds  ◄─── EDIT THESE to change button velocities ────────
# linear_x : forward / backward  (m/s)
# linear_y : strafe left / right (m/s)  — only for omni/mecanum robots
# angular_z: rotation            (rad/s)
SPEED_LINEAR   = 0.3   # forward / backward
SPEED_STRAFE   = 0.3   # lateral (left / right)
SPEED_ANGULAR  = 0.5   # rotation (rad/s)
SPEED_DIAG_LIN = 0.2   # diagonal forward component
SPEED_DIAG_ANG = 0.5   # diagonal angular component

# Colour palette — industrial dark
BG_DARK   = "#0D1117"
BG_PANEL  = "#161B22"
BG_CARD   = "#1C2128"
BG_INPUT  = "#21262D"
ACCENT    = "#F78166"
ACCENT2   = "#79C0FF"
GREEN     = "#3FB950"
YELLOW    = "#D29922"
RED       = "#F85149"
TEXT_PRI  = "#E6EDF3"
TEXT_SEC  = "#8B949E"
BORDER    = "#30363D"

MODE_AUTO   = "AUTONOMOUS"
MODE_MANUAL = "MANUAL"
MODE_DATA   = "DATA_ENTRY"


# ═══════════════════════════════════════════════════════════════════════════════
#  ROS2 Node
# ═══════════════════════════════════════════════════════════════════════════════
class ForkliftROS2Node(Node if ROS2_AVAILABLE else object):
    def __init__(self, gui_callback):
        if not ROS2_AVAILABLE:
            return
        super().__init__("forklift_gui_node")
        self.gui_callback = gui_callback

        self.pub_pallet   = self.create_publisher(String,      "/forklift/pallet_data",   10)
        self.pub_command  = self.create_publisher(String,      "/forklift/command",        10)
        self.pub_weight   = self.create_publisher(Float32,     "/forklift/current_weight", 10)
        self.pub_cmd_vel  = self.create_publisher(Twist,       "/cmd_vel",                 10)
        self.pub_goal     = self.create_publisher(PoseStamped, "/forklift/goal_pose",      10)

        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                                 self._on_pose, 10)
        self.create_subscription(String, "/forklift/status",
                                 self._on_status, 10)
        self.create_subscription(String, "/forklift/map_positions",
                                 self._on_map_positions, 10)
        self.get_logger().info("ForkliftGUI node started.")

    def _on_pose(self, msg):
        p = msg.pose.pose.position
        self.gui_callback({"type": "pose", "x": round(p.x,3), "y": round(p.y,3)})

    def _on_status(self, msg):
        self.gui_callback({"type": "status", "value": msg.data})

    def _on_map_positions(self, msg):
        try:
            self.gui_callback({"type": "map_positions",
                                "positions": json.loads(msg.data)})
        except json.JSONDecodeError:
            pass

    def publish_pallet_entry(self, position: str, weight: float):
        payload = json.dumps({"position": position, "weight_kg": weight,
                               "timestamp": datetime.now().isoformat()})
        self.pub_pallet.publish(String(data=payload))
        self.pub_weight.publish(Float32(data=weight))

    def publish_command(self, cmd: str):
        self.pub_command.publish(String(data=cmd))

    def publish_twist(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x  = float(linear_x)
        msg.linear.y  = float(linear_y)
        msg.angular.z = float(angular_z)
        self.pub_cmd_vel.publish(msg)

    def publish_goal_pose(self, x: float, y: float):
        """Publish a goal pose on /forklift/goal_pose (PoseStamped)."""
        from rclpy.clock import Clock
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = Clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0   # identity quaternion — facing +X
        self.pub_goal.publish(msg)


# ═══════════════════════════════════════════════════════════════════════════════
#  Touch Numpad Popup
# ═══════════════════════════════════════════════════════════════════════════════
class NumpadPopup(tk.Toplevel):
    """Full-screen-friendly numpad for touch entry."""

    def __init__(self, parent, title, initial_value, callback, integer_only=False):
        super().__init__(parent)
        self.callback = callback
        self.integer_only = integer_only

        self.configure(bg=BG_DARK)
        self.title(title)
        self.resizable(False, False)

        # Centre on parent
        pw = parent.winfo_width()
        ph = parent.winfo_height()
        px = parent.winfo_rootx()
        py = parent.winfo_rooty()
        w, h = 360, 520
        self.geometry(f"{w}x{h}+{px+(pw-w)//2}+{py+(ph-h)//2}")
        self.update_idletasks()
        self.grab_set()
        self.lift()
        self.focus_force()

        fB  = font.Font(family="Courier", size=26, weight="bold")
        fM  = font.Font(family="Courier", size=16, weight="bold")
        fSm = font.Font(family="Courier", size=12)

        tk.Label(self, text=title, font=fSm, bg=BG_DARK, fg=TEXT_SEC
                 ).pack(pady=(16, 4))

        # Display
        self.var = tk.StringVar(value=str(initial_value))
        disp_frame = tk.Frame(self, bg=BG_INPUT,
                              highlightthickness=1, highlightbackground=BORDER)
        disp_frame.pack(fill="x", padx=20, pady=4)
        self.disp = tk.Label(disp_frame, textvariable=self.var,
                             font=fB, bg=BG_INPUT, fg=TEXT_PRI,
                             anchor="e", width=12)
        self.disp.pack(padx=12, pady=10)

        # Numpad grid
        pad = tk.Frame(self, bg=BG_DARK)
        pad.pack(padx=20, pady=8)

        keys = [
            ("7", "8", "9"),
            ("4", "5", "6"),
            ("1", "2", "3"),
            (".", "0", "⌫"),
        ]

        for r, row in enumerate(keys):
            for c, lbl in enumerate(row):
                if lbl == "." and integer_only:
                    lbl = ""
                btn = tk.Button(
                    pad, text=lbl,
                    font=fM,
                    bg=BG_CARD if lbl not in ("⌫", "") else BG_INPUT,
                    fg=TEXT_PRI if lbl != "⌫" else ACCENT,
                    activebackground=ACCENT2, activeforeground=BG_DARK,
                    relief="flat",
                    highlightthickness=1, highlightbackground=BORDER,
                    width=4, height=2,
                    command=lambda l=lbl: self._press(l)
                )
                btn.grid(row=r, column=c, padx=4, pady=4, sticky="nsew")

        # OK / Cancel
        bot = tk.Frame(self, bg=BG_DARK)
        bot.pack(fill="x", padx=20, pady=(4, 16))

        tk.Button(bot, text="CANCEL", font=fSm,
                  bg=BG_INPUT, fg=TEXT_SEC,
                  relief="flat", width=10, pady=10,
                  command=self.destroy
                  ).pack(side="left", expand=True, padx=(0, 6))

        tk.Button(bot, text="✔  OK", font=fSm,
                  bg=GREEN, fg=BG_DARK,
                  relief="flat", width=10, pady=10,
                  command=self._confirm
                  ).pack(side="right", expand=True, padx=(6, 0))

    def _press(self, lbl):
        if not lbl:
            return
        cur = self.var.get()
        if lbl == "⌫":
            self.var.set(cur[:-1] if len(cur) > 1 else "0")
        elif lbl == ".":
            if "." not in cur:
                self.var.set(cur + ".")
        else:
            if cur == "0":
                self.var.set(lbl)
            else:
                self.var.set(cur + lbl)

    def _confirm(self):
        try:
            val = self.var.get().rstrip(".")
            if not val:
                val = "0"
            self.callback(val)
        except Exception:
            pass
        self.destroy()


# ═══════════════════════════════════════════════════════════════════════════════
#  Touch Keyboard Popup
# ═══════════════════════════════════════════════════════════════════════════════
class TouchKeyboardPopup(tk.Toplevel):
    """Full on-screen QWERTY keyboard for touch entry of text (position names)."""

    ROWS = [
        list("QWERTYUIOP"),
        list("ASDFGHJKL"),
        list("ZXCVBNM"),
    ]

    def __init__(self, parent, title: str, initial_value: str, callback):
        super().__init__(parent)
        self.callback = callback

        self.configure(bg=BG_DARK)
        self.title(title)
        self.resizable(False, False)

        # Centre on screen
        self.update_idletasks()
        sw = self.winfo_screenwidth()
        sh = self.winfo_screenheight()
        w, h = 680, 420
        self.geometry(f"{w}x{h}+{(sw-w)//2}+{(sh-h)//2}")
        self.update_idletasks()
        self.grab_set()
        self.lift()
        self.attributes("-topmost", True)
        self.focus_force()

        fTitle = font.Font(family="Courier", size=11, weight="bold")
        fKey   = font.Font(family="Courier", size=14, weight="bold")
        fDisp  = font.Font(family="Courier", size=20, weight="bold")
        fSm    = font.Font(family="Courier", size=10)

        tk.Label(self, text=title, font=fTitle, bg=BG_DARK, fg=TEXT_SEC
                 ).pack(pady=(12, 4))

        # Display
        self.var = tk.StringVar(value=initial_value.upper())
        disp_frame = tk.Frame(self, bg=BG_INPUT,
                              highlightthickness=1, highlightbackground=BORDER)
        disp_frame.pack(fill="x", padx=20, pady=(0, 8))
        tk.Label(disp_frame, textvariable=self.var,
                 font=fDisp, bg=BG_INPUT, fg=TEXT_PRI,
                 anchor="w", padx=12, pady=8).pack(fill="x")

        # Key grid
        key_cfg = dict(font=fKey, bg=BG_CARD, fg=TEXT_PRI,
                       relief="flat", width=3, height=1,
                       cursor="hand2",
                       highlightthickness=1, highlightbackground=BORDER,
                       activebackground=ACCENT2, activeforeground=BG_DARK)

        for row_chars in self.ROWS:
            row_frame = tk.Frame(self, bg=BG_DARK)
            row_frame.pack(pady=3)
            for ch in row_chars:
                tk.Button(row_frame, text=ch, **key_cfg,
                          command=lambda c=ch: self._press(c)
                          ).pack(side="left", padx=3)

        # Bottom row: space, numbers toggle, backspace, clear
        bot_keys = tk.Frame(self, bg=BG_DARK)
        bot_keys.pack(pady=3)

        spec_cfg = dict(font=fSm, relief="flat", cursor="hand2",
                        highlightthickness=1, highlightbackground=BORDER,
                        activebackground=ACCENT2, activeforeground=BG_DARK,
                        pady=8)

        tk.Button(bot_keys, text="⌫", bg=BG_INPUT, fg=ACCENT,
                  width=4, **spec_cfg,
                  command=self._backspace).pack(side="left", padx=3)

        tk.Button(bot_keys, text="0-9", bg=BG_INPUT, fg=ACCENT2,
                  width=4, **spec_cfg,
                  command=self._insert_digit_mode).pack(side="left", padx=3)

        tk.Button(bot_keys, text="_ SPACE", bg=BG_INPUT, fg=TEXT_SEC,
                  width=10, **spec_cfg,
                  command=lambda: self._press("_")).pack(side="left", padx=3)

        tk.Button(bot_keys, text="- DASH", bg=BG_INPUT, fg=TEXT_SEC,
                  width=8, **spec_cfg,
                  command=lambda: self._press("-")).pack(side="left", padx=3)

        tk.Button(bot_keys, text="CLR", bg=BG_INPUT, fg=RED,
                  width=4, **spec_cfg,
                  command=lambda: self.var.set("")).pack(side="left", padx=3)

        # Digit row (hidden by default, shown when 0-9 toggled)
        self.digit_frame = tk.Frame(self, bg=BG_DARK)
        for d in "1234567890":
            tk.Button(self.digit_frame, text=d, **key_cfg,
                      command=lambda c=d: self._press(c)
                      ).pack(side="left", padx=3)
        self._digits_visible = False

        # Confirm / Cancel
        act = tk.Frame(self, bg=BG_DARK)
        act.pack(pady=(4, 12))
        tk.Button(act, text="CANCEL", font=fSm,
                  bg=BG_INPUT, fg=TEXT_SEC, relief="flat",
                  width=10, pady=10, command=self.destroy
                  ).pack(side="left", padx=8)
        tk.Button(act, text="✔  OK", font=fSm,
                  bg=GREEN, fg=BG_DARK, relief="flat",
                  width=10, pady=10, command=self._confirm
                  ).pack(side="left", padx=8)

    def _press(self, ch: str):
        self.var.set(self.var.get() + ch)

    def _backspace(self):
        cur = self.var.get()
        self.var.set(cur[:-1])

    def _insert_digit_mode(self):
        self._digits_visible = not self._digits_visible
        if self._digits_visible:
            self.digit_frame.pack(before=self.digit_frame.master.winfo_children()[-1]
                                  if False else None, pady=3)
            self.digit_frame.pack(pady=3)
        else:
            self.digit_frame.pack_forget()

    def _confirm(self):
        val = self.var.get().strip()
        if val:
            self.callback(val)
        self.destroy()


# ═══════════════════════════════════════════════════════════════════════════════
#  Main GUI
# ═══════════════════════════════════════════════════════════════════════════════
class ForkliftGUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("FORKLIFT CONTROL SYSTEM")
        self.root.configure(bg=BG_DARK)
        self.root.minsize(960, 700)

        self.entries: list[dict] = []
        self.position_db: dict[str, dict] = self._load_positions()   # name -> {x, y}
        self.robot_pos   = {"x": 0.0, "y": 0.0}
        self.ros_status  = "Disconnected"
        self.ros_node: Optional[ForkliftROS2Node] = None
        self._ros_thread = None

        # Current mode
        self.mode = tk.StringVar(value=MODE_AUTO)

        # Topic prefix (editable) + numeric suffix
        self.pos_prefix = tk.StringVar(value="POS")
        self.pos_num = tk.StringVar(value="01")
        # Weight
        self.weight_val = tk.StringVar(value="0")
        # Autonomous operation state
        self.auto_running = False

        # Manual control key repeat state
        self._held_key: Optional[str] = None
        self._key_repeat_id = None

        self._build_fonts()
        self._build_ui()
        self._refresh_pos_tree()   # restore persisted positions into tree
        self._start_ros2()
        self._tick()
        self._update_mode_view()

    # ── Fonts ──────────────────────────────────────────────────────────────────
    def _build_fonts(self):
        self.f_title = font.Font(family="Courier", size=18, weight="bold")
        self.f_head  = font.Font(family="Courier", size=11, weight="bold")
        self.f_body  = font.Font(family="Courier", size=10)
        self.f_small = font.Font(family="Courier", size=9)
        self.f_big   = font.Font(family="Courier", size=24, weight="bold")
        self.f_mono  = font.Font(family="Courier", size=12)
        self.f_pad   = font.Font(family="Courier", size=20, weight="bold")

    # ── Master layout ──────────────────────────────────────────────────────────
    def _build_ui(self):
        # ── top bar ──
        top = tk.Frame(self.root, bg=BG_PANEL, height=60)
        top.pack(fill="x")
        top.pack_propagate(False)

        tk.Label(top, text="⬡  FORKLIFT CONTROL SYSTEM",
                 font=self.f_title, bg=BG_PANEL, fg=ACCENT
                 ).pack(side="left", padx=20, pady=12)

        self.lbl_time = tk.Label(top, text="", font=self.f_small,
                                 bg=BG_PANEL, fg=TEXT_SEC)
        self.lbl_time.pack(side="right", padx=20)

        self.lbl_ros = tk.Label(top, text="● ROS2 OFF", font=self.f_small,
                                bg=BG_PANEL, fg=RED)
        self.lbl_ros.pack(side="right", padx=12)

        # ── Mode selector bar ──
        mode_bar = tk.Frame(self.root, bg=BG_PANEL,
                            highlightthickness=1, highlightbackground=BORDER)
        mode_bar.pack(fill="x", padx=0, pady=0)

        tk.Label(mode_bar, text="SELECT MODE:", font=self.f_head,
                 bg=BG_PANEL, fg=TEXT_SEC).pack(side="left", padx=(20, 12), pady=8)

        self.btn_auto = tk.Button(
            mode_bar, text="🤖  AUTONOMOUS",
            font=self.f_head, relief="flat",
            padx=22, pady=10, cursor="hand2",
            command=lambda: self._set_mode(MODE_AUTO))
        self.btn_auto.pack(side="left", padx=(0, 6), pady=8)

        self.btn_manual = tk.Button(
            mode_bar, text="🕹  MANUAL CONTROL",
            font=self.f_head, relief="flat",
            padx=22, pady=10, cursor="hand2",
            command=lambda: self._set_mode(MODE_MANUAL))
        self.btn_manual.pack(side="left", padx=(0, 6), pady=8)

        self.btn_data = tk.Button(
            mode_bar, text="📍  DATA ENTRY",
            font=self.f_head, relief="flat",
            padx=22, pady=10, cursor="hand2",
            command=lambda: self._set_mode(MODE_DATA))
        self.btn_data.pack(side="left", padx=(0, 6), pady=8)

        # Mode label badge
        self.lbl_mode_badge = tk.Label(mode_bar, text="", font=self.f_head,
                                       bg=BG_PANEL, fg=GREEN)
        self.lbl_mode_badge.pack(side="left", padx=16)

        # ── body ──
        body = tk.Frame(self.root, bg=BG_DARK)
        body.pack(fill="both", expand=True, padx=12, pady=10)

        left = tk.Frame(body, bg=BG_DARK)
        left.pack(side="left", fill="both", expand=True)

        right = tk.Frame(body, bg=BG_DARK, width=300)
        right.pack(side="right", fill="y", padx=(10, 0))
        right.pack_propagate(False)

        # Build panels, toggled by mode
        self._build_autonomous_section(left)
        self._build_manual_section(left)
        self._build_position_data_section(left)
        self._build_table_section(left)
        self._build_right_panel(right)

    # ── Mode switching ─────────────────────────────────────────────────────────
    def _set_mode(self, mode: str):
        self.mode.set(mode)
        self._update_mode_view()
        if mode == MODE_AUTO:
            cmd = "MODE_AUTONOMOUS"
        elif mode == MODE_MANUAL:
            cmd = "MODE_MANUAL"
        else:
            cmd = "MODE_DATA_ENTRY"
        self._log(f"[MODE] Switched to {mode}", "info")
        if self.ros_node and ROS2_AVAILABLE:
            try:
                self.ros_node.publish_command(cmd)
            except Exception:
                pass

    def _update_mode_view(self):
        m = self.mode.get()
        # Reset all buttons to inactive style
        self.btn_auto.config(bg=BG_INPUT, fg=TEXT_SEC)
        self.btn_manual.config(bg=BG_INPUT, fg=TEXT_SEC)
        self.btn_data.config(bg=BG_INPUT, fg=TEXT_SEC)
        # Hide all mode panels
        self.auto_frame.pack_forget()
        self.manual_frame.pack_forget()
        self.pos_data_frame.pack_forget()
        # Hide pallet registry by default; show only in AUTO
        self.table_frame.pack_forget()

        if m == MODE_AUTO:
            self.btn_auto.config(bg=ACCENT2, fg=BG_DARK)
            self.lbl_mode_badge.config(text="▶  AUTONOMOUS MODE ACTIVE", fg=ACCENT2)
            self.auto_frame.pack(fill="x", pady=(0, 10))
            self.table_frame.pack(fill="both", expand=True)
        elif m == MODE_MANUAL:
            self.btn_manual.config(bg=YELLOW, fg=BG_DARK)
            self.lbl_mode_badge.config(text="▶  MANUAL MODE ACTIVE", fg=YELLOW)
            self.manual_frame.pack(fill="both", expand=True, pady=(0, 10))
        else:  # DATA_ENTRY
            self.btn_data.config(bg=GREEN, fg=BG_DARK)
            self.lbl_mode_badge.config(text="▶  DATA ENTRY MODE ACTIVE", fg=GREEN)
            self.pos_data_frame.pack(fill="both", expand=True, pady=(0, 10))

    # ── Autonomous input card ──────────────────────────────────────────────────
    def _build_autonomous_section(self, parent):
        self.auto_frame = tk.Frame(parent, bg=BG_CARD,
                                   highlightthickness=1, highlightbackground=BORDER)
        # will be packed by _update_mode_view

        # ── Title ──
        tk.Label(self.auto_frame, text="NEW PALLET ENTRY",
                 font=self.f_head, bg=BG_CARD, fg=ACCENT2
                 ).pack(anchor="w", padx=16, pady=(14, 4))

        # ── Labels row ──
        labels_row = tk.Frame(self.auto_frame, bg=BG_CARD)
        labels_row.pack(fill="x", padx=16, pady=(0, 2))

        tk.Label(labels_row, text="POSITION ID", font=self.f_small,
                 bg=BG_CARD, fg=TEXT_SEC, width=28, anchor="w").pack(side="left")
        tk.Label(labels_row, text="PALLET WEIGHT (kg)", font=self.f_small,
                 bg=BG_CARD, fg=TEXT_SEC, anchor="w").pack(side="left", padx=(0, 0))

        # ── Inputs row ──
        inputs_row = tk.Frame(self.auto_frame, bg=BG_CARD)
        inputs_row.pack(fill="x", padx=16, pady=(0, 12))

        # -- Position: editable prefix + "-" + number + ENTER button --
        pos_group = tk.Frame(inputs_row, bg=BG_CARD)
        pos_group.pack(side="left", padx=(0, 20))

        self.lbl_pos_prefix = tk.Label(
            pos_group, textvariable=self.pos_prefix,
            font=self.f_mono, bg=BG_INPUT, fg=ACCENT2,
            padx=6, pady=6, relief="flat", cursor="hand2",
            highlightthickness=2, highlightbackground=BORDER)
        self.lbl_pos_prefix.pack(side="left")
        self.lbl_pos_prefix.bind("<Button-1>", self._open_prefix_editor)

        tk.Label(pos_group, text="-", font=self.f_mono,
                 bg=BG_INPUT, fg=TEXT_SEC, padx=2, pady=6, relief="flat"
                 ).pack(side="left")

        self.lbl_pos_num = tk.Label(
            pos_group, textvariable=self.pos_num,
            font=self.f_big, bg=BG_INPUT, fg=TEXT_PRI,
            padx=12, pady=4, relief="flat", cursor="hand2",
            highlightthickness=2, highlightbackground=ACCENT2, width=4)
        self.lbl_pos_num.pack(side="left")
        self.lbl_pos_num.bind("<Button-1>", self._open_pos_numpad)

        tk.Button(pos_group, text="✎ ENTER",
                  font=self.f_small, bg=ACCENT2, fg=BG_DARK,
                  relief="flat", padx=10, pady=8, cursor="hand2",
                  command=self._open_pos_numpad
                  ).pack(side="left", padx=(8, 0))

        # -- Weight: display + ENTER button --
        wt_group = tk.Frame(inputs_row, bg=BG_CARD)
        wt_group.pack(side="left", padx=(0, 20))

        self.lbl_weight_disp = tk.Label(
            wt_group, textvariable=self.weight_val,
            font=self.f_big, bg=BG_INPUT, fg=TEXT_PRI,
            padx=16, pady=4, relief="flat", cursor="hand2",
            highlightthickness=2, highlightbackground=ACCENT2, width=7)
        self.lbl_weight_disp.pack(side="left")
        self.lbl_weight_disp.bind("<Button-1>", self._open_weight_numpad)

        tk.Label(wt_group, text="kg", font=self.f_small,
                 bg=BG_CARD, fg=TEXT_SEC).pack(side="left", padx=(6, 4))

        tk.Button(wt_group, text="✎ ENTER",
                  font=self.f_small, bg=ACCENT2, fg=BG_DARK,
                  relief="flat", padx=10, pady=8, cursor="hand2",
                  command=self._open_weight_numpad
                  ).pack(side="left", padx=(4, 0))

        # ── Position lookup indicator ──────────────────────────────────────
        lookup_row = tk.Frame(self.auto_frame, bg=BG_CARD)
        lookup_row.pack(fill="x", padx=16, pady=(0, 6))
        tk.Label(lookup_row, text="POSITION LOOKUP:", font=self.f_small,
                 bg=BG_CARD, fg=TEXT_SEC).pack(side="left")
        self.lbl_pos_lookup = tk.Label(lookup_row,
                                       text="—  enter a position to check DB",
                                       font=self.f_small, bg=BG_CARD, fg=TEXT_SEC)
        self.lbl_pos_lookup.pack(side="left", padx=8)

        def _update_lookup(*_):
            self._refresh_lookup_label()
        self.pos_prefix.trace_add("write", _update_lookup)
        self.pos_num.trace_add("write", _update_lookup)

        # -- Action buttons row --
        btn_group = tk.Frame(self.auto_frame, bg=BG_CARD)
        btn_group.pack(anchor="w", padx=16, pady=(0, 12))

        self.btn_add = tk.Button(
            btn_group, text="▶  EVALUATE & ADD",
            font=self.f_head, bg=ACCENT, fg=BG_DARK,
            relief="flat", cursor="hand2", padx=20, pady=12,
            command=self._on_add)
        self.btn_add.pack(side="left", padx=(0, 8))

        tk.Button(btn_group, text="CLEAR ALL",
                  font=self.f_small, bg=BG_INPUT, fg=TEXT_SEC,
                  relief="flat", cursor="hand2", padx=12, pady=12,
                  command=self._on_clear
                  ).pack(side="left", padx=(0, 8))

        self.btn_start = tk.Button(
            btn_group, text="🚀  START OPERATION",
            font=self.f_head, bg=GREEN, fg=BG_DARK,
            relief="flat", cursor="hand2", padx=20, pady=12,
            command=self._on_start_operation)
        self.btn_start.pack(side="left")

        # ── Legend ──
        legend = tk.Frame(self.auto_frame, bg=BG_CARD)
        legend.pack(anchor="w", padx=16, pady=(0, 6))
        _legend_item(legend, "0–50 kg",   "FULL SPEED", GREEN)
        _legend_item(legend, "50–100 kg", "WARNING",    YELLOW)
        _legend_item(legend, "> 100 kg",  "REJECTED",   RED)

        # ── Gauge ──
        tk.Label(self.auto_frame, text="WEIGHT GAUGE",
                 font=self.f_small, bg=BG_CARD, fg=TEXT_SEC
                 ).pack(anchor="w", padx=16)
        self.canvas_gauge = tk.Canvas(self.auto_frame, height=22, bg=BG_INPUT,
                                      highlightthickness=0)
        self.canvas_gauge.pack(fill="x", padx=16, pady=(2, 16))

        self.weight_val.trace_add("write", lambda *_: self._update_gauge())

    # ── Prefix editor popup ────────────────────────────────────────────────────
    def _open_prefix_editor(self, event=None):
        dlg = tk.Toplevel(self.root)
        dlg.configure(bg=BG_DARK)
        dlg.title("EDIT TOPIC NAME")
        dlg.resizable(False, False)
        dlg.grab_set()

        pw = self.root.winfo_width()
        ph = self.root.winfo_height()
        px = self.root.winfo_rootx()
        py = self.root.winfo_rooty()
        w, h = 400, 200
        dlg.geometry(f"{w}x{h}+{px+(pw-w)//2}+{py+(ph-h)//2}")

        tk.Label(dlg, text="EDIT TOPIC / PREFIX NAME",
                 font=self.f_head, bg=BG_DARK, fg=ACCENT2
                 ).pack(pady=(20, 8))

        entry_var = tk.StringVar(value=self.pos_prefix.get())
        entry = tk.Entry(dlg, textvariable=entry_var,
                         font=self.f_big, bg=BG_INPUT, fg=TEXT_PRI,
                         insertbackground=TEXT_PRI, relief="flat",
                         highlightthickness=2, highlightbackground=ACCENT2,
                         width=12, justify="center")
        entry.pack(padx=30, pady=8, ipady=6)
        entry.focus_set()
        entry.select_range(0, "end")

        def _confirm(e=None):
            val = entry_var.get().strip().upper()
            if val:
                self.pos_prefix.set(val)
                self._log(f"[INFO] Topic prefix changed to '{val}'", "info")
            dlg.destroy()

        entry.bind("<Return>", _confirm)

        btn_row = tk.Frame(dlg, bg=BG_DARK)
        btn_row.pack(pady=8)
        tk.Button(btn_row, text="CANCEL", font=self.f_small,
                  bg=BG_INPUT, fg=TEXT_SEC, relief="flat", padx=16, pady=10,
                  command=dlg.destroy).pack(side="left", padx=8)
        tk.Button(btn_row, text="✔  SAVE", font=self.f_small,
                  bg=GREEN, fg=BG_DARK, relief="flat", padx=16, pady=10,
                  command=_confirm).pack(side="right", padx=8)

    # ── Autonomous START operation ─────────────────────────────────────────────
    def _on_start_operation(self):
        valid_entries = [e for e in self.entries if e["tag"] != "red"]
        if not valid_entries:
            self._show_toast("No valid entries to operate on. Add positions first.", color=YELLOW)
            return
        if self.auto_running:
            self._show_toast("Operation already running!", color=YELLOW)
            return

        self.auto_running = True
        self.btn_start.config(text="⏳  RUNNING...", bg=YELLOW, state="disabled")
        self.btn_add.config(state="disabled")
        self._log(f"[START] Autonomous operation started — {len(valid_entries)} target(s).", "ok")

        if self.ros_node and ROS2_AVAILABLE:
            try:
                payload = json.dumps([{"pos": e["pos"], "weight": e["weight"]} for e in valid_entries])
                self.ros_node.publish_command(f"START_AUTONOMOUS:{payload}")
            except Exception as ex:
                self._log(f"[ROS2] Start command error: {ex}", "err")

        # Simulate sequential operation (replace with real ROS2 feedback loop)
        self._run_next_target(valid_entries, 0)

    def _run_next_target(self, targets: list, idx: int):
        if idx >= len(targets):
            self.auto_running = False
            self.btn_start.config(text="🚀  START OPERATION", bg=GREEN, state="normal")
            self.btn_add.config(state="normal")
            self._log("[DONE] All targets completed. Forklift returned to home.", "ok")
            self._show_toast("✔ All operations complete!", color=GREEN, duration=4000)
            return

        t = targets[idx]
        self._log(f"[AUTO] → Navigating to {t['pos']} ({t['weight']:.1f} kg)...", "info")
        if self.ros_node and ROS2_AVAILABLE:
            try:
                self.ros_node.publish_command(f"GOTO:{t['pos']}")
            except Exception:
                pass
        # Simulate travel + lift (3 s per target in simulation mode)
        self.root.after(1500, lambda: self._log(f"[AUTO] → Lifting pallet at {t['pos']}...", "warn"))
        self.root.after(3000, lambda: self._finish_target(targets, idx))

    def _finish_target(self, targets: list, idx: int):
        t = targets[idx]
        self._log(f"[AUTO] ✔ Pallet at {t['pos']} lifted & completed.", "ok")
        self._run_next_target(targets, idx + 1)

    # ── Numpad launchers ──────────────────────────────────────────────────────
    def _open_pos_numpad(self, event=None):
        NumpadPopup(self.root, "ENTER POSITION NUMBER",
                    self.pos_num.get(),
                    self._set_pos_num, integer_only=True)

    def _set_pos_num(self, val: str):
        try:
            n = int(val)
            self.pos_num.set(f"{n:02d}")
        except ValueError:
            self.pos_num.set("01")

    def _open_weight_numpad(self, event=None):
        NumpadPopup(self.root, "ENTER WEIGHT (kg)",
                    self.weight_val.get(),
                    self._set_weight_val, integer_only=False)

    def _set_weight_val(self, val: str):
        try:
            float(val)
            self.weight_val.set(val)
        except ValueError:
            self.weight_val.set("0")

    # ── Gauge ──────────────────────────────────────────────────────────────────
    def _update_gauge(self):
        try:
            w = float(self.weight_val.get())
        except ValueError:
            return
        self.canvas_gauge.update_idletasks()
        W = self.canvas_gauge.winfo_width() or 400
        frac = min(w / WEIGHT_WARNING, 1.0)
        fill_w = int(frac * W)
        color = GREEN if w <= WEIGHT_FULL_SPEED else (YELLOW if w <= WEIGHT_WARNING else RED)
        self.canvas_gauge.delete("all")
        self.canvas_gauge.create_rectangle(0, 0, W, 22, fill=BG_INPUT, outline="")
        if fill_w > 0:
            self.canvas_gauge.create_rectangle(0, 0, fill_w, 22, fill=color, outline="")
        self.canvas_gauge.create_text(W // 2, 11,
                                      text=f"{w:.0f} kg",
                                      fill=BG_DARK if frac > 0.3 else TEXT_SEC,
                                      font=self.f_small)

    def _refresh_lookup_label(self):
        """Update the position-lookup indicator in Autonomous mode.
        Tries both the zero-padded form (POS-01) and the raw form (POS-1)
        so it matches however the name was typed in Data Entry.
        """
        if not hasattr(self, "lbl_pos_lookup"):
            return
        prefix = self.pos_prefix.get()
        num    = self.pos_num.get()
        # Build both variants
        padded   = f"{prefix}-{num}"          # e.g. POS-01
        stripped = f"{prefix}-{num.lstrip('0') or '0'}"  # e.g. POS-1

        entry = self.position_db.get(padded) or self.position_db.get(stripped)
        matched_key = padded if self.position_db.get(padded) else stripped

        if entry:
            self.lbl_pos_lookup.config(
                text=f"✔  '{matched_key}'  →  X = {entry['x']:.3f} m   Y = {entry['y']:.3f} m",
                fg=GREEN)
        else:
            # Show all saved names as hint
            saved = ", ".join(list(self.position_db.keys())[:6]) or "none yet"
            self.lbl_pos_lookup.config(
                text=f"✘  '{padded}' not found  |  saved: {saved}",
                fg=RED)

    # ── Manual control panel ───────────────────────────────────────────────────
    def _build_manual_section(self, parent):
        self.manual_frame = tk.Frame(parent, bg=BG_CARD,
                                     highlightthickness=1, highlightbackground=BORDER)
        # will be packed by _update_mode_view

        tk.Label(self.manual_frame, text="MANUAL CONTROL",
                 font=self.f_head, bg=BG_CARD, fg=YELLOW
                 ).pack(anchor="w", padx=16, pady=(14, 4))

        tk.Label(self.manual_frame,
                 text="Hold arrow to drive. Release to stop.",
                 font=self.f_small, bg=BG_CARD, fg=TEXT_SEC
                 ).pack(anchor="w", padx=16, pady=(0, 8))

        # D-pad layout
        dpad = tk.Frame(self.manual_frame, bg=BG_CARD)
        dpad.pack(pady=(0, 16))

        btn_cfg = dict(font=self.f_pad, bg=BG_INPUT, fg=TEXT_PRI,
                       relief="flat", width=4, height=2,
                       cursor="hand2",
                       highlightthickness=1, highlightbackground=BORDER,
                       activebackground=YELLOW, activeforeground=BG_DARK)

        # Up
        btn_up = tk.Button(dpad, text="▲", **btn_cfg)
        btn_up.grid(row=0, column=1, padx=6, pady=6)

        # Diagonal rotate buttons (top-left, top-right)
        diag_cfg = dict(font=("Courier", 16, "bold"),
                        bg=BG_INPUT, fg=ACCENT2,
                        relief="flat", width=4, height=2, cursor="hand2",
                        highlightthickness=1, highlightbackground=BORDER,
                        activebackground=ACCENT2, activeforeground=BG_DARK)

        btn_diag_fl = tk.Button(dpad, text="⬉", **diag_cfg)   # forward-left rotate
        btn_diag_fl.grid(row=0, column=0, padx=6, pady=6)

        btn_diag_fr = tk.Button(dpad, text="⬈", **diag_cfg)   # forward-right rotate
        btn_diag_fr.grid(row=0, column=2, padx=6, pady=6)

        # Left
        btn_left = tk.Button(dpad, text="◀", **btn_cfg)
        btn_left.grid(row=1, column=0, padx=6, pady=6)

        # Emergency Stop
        self.btn_estop = tk.Button(dpad, text="⛔",
                             font=self.f_pad, bg=RED, fg=BG_DARK,
                             relief="flat", width=4, height=2,
                             cursor="hand2",
                             command=self._emergency_stop)
        self.btn_estop.grid(row=1, column=1, padx=6, pady=6)

        # Right
        btn_right = tk.Button(dpad, text="▶", **btn_cfg)
        btn_right.grid(row=1, column=2, padx=6, pady=6)

        # Down
        btn_down = tk.Button(dpad, text="▼", **btn_cfg)
        btn_down.grid(row=2, column=1, padx=6, pady=6)

        # Diagonal rotate buttons (back-left, back-right)
        btn_diag_bl = tk.Button(dpad, text="⬋", **diag_cfg)   # backward-left rotate
        btn_diag_bl.grid(row=2, column=0, padx=6, pady=6)

        btn_diag_br = tk.Button(dpad, text="⬊", **diag_cfg)   # backward-right rotate
        btn_diag_br.grid(row=2, column=2, padx=6, pady=6)

        # Lights / lift row
        extras = tk.Frame(self.manual_frame, bg=BG_CARD)
        extras.pack(pady=(0, 4))

        # Lights label
        tk.Label(extras, text="LIGHTS:", font=self.f_small,
                 bg=BG_CARD, fg=TEXT_SEC).pack(side="left", padx=(8, 4))

        # Track toggle state for each light
        self._light_state = {"LEFT": False, "RIGHT": False, "FRONT": False, "BACK": False}

        def _make_light_btn(parent, label, cmd_key, icon):
            btn_var = [None]
            def _toggle():
                self._light_state[cmd_key] = not self._light_state[cmd_key]
                on = self._light_state[cmd_key]
                btn_var[0].config(
                    bg=YELLOW if on else BG_INPUT,
                    fg=BG_DARK if on else TEXT_SEC,
                    text=f"{icon} {label} {'ON' if on else 'OFF'}"
                )
                self._manual_command(f"LIGHT_{cmd_key}_{'ON' if on else 'OFF'}")
            b = tk.Button(parent, text=f"{icon} {label} OFF",
                          font=self.f_small, bg=BG_INPUT, fg=TEXT_SEC,
                          relief="flat", padx=14, pady=10, cursor="hand2",
                          command=_toggle)
            btn_var[0] = b
            return b

        _make_light_btn(extras, "LEFT",  "LEFT",  "◄").pack(side="left", padx=6)
        _make_light_btn(extras, "FRONT", "FRONT", "●").pack(side="left", padx=6)
        _make_light_btn(extras, "RIGHT", "RIGHT", "►").pack(side="left", padx=6)
        _make_light_btn(extras, "BACK",  "BACK",  "◉").pack(side="left", padx=6)

        lift_row = tk.Frame(self.manual_frame, bg=BG_CARD)
        lift_row.pack(pady=(0, 16))

        btn_lift_up = tk.Button(lift_row, text="⬆ LIFT",
                                font=self.f_head, bg=BG_INPUT, fg=ACCENT2,
                                relief="flat", padx=16, pady=10, cursor="hand2",
                                command=lambda: self._manual_command("LIFT_UP"))
        btn_lift_up.pack(side="left", padx=8)

        btn_lift_dn = tk.Button(lift_row, text="⬇ LOWER",
                                font=self.f_head, bg=BG_INPUT, fg=ACCENT2,
                                relief="flat", padx=16, pady=10, cursor="hand2",
                                command=lambda: self._manual_command("LIFT_DOWN"))
        btn_lift_dn.pack(side="left", padx=8)

        # Bind press/release for continuous movement
        arrow_map = {
            btn_up:       "FORWARD",
            btn_down:     "BACKWARD",
            btn_left:     "STRAFE_LEFT",
            btn_right:    "STRAFE_RIGHT",
            btn_diag_fl:  "ROTATE_FL",
            btn_diag_fr:  "ROTATE_FR",
            btn_diag_bl:  "ROTATE_BL",
            btn_diag_br:  "ROTATE_BR",
        }
        for btn, cmd in arrow_map.items():
            # command= is reliable for press; <ButtonRelease-1> bind handles release.
            # We must NOT use both bind(<ButtonPress-1>) AND command= — double-fire.
            btn.config(command=lambda c=cmd: self._start_move(c))
            btn.bind("<ButtonRelease-1>", lambda e: self._stop_move())

    def _emergency_stop(self):
        """Hard stop: cancel held key, publish zero Twist AND E_STOP command,
        flash the button to give tactile feedback."""
        # Cancel any ongoing hold-repeat
        self._held_key = None
        if self._key_repeat_id:
            self.root.after_cancel(self._key_repeat_id)
            self._key_repeat_id = None
        # Cancel autonomous operation if running
        if self.auto_running:
            self.auto_running = False
            self.btn_start.config(text="🚀  START OPERATION", bg=GREEN, state="normal")
            self.btn_add.config(state="normal")
            self._log("[E-STOP] Autonomous operation cancelled.", "err")
        # Publish zero Twist to /cmd_vel
        if self.ros_node and ROS2_AVAILABLE:
            try:
                self.ros_node.publish_twist(0.0, 0.0, 0.0)
                self.ros_node.publish_command("E_STOP")
            except Exception as e:
                self._log(f"[ROS2] E-STOP publish error: {e}", "err")
        self._log("[E-STOP] ⛔ EMERGENCY STOP TRIGGERED", "err")
        # Flash button white → red to give visual feedback
        self.btn_estop.config(bg="white", fg=RED)
        self.root.after(150, lambda: self.btn_estop.config(bg=RED, fg=BG_DARK))
        self.root.after(300, lambda: self.btn_estop.config(bg="white", fg=RED))
        self.root.after(450, lambda: self.btn_estop.config(bg=RED, fg=BG_DARK))

    def _start_move(self, cmd: str):
        self._manual_command(cmd)
        self._held_key = cmd
        self._schedule_repeat()

    def _schedule_repeat(self):
        if self._held_key:
            self._key_repeat_id = self.root.after(120, self._repeat_move)

    def _repeat_move(self):
        if self._held_key:
            self._manual_command(self._held_key)
            self._schedule_repeat()

    def _stop_move(self):
        self._held_key = None
        if self._key_repeat_id:
            self.root.after_cancel(self._key_repeat_id)
            self._key_repeat_id = None
        self._manual_command("STOP")

    def _manual_command(self, cmd: str):
        self._log(f"[MANUAL] {cmd}", "warn")
        if self.ros_node and ROS2_AVAILABLE:
            try:
                # Movement commands → /cmd_vel (Twist) ONLY — no String on /forklift/command
                twist_map = {
                    "FORWARD":      ( SPEED_LINEAR,    0.0,             0.0),
                    "BACKWARD":     (-SPEED_LINEAR,    0.0,             0.0),
                    "STRAFE_LEFT":  ( 0.0,             SPEED_STRAFE,    0.0),
                    "STRAFE_RIGHT": ( 0.0,            -SPEED_STRAFE,    0.0),
                    "ROTATE_FL":    ( SPEED_DIAG_LIN,  0.0,             SPEED_DIAG_ANG),
                    "ROTATE_FR":    ( SPEED_DIAG_LIN,  0.0,            -SPEED_DIAG_ANG),
                    "ROTATE_BL":    (-SPEED_DIAG_LIN,  0.0,             SPEED_DIAG_ANG),
                    "ROTATE_BR":    (-SPEED_DIAG_LIN,  0.0,            -SPEED_DIAG_ANG),
                    "STOP":         ( 0.0,             0.0,             0.0),
                }
                if cmd in twist_map:
                    lx, ly, az = twist_map[cmd]
                    self.ros_node.publish_twist(lx, ly, az)
                    # Do NOT also publish_command here — keeps /forklift/command clean
                else:
                    # Non-movement commands (lights, lift) go to /forklift/command
                    self.ros_node.publish_command(f"MANUAL_{cmd}")
            except Exception as e:
                self._log(f"[ROS2] Error: {e}", "err")

    # ── Position Data Entry mode panel ────────────────────────────────────────
    def _build_position_data_section(self, parent):
        # Main card — shown/hidden by _update_mode_view like other mode panels
        self.pos_data_frame = tk.Frame(parent, bg=BG_CARD,
                                       highlightthickness=1, highlightbackground=BORDER)
        # will be packed by _update_mode_view

        tk.Label(self.pos_data_frame, text="POSITION DATA ENTRY",
                 font=self.f_head, bg=BG_CARD, fg=GREEN
                 ).pack(anchor="w", padx=16, pady=(14, 2))
        tk.Label(self.pos_data_frame,
                 text="Define pallet locations by name and X,Y coordinates.",
                 font=self.f_small, bg=BG_CARD, fg=TEXT_SEC
                 ).pack(anchor="w", padx=16, pady=(0, 10))

        # ── Input row ──
        inp = tk.Frame(self.pos_data_frame, bg=BG_CARD)
        inp.pack(fill="x", padx=16, pady=(10, 4))

        # Name field — touch-friendly: tap opens on-screen keyboard
        tk.Label(inp, text="POSITION NAME", font=self.f_small,
                 bg=BG_CARD, fg=TEXT_SEC).grid(row=0, column=0, sticky="w", pady=(0,2))
        self.pos_name_var = tk.StringVar(value="")
        name_entry = tk.Label(
            inp, textvariable=self.pos_name_var,
            font=self.f_mono, bg=BG_INPUT, fg=TEXT_PRI,
            anchor="w", padx=8, pady=6, cursor="hand2",
            width=14,
            highlightthickness=2, highlightbackground=ACCENT2)
        name_entry.grid(row=1, column=0, padx=(0, 12), ipady=2, sticky="ew")
        name_entry.bind("<Button-1>", lambda e: TouchKeyboardPopup(
            self.root, "ENTER POSITION NAME", self.pos_name_var.get(),
            lambda v: self.pos_name_var.set(v)))

        # X field
        tk.Label(inp, text="X (m)", font=self.f_small,
                 bg=BG_CARD, fg=TEXT_SEC).grid(row=0, column=1, sticky="w", pady=(0,2))
        self.pos_x_var = tk.StringVar(value="0.0")
        x_entry = tk.Entry(inp, textvariable=self.pos_x_var,
                           font=self.f_mono, bg=BG_INPUT, fg=TEXT_PRI,
                           insertbackground=TEXT_PRI, relief="flat",
                           highlightthickness=2, highlightbackground=BORDER,
                           width=8)
        x_entry.grid(row=1, column=1, padx=(0, 8), ipady=6)
        x_entry.bind("<Button-1>", lambda e: NumpadPopup(
            self.root, "ENTER X (m)", self.pos_x_var.get(),
            lambda v: self.pos_x_var.set(v)))

        # Y field
        tk.Label(inp, text="Y (m)", font=self.f_small,
                 bg=BG_CARD, fg=TEXT_SEC).grid(row=0, column=2, sticky="w", pady=(0,2))
        self.pos_y_var = tk.StringVar(value="0.0")
        y_entry = tk.Entry(inp, textvariable=self.pos_y_var,
                           font=self.f_mono, bg=BG_INPUT, fg=TEXT_PRI,
                           insertbackground=TEXT_PRI, relief="flat",
                           highlightthickness=2, highlightbackground=BORDER,
                           width=8)
        y_entry.grid(row=1, column=2, padx=(0, 12), ipady=6)
        y_entry.bind("<Button-1>", lambda e: NumpadPopup(
            self.root, "ENTER Y (m)", self.pos_y_var.get(),
            lambda v: self.pos_y_var.set(v)))

        # Buttons
        btn_col = tk.Frame(inp, bg=BG_CARD)
        btn_col.grid(row=0, column=3, rowspan=2, padx=(4, 0), sticky="ns")

        tk.Button(btn_col, text="➕  ADD / UPDATE",
                  font=self.f_small, bg=GREEN, fg=BG_DARK,
                  relief="flat", padx=12, pady=8, cursor="hand2",
                  command=self._pos_data_add
                  ).pack(fill="x", pady=(0, 4))

        tk.Button(btn_col, text="📍  USE CURRENT POS",
                  font=self.f_small, bg=ACCENT2, fg=BG_DARK,
                  relief="flat", padx=12, pady=8, cursor="hand2",
                  command=self._pos_data_use_current
                  ).pack(fill="x")

        # ── Position table ──
        tbl_frame = tk.Frame(self.pos_data_frame, bg=BG_CARD)
        tbl_frame.pack(fill="x", padx=16, pady=(8, 12))

        cols = ("name", "x", "y", "updated")
        self.pos_tree = ttk.Treeview(tbl_frame, columns=cols,
                                     show="headings", height=4)
        hdrs = {
            "name":    ("POSITION NAME", 160),
            "x":       ("X (m)",          90),
            "y":       ("Y (m)",          90),
            "updated": ("LAST UPDATED",  160),
        }
        style = ttk.Style()
        style.configure("PosData.Treeview",
                        background=BG_INPUT, foreground=TEXT_PRI,
                        fieldbackground=BG_INPUT, rowheight=28,
                        font=("Courier", 9), borderwidth=0)
        style.configure("PosData.Treeview.Heading",
                        background=BG_PANEL, foreground=ACCENT2,
                        font=("Courier", 9, "bold"), relief="flat")
        self.pos_tree.configure(style="PosData.Treeview")
        for col, (label, w) in hdrs.items():
            self.pos_tree.heading(col, text=label)
            self.pos_tree.column(col, width=w, anchor="center")

        pos_sb = ttk.Scrollbar(tbl_frame, orient="vertical",
                               command=self.pos_tree.yview)
        self.pos_tree.configure(yscrollcommand=pos_sb.set)
        self.pos_tree.pack(side="left", fill="x", expand=True)
        pos_sb.pack(side="right", fill="y")

        # Click row to load into fields
        self.pos_tree.bind("<<TreeviewSelect>>", self._pos_tree_select)

        # Delete button
        tk.Button(self.pos_data_frame, text="🗑  DELETE SELECTED",
                  font=self.f_small, bg=BG_INPUT, fg=RED,
                  relief="flat", padx=12, pady=6, cursor="hand2",
                  command=self._pos_data_delete
                  ).pack(anchor="e", padx=16, pady=(0, 10))

    def _pos_data_add(self):
        name = self.pos_name_var.get().strip()
        if not name:
            self._show_toast("Enter a position name first.", color=YELLOW)
            return
        try:
            x = float(self.pos_x_var.get())
            y = float(self.pos_y_var.get())
        except ValueError:
            self._show_toast("X and Y must be valid numbers.", color=RED)
            return

        ts = datetime.now().strftime("%H:%M:%S")
        self.position_db[name] = {"x": x, "y": y, "ts": ts}
        self._save_positions()
        self._refresh_pos_tree()
        # Refresh the autonomous lookup indicator
        self._refresh_lookup_label()
        action = "Updated"
        self._log(f"[DATA] {action} position '{name}' → X={x:.3f} Y={y:.3f}", "ok")
        self._show_toast(f"✔ '{name}' saved at ({x:.2f}, {y:.2f})", color=GREEN, duration=2000)

        # Publish to ROS2 so navigation can use it
        if self.ros_node and ROS2_AVAILABLE:
            try:
                payload = json.dumps({"name": name, "x": x, "y": y})
                self.ros_node.publish_command(f"SAVE_POSITION:{payload}")
            except Exception as ex:
                self._log(f"[ROS2] Position save error: {ex}", "err")

    def _pos_data_use_current(self):
        name = self.pos_name_var.get().strip()
        if not name:
            self._show_toast("Enter a position name first.", color=YELLOW)
            return
        x = self.robot_pos.get("x", 0.0)
        y = self.robot_pos.get("y", 0.0)
        self.pos_x_var.set(f"{x:.3f}")
        self.pos_y_var.set(f"{y:.3f}")
        self._pos_data_add()

    def _pos_tree_select(self, event=None):
        sel = self.pos_tree.selection()
        if not sel:
            return
        vals = self.pos_tree.item(sel[0], "values")
        self.pos_name_var.set(vals[0])
        self.pos_x_var.set(vals[1])
        self.pos_y_var.set(vals[2])

    def _pos_data_delete(self):
        sel = self.pos_tree.selection()
        if not sel:
            return
        name = self.pos_tree.item(sel[0], "values")[0]
        if name in self.position_db:
            del self.position_db[name]
        self.pos_tree.delete(sel[0])
        self._save_positions()
        self._log(f"[DATA] Deleted position '{name}'", "warn")

    def _load_positions(self) -> dict:
        """Load position_db from disk; return empty dict if file missing or corrupt."""
        try:
            with open(POSITIONS_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)
            if isinstance(data, dict):
                return data
        except (FileNotFoundError, json.JSONDecodeError, OSError):
            pass
        return {}

    def _save_positions(self):
        """Persist position_db to disk atomically."""
        try:
            tmp = POSITIONS_FILE + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(self.position_db, f, indent=2)
            os.replace(tmp, POSITIONS_FILE)
        except OSError as e:
            self._log(f"[WARN] Could not save positions: {e}", "warn")

    def _refresh_pos_tree(self):
        self.pos_tree.delete(*self.pos_tree.get_children())
        for name, d in self.position_db.items():
            self.pos_tree.insert("", "end",
                                 values=(name, f"{d['x']:.3f}", f"{d['y']:.3f}", d["ts"]))

    # ── Pallet table ───────────────────────────────────────────────────────────
    def _build_table_section(self, parent):
        # Wrapper — shown only in AUTONOMOUS mode
        self.table_frame = tk.Frame(parent, bg=BG_DARK)
        # (packed / forgotten by _update_mode_view)

        hdr = tk.Frame(self.table_frame, bg=BG_DARK)
        hdr.pack(fill="x", pady=(0, 4))
        tk.Label(hdr, text="PALLET REGISTRY", font=self.f_head,
                 bg=BG_DARK, fg=TEXT_PRI).pack(side="left")
        self.lbl_count = tk.Label(hdr, text="0 entries", font=self.f_small,
                                  bg=BG_DARK, fg=TEXT_SEC)
        self.lbl_count.pack(side="right")

        cols = ("position", "weight_kg", "status", "speed_mode", "timestamp")
        self.tree = ttk.Treeview(self.table_frame, columns=cols, show="headings", height=10)

        style = ttk.Style()
        style.theme_use("default")
        style.configure("Treeview",
                         background=BG_CARD, foreground=TEXT_PRI,
                         fieldbackground=BG_CARD, rowheight=32,
                         font=("Courier", 10), borderwidth=0)
        style.configure("Treeview.Heading",
                         background=BG_PANEL, foreground=ACCENT2,
                         font=("Courier", 10, "bold"), borderwidth=0, relief="flat")
        style.map("Treeview", background=[("selected", "#264F78")])

        hdrs = {
            "position":  ("POSITION",   120),
            "weight_kg": ("WEIGHT (kg)", 110),
            "status":    ("STATUS",      180),
            "speed_mode":("SPEED MODE",  140),
            "timestamp": ("TIMESTAMP",   160),
        }
        for col, (label, w) in hdrs.items():
            self.tree.heading(col, text=label)
            self.tree.column(col, width=w, anchor="center")

        self.tree.tag_configure("green",  foreground=GREEN)
        self.tree.tag_configure("yellow", foreground=YELLOW)
        self.tree.tag_configure("red",    foreground=RED)

        sb = ttk.Scrollbar(self.table_frame, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=sb.set)
        self.tree.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")

    # ── Right panel ────────────────────────────────────────────────────────────
    def _build_right_panel(self, parent):
        # Robot position
        pos_card = _card(parent, "ROBOT POSITION (AMCL)")
        pos_card.pack(fill="x", pady=(0, 10))
        self.lbl_rx = tk.Label(pos_card, text="X :  0.000 m",
                               font=self.f_mono, bg=BG_CARD, fg=ACCENT2)
        self.lbl_rx.pack(anchor="w", padx=16, pady=(4, 2))
        self.lbl_ry = tk.Label(pos_card, text="Y :  0.000 m",
                               font=self.f_mono, bg=BG_CARD, fg=ACCENT2)
        self.lbl_ry.pack(anchor="w", padx=16, pady=(2, 14))

        # ROS2 commands
        cmd_card = _card(parent, "ROS2 COMMANDS")
        cmd_card.pack(fill="x", pady=(0, 10))

        cmds = [
            ("NAV HOME",       "NAVIGATE_HOME"),
            ("PAUSE",          "PAUSE"),
            ("RESUME",         "RESUME"),
            ("🛑 EMERGENCY STOP", "E_STOP"),
        ]
        for label, cmd in cmds:
            color = RED if "STOP" in cmd else (YELLOW if cmd == "PAUSE" else ACCENT2)
            bg    = RED if "STOP" in cmd else BG_INPUT
            fg    = BG_DARK if "STOP" in cmd else color
            tk.Button(cmd_card, text=label, font=self.f_small,
                      bg=bg, fg=fg,
                      relief="flat", cursor="hand2", padx=8, pady=10,
                      command=lambda c=cmd: self._send_ros_command(c)
                      ).pack(fill="x", padx=16, pady=3)
        tk.Frame(cmd_card, height=10, bg=BG_CARD).pack()

        # Status log
        log_card = _card(parent, "STATUS LOG")
        log_card.pack(fill="both", expand=True)
        self.log_text = tk.Text(log_card, font=self.f_small, bg=BG_INPUT,
                                fg=TEXT_SEC, relief="flat", bd=0,
                                state="disabled", wrap="word")
        self.log_text.pack(fill="both", expand=True, padx=10, pady=(4, 12))
        self.log_text.tag_config("ok",   foreground=GREEN)
        self.log_text.tag_config("warn", foreground=YELLOW)
        self.log_text.tag_config("err",  foreground=RED)
        self.log_text.tag_config("info", foreground=ACCENT2)

    # ── Add entry ──────────────────────────────────────────────────────────────
    def _on_add(self):
        pos = f"{self.pos_prefix.get()}-{self.pos_num.get()}"
        try:
            weight = float(self.weight_val.get())
            if weight < 0:
                raise ValueError
        except ValueError:
            self._show_toast("Enter a valid non-negative weight.")
            return

        # ── Position DB lookup ──────────────────────────────────────────────
        # If the entered position name exists in the Data Entry database,
        # resolve and display its X,Y coordinates and publish a goal pose.
        pos_coords = self.position_db.get(pos)
        if pos_coords:
            px, py = pos_coords["x"], pos_coords["y"]
            self._log(f"[POS] '{pos}' resolved → X={px:.3f} Y={py:.3f}", "info")
            if self.ros_node and ROS2_AVAILABLE:
                try:
                    self.ros_node.publish_goal_pose(px, py)
                    self._log(f"[ROS2] Goal pose published for '{pos}' → /forklift/goal_pose", "ok")
                except Exception as ex:
                    self._log(f"[ROS2] Goal pose error: {ex}", "err")
        else:
            self._log(f"[WARN] '{pos}' not found in position database — no goal pose sent.", "warn")
            self._show_toast(
                f"\u26a0 '{pos}' has no saved coordinates. Add it in DATA ENTRY first.",
                color=YELLOW, duration=3500
            )

        if weight <= WEIGHT_FULL_SPEED:
            status    = "✔  ACCEPTED"
            speed     = "FULL SPEED"
            tag       = "green"
            log_level = "ok"
            log_msg   = f"[ACCEPTED] {pos} — {weight:.1f} kg — Full speed."
        elif weight <= WEIGHT_WARNING:
            status    = "⚠  ACCEPTED (WARNING)"
            speed     = "REDUCED SPEED"
            tag       = "yellow"
            log_level = "warn"
            log_msg   = f"[WARNING]  {pos} — {weight:.1f} kg — Reduced speed."
        else:
            status    = "✖  ERROR: EXCEEDS LIMIT"
            speed     = "REJECTED"
            tag       = "red"
            log_level = "err"
            log_msg   = f"[ERROR]    {pos} — {weight:.1f} kg — Exceeds limit!"

        ts = datetime.now().strftime("%H:%M:%S")
        self.tree.insert("", "end",
                         values=(pos, f"{weight:.1f}", status, speed, ts),
                         tags=(tag,))
        self.entries.append({"pos": pos, "weight": weight,
                              "status": status, "speed": speed, "tag": tag})
        self.lbl_count.config(text=f"{len(self.entries)} entries")
        self._log(log_msg, log_level)

        if self.ros_node and ROS2_AVAILABLE:
            try:
                self.ros_node.publish_pallet_entry(pos, weight)
                self._log(f"[ROS2] Published {pos} → /forklift/pallet_data", "info")
                if weight > WEIGHT_WARNING:
                    self.ros_node.publish_command("REDUCE_SPEED")
            except Exception as e:
                self._log(f"[ROS2] Publish error: {e}", "err")

        # Advance to next position number
        try:
            n = int(self.pos_num.get()) + 1
            self.pos_num.set(f"{n:02d}")
        except Exception:
            pass
        self.weight_val.set("0")

        if weight > WEIGHT_WARNING:
            self._show_toast(f"⚠ {pos}: {weight:.1f} kg EXCEEDS LIMIT — REJECTED",
                             color=RED)

    def _on_clear(self):
        if not self.entries:
            return
        self._show_confirm("Remove all pallet entries?",
                           self._do_clear)

    def _do_clear(self):
        self.tree.delete(*self.tree.get_children())
        self.entries.clear()
        self.lbl_count.config(text="0 entries")
        self._log("[INFO] Registry cleared.", "info")

    # ── Toast notification (replaces messagebox for touch) ────────────────────
    def _show_toast(self, msg: str, color: str = YELLOW, duration: int = 3000):
        toast = tk.Toplevel(self.root)
        toast.overrideredirect(True)
        toast.configure(bg=BG_PANEL)
        toast.attributes("-topmost", True)

        pw = self.root.winfo_width()
        ph = self.root.winfo_height()
        px = self.root.winfo_rootx()
        py = self.root.winfo_rooty()

        lbl = tk.Label(toast, text=msg, font=self.f_head,
                       bg=BG_PANEL, fg=color,
                       padx=24, pady=16,
                       wraplength=600)
        lbl.pack()
        toast.update_idletasks()
        tw = toast.winfo_reqwidth()
        th = toast.winfo_reqheight()
        toast.geometry(f"{tw}x{th}+{px+(pw-tw)//2}+{py+ph-th-40}")
        self.root.after(duration, toast.destroy)

    def _show_confirm(self, msg: str, yes_cmd):
        dlg = tk.Toplevel(self.root)
        dlg.configure(bg=BG_DARK)
        dlg.resizable(False, False)
        dlg.grab_set()
        pw = self.root.winfo_width()
        ph = self.root.winfo_height()
        px = self.root.winfo_rootx()
        py = self.root.winfo_rooty()
        w, h = 380, 180
        dlg.geometry(f"{w}x{h}+{px+(pw-w)//2}+{py+(ph-h)//2}")

        tk.Label(dlg, text=msg, font=self.f_head,
                 bg=BG_DARK, fg=TEXT_PRI,
                 wraplength=340).pack(pady=30)
        btn_row = tk.Frame(dlg, bg=BG_DARK)
        btn_row.pack()
        tk.Button(btn_row, text="CANCEL", font=self.f_head,
                  bg=BG_INPUT, fg=TEXT_SEC, relief="flat",
                  padx=20, pady=12,
                  command=dlg.destroy).pack(side="left", padx=10)
        tk.Button(btn_row, text="YES, CLEAR", font=self.f_head,
                  bg=RED, fg=BG_DARK, relief="flat",
                  padx=20, pady=12,
                  command=lambda: (dlg.destroy(), yes_cmd())
                  ).pack(side="right", padx=10)

    # ── ROS2 command ──────────────────────────────────────────────────────────
    def _send_ros_command(self, cmd: str):
        self._log(f"[CMD] → {cmd}", "info")
        if self.ros_node and ROS2_AVAILABLE:
            try:
                self.ros_node.publish_command(cmd)
            except Exception as e:
                self._log(f"[ROS2] Command error: {e}", "err")

    # ── ROS2 startup ──────────────────────────────────────────────────────────
    def _start_ros2(self):
        if not ROS2_AVAILABLE:
            self._log("[INFO] rclpy not found — SIMULATION mode.", "warn")
            self._log("[INFO] Install ROS2 Humble + source setup.bash to enable.", "warn")
            return
        try:
            rclpy.init()
            self.ros_node = ForkliftROS2Node(self._ros_callback)
            self._ros_thread = threading.Thread(target=self._ros_spin, daemon=True)
            self._ros_thread.start()
            self.lbl_ros.config(text="● ROS2 ON", fg=GREEN)
            self.ros_status = "Connected"
            self._log("[ROS2] Node started — /forklift/pallet_data ready.", "ok")
        except Exception as e:
            self._log(f"[ROS2] Init failed: {e}", "err")

    def _ros_spin(self):
        try:
            rclpy.spin(self.ros_node)
        except Exception:
            pass

    def _ros_callback(self, data: dict):
        self.root.after(0, self._handle_ros_data, data)

    def _handle_ros_data(self, data: dict):
        dtype = data.get("type")
        if dtype == "pose":
            self.robot_pos = data
            self.lbl_rx.config(text=f"X :  {data['x']:+.3f} m")
            self.lbl_ry.config(text=f"Y :  {data['y']:+.3f} m")
        elif dtype == "status":
            self._log(f"[ROS2] Status: {data['value']}", "info")
        elif dtype == "map_positions":
            self._log(f"[ROS2] Map positions: {data['positions']}", "info")

    # ── Logging ───────────────────────────────────────────────────────────────
    def _log(self, msg: str, level: str = "info"):
        self.log_text.config(state="normal")
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_text.insert("end", f"{ts}  {msg}\n", level)
        self.log_text.see("end")
        self.log_text.config(state="disabled")

    # ── Clock ─────────────────────────────────────────────────────────────────
    def _tick(self):
        self.lbl_time.config(text=datetime.now().strftime("%Y-%m-%d  %H:%M:%S"))
        self.root.after(1000, self._tick)

    # ── Cleanup ───────────────────────────────────────────────────────────────
    def on_close(self):
        if self.ros_node and ROS2_AVAILABLE:
            try:
                self.ros_node.destroy_node()
                rclpy.shutdown()
            except Exception:
                pass
        self.root.destroy()


# ── Helpers ───────────────────────────────────────────────────────────────────
def _card(parent, title: str) -> tk.Frame:
    outer = tk.Frame(parent, bg=BG_CARD,
                     highlightthickness=1, highlightbackground=BORDER)
    tk.Label(outer, text=title,
             font=("Courier", 9, "bold"),
             bg=BG_CARD, fg=TEXT_SEC).pack(anchor="w", padx=16, pady=(10, 4))
    return outer


def _legend_item(parent, range_lbl, desc, color):
    f = tk.Frame(parent, bg=BG_CARD)
    f.pack(side="left", padx=(0, 20))
    dot = tk.Canvas(f, width=10, height=10, bg=BG_CARD, highlightthickness=0)
    dot.create_oval(1, 1, 9, 9, fill=color, outline="")
    dot.pack(side="left", padx=(0, 4))
    tk.Label(f, text=f"{range_lbl}  {desc}",
             font=("Courier", 9),
             bg=BG_CARD, fg=color).pack(side="left")


# ═══════════════════════════════════════════════════════════════════════════════
#  Entry point
# ═══════════════════════════════════════════════════════════════════════════════
def main():
    root = tk.Tk()
    root.geometry("1080x760")
    app = ForkliftGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
