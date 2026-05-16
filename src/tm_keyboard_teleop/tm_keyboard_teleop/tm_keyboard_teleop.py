#!/usr/bin/env python3
import math
import sys
import termios
import tty
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_srvs.srv import Trigger

from tm_msgs.srv import SetPositions


def quat_to_euler_xyz(x: float, y: float, z: float, w: float):
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = 2.0 * (w * y - z * x)
    t2 = max(min(t2, 1.0), -1.0)
    pitch = math.asin(t2)

    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw


class TmKeyboardTeleop(Node):
    def __init__(self):
        super().__init__("tm_keyboard_teleop")

        self.declare_parameter("step_xyz", 0.001)          # meters
        self.declare_parameter("step_rpy_deg", 0.5)       # degrees
        self.declare_parameter("velocity", 0.03)          # m/s
        self.declare_parameter("acc_time", 200.0)         # ms
        self.declare_parameter("blend_percentage", 0)
        self.declare_parameter("fine_goal", True)
        self.declare_parameter("use_line_motion", True)

        self.step_xyz = float(self.get_parameter("step_xyz").value)
        self.step_rpy = math.radians(float(self.get_parameter("step_rpy_deg").value))
        self.velocity = float(self.get_parameter("velocity").value)
        self.acc_time = float(self.get_parameter("acc_time").value)
        self.blend_percentage = int(self.get_parameter("blend_percentage").value)
        self.fine_goal = bool(self.get_parameter("fine_goal").value)
        self.use_line_motion = bool(self.get_parameter("use_line_motion").value)

        self.current_pose: Optional[PoseStamped] = None

        self.pose_sub = self.create_subscription(
            PoseStamped, "/tool_pose", self.pose_cb, 10
        )

        self.move_cli = self.create_client(SetPositions, "/set_positions")
        self.open_cli = self.create_client(Trigger, "/robotiq_2f85/open")
        self.close_cli = self.create_client(Trigger, "/robotiq_2f85/close")

        self.get_logger().info("Waiting for /set_positions ...")
        self.move_cli.wait_for_service()
        self.get_logger().info("Keyboard teleop ready. Press h for help.")

    def pose_cb(self, msg: PoseStamped):
        self.current_pose = msg

    def send_gripper(self, open_gripper: bool):
        cli = self.open_cli if open_gripper else self.close_cli
        if not cli.service_is_ready():
            self.get_logger().warning("Gripper service not available.")
            return

        req = Trigger.Request()
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.result() is None:
            self.get_logger().warning("Gripper command failed.")
        else:
            self.get_logger().info(f"Gripper: {future.result().message}")

    def send_pose_delta(self, dx=0.0, dy=0.0, dz=0.0, droll=0.0, dpitch=0.0, dyaw=0.0):
        if self.current_pose is None:
            self.get_logger().warning("No /tool_pose yet.")
            return

        p = self.current_pose.pose.position
        q = self.current_pose.pose.orientation
        roll, pitch, yaw = quat_to_euler_xyz(q.x, q.y, q.z, q.w)

        target = [
            p.x + dx,
            p.y + dy,
            p.z + dz,
            roll + droll,
            pitch + dpitch,
            yaw + dyaw,
        ]

        req = SetPositions.Request()
        req.motion_type = SetPositions.Request.LINE_T if self.use_line_motion else SetPositions.Request.PTP_T
        req.positions = target
        req.velocity = self.velocity
        req.acc_time = self.acc_time
        req.blend_percentage = self.blend_percentage
        req.fine_goal = self.fine_goal

        future = self.move_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

        result = future.result()
        if result is None:
            self.get_logger().warning("Move command failed or timed out.")
            return

        ok = getattr(result, "ok", True)
        if ok:
            self.get_logger().info(
                f"Move command sent: x={target[0]:.3f}, y={target[1]:.3f}, z={target[2]:.3f}, "
                f"rpy=({math.degrees(target[3]):.1f}, {math.degrees(target[4]):.1f}, {math.degrees(target[5]):.1f})"
            )
        else:
            self.get_logger().warning("Robot rejected move command.")

    def print_help(self):
        print(
            """
TM keyboard teleop
------------------
Move:
  w/s : +X / -X
  a/d : +Y / -Y
  q/e : +Z / -Z

Rotate:
  u/j : +roll / -roll
  i/k : +pitch / -pitch
  o/l : +yaw / -yaw

Gripper:
  [   : open
  ]   : close

Other:
  h   : help
  x   : quit
"""
        )


def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def main():
    rclpy.init()
    node = TmKeyboardTeleop()
    node.print_help()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            key = get_key()

            if key == "w":
                node.send_pose_delta(dx=node.step_xyz)
            elif key == "s":
                node.send_pose_delta(dx=-node.step_xyz)
            elif key == "a":
                node.send_pose_delta(dy=node.step_xyz)
            elif key == "d":
                node.send_pose_delta(dy=-node.step_xyz)
            elif key == "q":
                node.send_pose_delta(dz=node.step_xyz)
            elif key == "e":
                node.send_pose_delta(dz=-node.step_xyz)

            elif key == "u":
                node.send_pose_delta(droll=node.step_rpy)
            elif key == "j":
                node.send_pose_delta(droll=-node.step_rpy)
            elif key == "i":
                node.send_pose_delta(dpitch=node.step_rpy)
            elif key == "k":
                node.send_pose_delta(dpitch=-node.step_rpy)
            elif key == "o":
                node.send_pose_delta(dyaw=node.step_rpy)
            elif key == "l":
                node.send_pose_delta(dyaw=-node.step_rpy)

            elif key == "[":
                node.send_gripper(True)
            elif key == "]":
                node.send_gripper(False)

            elif key == "h":
                node.print_help()
            elif key == "x":
                break

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()