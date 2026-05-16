#!/usr/bin/env python3
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, UInt8, Bool, String
from std_srvs.srv import Trigger

from pyrobotiqgripper import RobotiqGripper


class Robotiq2F85USBNode(Node):
    def __init__(self) -> None:
        super().__init__("robotiq_2f85_usb_node")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("open_mm", 85.0)
        self.declare_parameter("close_mm", 0.0)
        self.declare_parameter("calibrate_on_activate", True)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("default_force", 80)
        self.declare_parameter("default_speed", 80)

        self.default_force = int(self.get_parameter("default_force").value)
        self.default_speed = int(self.get_parameter("default_speed").value)

        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.open_mm = float(self.get_parameter("open_mm").value)
        self.close_mm = float(self.get_parameter("close_mm").value)
        self.calibrate_on_activate = bool(self.get_parameter("calibrate_on_activate").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self._lock = threading.Lock()
        self._activated = False
        self._connected = False
        self._connect_attempted = False
        self._gripper: Optional[RobotiqGripper] = None

        self.width_pub = self.create_publisher(Float32, "robotiq_2f85/width_mm", 10)
        self.raw_pub = self.create_publisher(UInt8, "robotiq_2f85/raw_position", 10)
        self.active_pub = self.create_publisher(Bool, "robotiq_2f85/active", 10)
        self.status_pub = self.create_publisher(String, "robotiq_2f85/status_text", 10)

        self.width_cmd_sub = self.create_subscription(
            Float32,
            "robotiq_2f85/command_mm",
            self._on_width_cmd,
            10,
        )

        self.activate_srv = self.create_service(Trigger, "robotiq_2f85/activate", self._on_activate)
        self.open_srv = self.create_service(Trigger, "robotiq_2f85/open", self._on_open)
        self.close_srv = self.create_service(Trigger, "robotiq_2f85/close", self._on_close)
        self.connect_srv = self.create_service(Trigger, "robotiq_2f85/connect", self._on_connect)

        self._connect()

        period = 1.0 / max(publish_rate_hz, 1.0)
        self.timer = self.create_timer(period, self._publish_state)

    def _status(self, text: str) -> None:
        self.get_logger().info(text)
        if not rclpy.ok():
            return
        msg = String()
        msg.data = text
        try:
            self.status_pub.publish(msg)
        except Exception:
            pass

    def _connect(self) -> None:
        with self._lock:
            self._connect_attempted = True
            try:
                self._gripper = RobotiqGripper(portname=self.port)
                self._connected = True
                self._status(f"Connected to gripper on {self.port}")
            except Exception as exc:
                self._connected = False
                self._gripper = None
                self._status(f"Connect failed on {self.port}: {exc}")

    def _ensure_connected(self) -> bool:
        return self._connected and self._gripper is not None

    def _on_connect(self, request, response):
        del request
        self._connect_attempted = False
        self._connect()
        response.success = self._connected
        response.message = "connected" if self._connected else f"connect failed on {self.port}"
        return response

    def _on_activate(self, request, response):
        del request
        if not self._ensure_connected():
            response.success = False
            response.message = "gripper not connected"
            return response

        with self._lock:
            try:
                self._gripper.activate()
                if self.calibrate_on_activate and hasattr(self._gripper, "calibrate"):
                    self._gripper.calibrate(closemm=self.close_mm, openmm=self.open_mm)
                self._activated = True
                response.success = True
                response.message = "activated"
                self._status("Gripper activated")
            except Exception as exc:
                response.success = False
                response.message = f"activate failed: {exc}"
                self._status(response.message)
        return response

    def _on_open(self, request, response):
        del request
        if not self._activated:
            response.success = False
            response.message = "activate first"
            return response

        with self._lock:
            try:
                if hasattr(self._gripper, "goTo"):
                    self._gripper.goTo(0, self.default_speed, self.default_force)
                else:
                    self._gripper.open()
                response.success = True
                response.message = "opened"
            except Exception as exc:
                response.success = False
                response.message = f"open failed: {exc}"
        return response


    def _on_close(self, request, response):
        del request
        if not self._activated:
            response.success = False
            response.message = "activate first"
            return response

        with self._lock:
            try:
                if hasattr(self._gripper, "goTo"):
                    self._gripper.goTo(255, self.default_speed, self.default_force)
                else:
                    self._gripper.close()
                response.success = True
                response.message = "closed"
            except Exception as exc:
                response.success = False
                response.message = f"close failed: {exc}"
        return response

    def _on_width_cmd(self, msg: Float32) -> None:
        if not self._activated:
            self._status("Ignoring command_mm because gripper is not activated")
            return

        target = max(self.close_mm, min(self.open_mm, float(msg.data)))
        with self._lock:
            try:
                if hasattr(self._gripper, "goTomm"):
                    self._gripper.goTomm(target, self.default_speed, self.default_force)
                elif hasattr(self._gripper, "move_mm"):
                    self._gripper.move_mm(target)
                else:
                    self._status("No mm move function supported by installed pyRobotiqGripper version")
            except Exception as exc:
                self._status(f"move_mm failed: {exc}")

    def _publish_state(self) -> None:
        active_msg = Bool()
        active_msg.data = self._activated
        try:
            self.active_pub.publish(active_msg)
        except Exception:
            return

        if not self._ensure_connected():
            return

        with self._lock:
            try:
                raw = int(self._gripper.getPosition())
                raw_msg = UInt8()
                raw_msg.data = raw
                self.raw_pub.publish(raw_msg)
            except Exception:
                pass

            try:
                width = float(self._gripper.getPositionmm())
                width_msg = Float32()
                width_msg.data = width
                self.width_pub.publish(width_msg)
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = Robotiq2F85USBNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()