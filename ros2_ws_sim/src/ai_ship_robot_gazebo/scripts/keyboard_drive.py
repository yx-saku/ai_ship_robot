#!/usr/bin/env python3

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


HELP_TEXT = """
Keyboard drive controls, TurtleBot3/teleop style:
  w or i: toggle forward component
  s or ,: toggle backward component
  j/l: toggle strafe left/right component
  a/d: toggle yaw left/right component
  Combine keys sequentially, e.g. w then a for a forward-left arc.
  Press the same component key again to clear only that component.
  space/x/k: stop all components
  Q or Esc: quit
"""


LINEAR_X_KEYS = {"w": 1.0, "i": 1.0, "s": -1.0, ",": -1.0}
LINEAR_Y_KEYS = {"j": 1.0, "l": -1.0}
ANGULAR_Z_KEYS = {"a": 1.0, "d": -1.0}


def toggle_component(current_value, next_value):
    return 0.0 if current_value == next_value else next_value


class KeyboardDrive(Node):
    def __init__(self):
        super().__init__("keyboard_drive")
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("linear_speed", 0.20)
        self.declare_parameter("lateral_speed", 0.20)
        self.declare_parameter("angular_speed", 0.60)
        self.declare_parameter("publish_rate", 10.0)

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.lateral_speed = float(self.get_parameter("lateral_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)

    def handle_key(self, key):
        if key in LINEAR_X_KEYS:
            self.linear_x = toggle_component(self.linear_x, LINEAR_X_KEYS[key] * self.linear_speed)
        elif key in LINEAR_Y_KEYS:
            self.linear_y = toggle_component(self.linear_y, LINEAR_Y_KEYS[key] * self.lateral_speed)
        elif key in ANGULAR_Z_KEYS:
            self.angular_z = toggle_component(self.angular_z, ANGULAR_Z_KEYS[key] * self.angular_speed)
        elif key in (" ", "x", "k"):
            self.stop()
        elif key in ("Q", "\x1b"):
            return False
        return True

    def stop(self):
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        self.publish()

    def publish(self):
        twist = Twist()
        twist.linear.x = self.linear_x
        twist.linear.y = self.linear_y
        twist.angular.z = self.angular_z
        self.publisher.publish(twist)


def read_key(timeout_sec):
    readable, _, _ = select.select([sys.stdin], [], [], timeout_sec)
    if not readable:
        return None
    return sys.stdin.read(1)


def main():
    if not sys.stdin.isatty():
        print("keyboard_drive requires an interactive terminal.", file=sys.stderr)
        return 1

    old_settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = KeyboardDrive()

    try:
        tty.setcbreak(sys.stdin.fileno())
        print(HELP_TEXT)
        timeout_sec = 1.0 / node.publish_rate
        keep_running = True
        while rclpy.ok() and keep_running:
            key = read_key(timeout_sec)
            if key is not None:
                keep_running = node.handle_key(key)
            node.publish()
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSAFLUSH, old_settings)
        try:
            node.stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
