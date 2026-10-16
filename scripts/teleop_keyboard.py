#!/usr/bin/env python3
"""Omni-directional Mecanum Chassis Keyboard Teleoperation.

Controls:
    W / S : Forward / Backward (linear.x)
    A / D : Strafe Left / Right (linear.y)
    Q / E : Turn Left / Right  (angular.z)
    SPACE / X : Stop immediately
    1 / 2 / 3 : Set speed gear (Low / Medium / High)
    + / -     : Fine-tune linear speed
    M         : Toggle Hold mode vs Latch mode
    Ctrl+C    : Exit safely (publishes stop command)
"""
import sys
import select
import termios
import tty
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

HELP_MSG = """
======================================================
     全向底盘键盘控制 (Omni Teleop Keyboard)
======================================================
  [平移]                [旋转]
    W                     Q      E
  A   D                (左转)  (右转)
    S

  W : 前进 (Forward)       Q : 原地左转 (Turn Left)
  S : 后退 (Backward)      E : 原地右转 (Turn Right)
  A : 左移 (Strafe Left)   D : 右移 (Strafe Right)

  SPACE / X : 急停 (Stop)
  1 / 2 / 3 : 切换档位 (低速 0.15 / 中速 0.30 / 高速 0.45 m/s)
  + / -     : 微调线速度 (+/- 0.05 m/s)
  M         : 切换模式 [按住保持(Hold) / 巡航锁定(Latch)]
  Ctrl+C    : 安全退出
======================================================
"""

SPEED_GEARS = {
    '1': (0.15, 0.30),
    '2': (0.30, 0.60),
    '3': (0.45, 0.80),
}


class OmniTeleop(Node):
    def __init__(self):
        super().__init__('omni_teleop_keyboard')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.declare_parameter('default_gear', '2')
        self.linear_speed, self.angular_speed = SPEED_GEARS['2']
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.mode = 'HOLD'  # 'HOLD' or 'LATCH'
        self.last_key_time = time.time()
        self.timer = self.create_timer(0.05, self.publish_loop)

    def update_key(self, key):
        now = time.time()
        self.last_key_time = now

        if key in ('w', 'W'):
            self.vx = self.linear_speed
        elif key in ('s', 'S'):
            self.vx = -self.linear_speed
        elif key in ('a', 'A'):
            self.vy = self.linear_speed
        elif key in ('d', 'D'):
            self.vy = -self.linear_speed
        elif key in ('q', 'Q'):
            self.wz = self.angular_speed
        elif key in ('e', 'E'):
            self.wz = -self.angular_speed
        elif key in (' ', 'x', 'X'):
            self.vx = 0.0
            self.vy = 0.0
            self.wz = 0.0
        elif key in SPEED_GEARS:
            self.linear_speed, self.angular_speed = SPEED_GEARS[key]
            if abs(self.vx) > 1e-4:
                self.vx = (1.0 if self.vx > 0 else -1.0) * self.linear_speed
            if abs(self.vy) > 1e-4:
                self.vy = (1.0 if self.vy > 0 else -1.0) * self.linear_speed
            if abs(self.wz) > 1e-4:
                self.wz = (1.0 if self.wz > 0 else -1.0) * self.angular_speed
        elif key in ('+', '='):
            self.linear_speed = min(0.50, round(self.linear_speed + 0.05, 2))
            self.angular_speed = min(0.80, round(self.angular_speed + 0.10, 2))
        elif key in ('-', '_'):
            self.linear_speed = max(0.05, round(self.linear_speed - 0.05, 2))
            self.angular_speed = max(0.10, round(self.angular_speed - 0.10, 2))
        elif key in ('m', 'M'):
            self.mode = 'LATCH' if self.mode == 'HOLD' else 'HOLD'
            if self.mode == 'HOLD':
                self.vx = self.vy = self.wz = 0.0

    def publish_loop(self):
        # In HOLD mode, stop if no key received for 0.35 seconds
        if self.mode == 'HOLD' and (time.time() - self.last_key_time > 0.35):
            self.vx = 0.0
            self.vy = 0.0
            self.wz = 0.0

        msg = Twist()
        msg.linear.x = float(self.vx)
        msg.linear.y = float(self.vy)
        msg.angular.z = float(self.wz)
        self.pub.publish(msg)

    def print_status(self):
        status = (f"\r[模式: {self.mode:5s}] 速度上限: {self.linear_speed:.2f} m/s, {self.angular_speed:.2f} rad/s | "
                  f"当前指令: vx={self.vx:+.2f}, vy={self.vy:+.2f}, wz={self.wz:+.2f}    ")
        sys.stdout.write(status)
        sys.stdout.flush()


def get_key(settings, timeout=0.05):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = OmniTeleop()
    print(HELP_MSG)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            key = get_key(settings, timeout=0.04)
            if key:
                if key == '\x03':  # Ctrl+C
                    break
                node.update_key(key)
            node.print_status()
    except Exception as exc:
        print(f"\nTeleop error: {exc}")
    finally:
        # Publish final zero twist
        stop_msg = Twist()
        node.pub.publish(stop_msg)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("\n[Teleop] 已发送停止指令并安全退出。")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
