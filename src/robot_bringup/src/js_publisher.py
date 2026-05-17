#!/usr/bin/env python3
"""Reads /dev/input/js0 (Linux joystick API) and publishes sensor_msgs/Joy.
Uses the same interface as the move_basic C++ code — known to work with this controller."""
import struct
import array
import fcntl
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

# From linux/joystick.h
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS   = 0x02
JS_EVENT_INIT   = 0x80
JSIOCGAXES      = 0x80016a11
JSIOCGBUTTONS   = 0x80016a12

class JsPublisher(Node):
    def __init__(self):
        super().__init__('js_publisher')
        self.declare_parameter('dev', '/dev/input/js0')
        self.declare_parameter('autorepeat_rate', 20.0)

        dev = self.get_parameter('dev').get_parameter_value().string_value
        rate = self.get_parameter('autorepeat_rate').get_parameter_value().double_value

        self.pub = self.create_publisher(Joy, 'joy', 10)

        self.fd = os.open(dev, os.O_RDONLY | os.O_NONBLOCK)

        # Get axis and button counts from driver
        buf = array.array('B', [0])
        fcntl.ioctl(self.fd, JSIOCGAXES, buf)
        self.n_axes = buf[0]
        fcntl.ioctl(self.fd, JSIOCGBUTTONS, buf)
        self.n_buttons = buf[0]

        self.axes    = [0.0] * self.n_axes
        self.buttons = [0]   * self.n_buttons

        self.get_logger().info(f'Opened {dev}: {self.n_axes} axes, {self.n_buttons} buttons')

        self.timer = self.create_timer(1.0 / rate, self.timer_cb)

    def timer_cb(self):
        # Drain all pending events
        while True:
            try:
                data = os.read(self.fd, 8)
            except BlockingIOError:
                break
            if len(data) < 8:
                break
            _, value, etype, number = struct.unpack('IhBB', data)
            is_init = bool(etype & JS_EVENT_INIT)
            etype &= ~JS_EVENT_INIT
            if is_init:
                continue  # ignore hardware defaults; start all axes at 0 like move_basic
            if etype == JS_EVENT_AXIS and number < self.n_axes:
                # Negate axis 1 (Y) so stick-up = positive, matching move_basic convention
                sign = -1.0 if number == 1 else 1.0
                self.axes[number] = sign * value / 32767.0
            elif etype == JS_EVENT_BUTTON and number < self.n_buttons:
                self.buttons[number] = int(value)

        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'joy'
        msg.axes    = self.axes[:]
        msg.buttons = self.buttons[:]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JsPublisher()
    try:
        rclpy.spin(node)
    finally:
        os.close(node.fd)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
