#!/usr/bin/env python3
import sys
import socket
import time

import rclpy
from rclpy.node import Node
from bar30_depth.msg import Depth

from bluerov_bridge import Bridge

FLUID_DENSITY = {'fresh': 9.97, 'salt': 10.29}


class Bar30DepthNode(Node):
    def __init__(self):
        super().__init__('bar30_depth_node')

        # Declare parameters
        self.declare_parameter('device', 'udp:192.168.2.1:14552')
        self.declare_parameter('water', 'salt')

        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.water = self.get_parameter('water').get_parameter_value().string_value

        self.bridge = None
        self.depth_pub = self.create_publisher(Depth, '/bar30/depth/raw', 10)

        # Connect to bridge
        self.connect_to_bridge()

        # Create timer for 100Hz publishing
        self.timer = self.create_timer(0.01, self.timer_callback)

    def connect_to_bridge(self):
        while rclpy.ok():
            try:
                self.bridge = Bridge(self.device, write=False)
                break
            except socket.error:
                self.get_logger().error(
                    f'Failed to make mavlink connection to device {self.device}')
                time.sleep(1.0)

        if self.bridge is not None:
            self.bridge.update()

    def timer_callback(self):
        if self.bridge is None:
            return

        msg = self.bridge.get_msg(type='SCALED_PRESSURE2')
        if msg is not None:
            d = Depth()
            d.header.stamp = self.get_clock().now().to_msg()
            d.time = float(msg.time_boot_ms / 1000.0)
            d.pressure_abs = float(msg.press_abs)
            d.pressure_diff = float(msg.press_diff)
            d.temperature = float(msg.temperature / 100.0)
            # Assume pressure_diff is temperature compensated
            # https://github.com/bluerobotics/ardusub/blob/978cd64a1e3b0cb5ba1f3bcc995fcc39bea7e9ff/libraries/AP_Baro/AP_Baro_MS5611.cpp#L481
            # https://github.com/bluerobotics/ms5837-python/blob/c83bdc969ea1654a2e2759783546245709bd9914/ms5837.py#L146
            d.depth = float(d.pressure_diff / (FLUID_DENSITY[self.water] * 9.80665))
            self.depth_pub.publish(d)


def main(args=None):
    rclpy.init(args=args)
    node = Bar30DepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
